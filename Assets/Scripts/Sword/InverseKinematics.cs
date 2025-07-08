using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InverseKinematics : MonoBehaviour
{
    protected LineRenderer lr;

    //values
    //----------------------------------------------------------------

    //Transform references
    public Transform target;
    public Transform pole;
    [Range(0, 1)]
    public float poleWeight = 0.3f;
    protected Transform[] bones;
    //----------------------------------------------------------------


    //constants
    public int chainLength = 2;

    [HideInInspector] public float length;

    public bool useBezier = false;

    protected int iterations = 10;
    protected float delta = 0.001f;
    //----------------------------------------------------------------


    //variables
    protected Quaternion targetInitRot;
    protected Quaternion endInitRot;
    //----------------------------------------------------------------

    protected void Awake()
    {
        lr = GetComponent<LineRenderer>();
        lr.SetWidth(0.1f, 0.1f);

        bones = new Transform[chainLength + 1];

        targetInitRot = target.rotation;
        endInitRot = transform.rotation;

        var current = transform;

        length = 0;

        for (int i = chainLength - 1; i >= 0; i--)
        {
            length += (current.position - current.parent.position).magnitude;
            bones[i + 1] = current;
            bones[i] = current.parent;

            current = current.parent;
        }
        if (bones[0] == null)
            throw new UnityException("The chain value is longer than the ancestor chain!");
    }

    protected void Start()
    {
    }

    protected void LateUpdate()
    {
        UpdateJoints();
        DrawLimbLine();
    }

    protected void UpdateJoints()
    {
        if (bones == null || bones.Length == 0) return;

        Vector3 start = bones[0].position;
        Vector3 end = Vector3.ClampMagnitude(target.position - start, length) + start;

        if (!useBezier)
        {
            int numBones = bones.Length;

            // Store current joint positions
            Vector3[] positions = new Vector3[numBones];
            for (int i = 0; i < numBones; i++)
                positions[i] = bones[i].position;

            // FABRIK iterations
            for (int iteration = 0; iteration < iterations; iteration++)
            {
                // Backward pass
                positions[numBones - 1] = end;
                for (int i = numBones - 2; i >= 0; i--)
                {
                    float boneLength = Vector3.Distance(bones[i].position, bones[i + 1].position);
                    Vector3 dir = (positions[i] - positions[i + 1]).normalized;
                    positions[i] = positions[i + 1] + dir * boneLength;
                }

                // Forward pass
                positions[0] = start;
                for (int i = 1; i < numBones; i++)
                {
                    float boneLength = Vector3.Distance(bones[i].position, bones[i - 1].position);
                    Vector3 dir = (positions[i] - positions[i - 1]).normalized;
                    positions[i] = positions[i - 1] + dir * boneLength;
                }

                // Check if we're close enough to target
                if ((positions[numBones - 1] - end).sqrMagnitude < delta * delta)
                    break;
            }

            // Apply positions
            for (int i = 0; i < numBones; i++)
            {
                bones[i].position = positions[i];
            }
        }
        else
        {
            // Define the initial pole direction
            Vector3 initialPoleDir = (pole != null) 
            ? (pole.position - (start + end) * 0.5f).normalized
            : Vector3.up;

            // Solve for pole that matches arc length
            Vector3 bestPole = PoleSolver.FindPoleForLength(start, end, initialPoleDir, length);

            // Reposition bones along the curve
            for (int i = 0; i < bones.Length; i++)
            {
                float t = i / (float)(bones.Length - 1);
                Vector3 pos = BezierUtils.BezierPoint(start, bestPole, end, t);
                bones[i].position = pos;
            }
        }
    }

    protected void DrawLimbLine()
    {
        Vector3[] points = new Vector3[bones.Length];

        for (int i = 0; i < bones.Length; i++)
        {
            points[i] = bones[i].position;
        }

        lr.SetPositions(points);
    }
}

public static class BezierUtils
{
    public static Vector3 BezierPoint(Vector3 p0, Vector3 p1, Vector3 p2, float t)
    {
        return (1 - t) * (1 - t) * p0 + 2 * (1 - t) * t * p1 + t * t * p2;
    }

    public static float BezierArcLength(Vector3 p0, Vector3 p1, Vector3 p2, int samples = 50)
    {
        float length = 0f;
        Vector3 prev = p0;
        for (int i = 1; i <= samples; i++)
        {
            float t = i / (float)samples;
            Vector3 pt = BezierPoint(p0, p1, p2, t);
            length += Vector3.Distance(prev, pt);
            prev = pt;
        }
        return length;
    }
}

public static class PoleSolver
{
    public static Vector3 FindPoleForLength(Vector3 start, Vector3 end, Vector3 poleDirection, float targetLength, float tolerance = 0.001f, int maxIter = 20)
    {
        float minD = 0f;
        float maxD = Vector3.Distance(start, end) * 2f;
        Vector3 midPole = Vector3.zero;

        for (int i = 0; i < maxIter; i++)
        {
            float d = (minD + maxD) * 0.5f;
            midPole = (start + end) * 0.5f + poleDirection.normalized * d;

            float arcLength = BezierUtils.BezierArcLength(start, midPole, end);
            float error = arcLength - targetLength;

            if (Mathf.Abs(error) < tolerance)
                break;

            if (error < 0)
                minD = d;
            else
                maxD = d;
        }

        return midPole;
    }
}
