using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UtilFunctions;

public class JointNode : PhysicsBody
{
    public Transform parent;

    private float restLength;

    public float stiffness = 200f;
    public float damping = 5f;

    List<ChildJoint> childJoints;
    public Material lineMaterial;
    public Color lineColor = Color.green;
    public float lineWidth = 0.5f;
    private LineRenderer[] lines;

    public struct ChildJoint
    {
        public JointNode joint;
        public JointNode parentJoint;
        public float restLength;

        public LineRenderer lr;

        public ChildJoint(JointNode joint, JointNode parentJoint, float restLength,
                          LineRenderer lr)
        {
            this.joint = joint;
            this.parentJoint = parentJoint;
            this.restLength = restLength;

            this.lr = lr;
        }
    }

    public void Awake()
    {
        base.Awake();
    }

    private void InitChildJoints()
    {
        childJoints = new List<ChildJoint>();

        Transform[] children = GetComponentsInChildren<Transform>();
        foreach (Transform child in children)
        {
            if (child == transform || child.parent != transform) continue;

            JointNode childJoint = child.GetComponent<JointNode>();
            if (childJoint == null) continue;

            float restLength = Vector3.Distance(position, childJoint.position);

            // Draw a line between joint and child joint
            GameObject lineObj = new GameObject("Line");
            lineObj.transform.parent = transform;
            LineRenderer lr = lineObj.AddComponent<LineRenderer>();
            lr.material = lineMaterial != null ? lineMaterial : new Material(Shader.Find("Sprites/Default"));
            lr.startColor = lineColor;
            lr.endColor = lineColor;
            lr.startWidth = lineWidth;
            lr.endWidth = lineWidth;
            lr.positionCount = 2;
            lr.useWorldSpace = true;

            // Create ChildJoint
            childJoints.Add(new ChildJoint(childJoint, this, restLength, lr));
        }
    }

    public void Start()
    {
        base.Start();

        InitChildJoints();
    }

    public void Update()
    {
        transform.parent = parent;

        base.Update();

        if (transform.childCount > transform.NumChildrenWithComponent<LineRenderer>()) return;

        foreach (ChildJoint childJoint in childJoints)
        {
            childJoint.joint.rotation = Quaternion.LookRotation(position - childJoint.joint.position);

            ConstrainDistance(childJoint);

            childJoint.lr.SetPosition(0, position);
            childJoint.lr.SetPosition(1, childJoint.joint.position);
        }
    }

    public void OnTriggerStay(Collider other)
    {
        base.OnTriggerStay(other);
    }

    public void ConstrainDistance(ChildJoint other)
    {
    
        JointNode A = this;
        JointNode B = other.joint;

        Vector3 pa = A.position;
        Vector3 pb = B.position;

        Vector3 delta = pa - pb;
        float currentLength = delta.magnitude;
        if (currentLength < 1e-6f) return;

        Vector3 n = delta / currentLength;
        float C = currentLength - other.restLength;

        // Constraint point offsets from center of mass
        Vector3 ra = pa - A.position;
        Vector3 rb = pb - B.position;

        // Relative velocity at the constraint points
        Vector3 va = A.velocity + Vector3.Cross(A.angularVelocity, ra);
        Vector3 vb = B.velocity + Vector3.Cross(B.angularVelocity, rb);
        Vector3 relVel = va - vb;

        float Cdot = Vector3.Dot(relVel, n);

        // Baumgarte stabilization
        float beta = 0.2f;
        float b = beta * C / Time.deltaTime;

        // Inverse masses
        float invMassA = float.IsInfinity(A.mass) ? 0f : 1f / A.mass;
        float invMassB = float.IsInfinity(B.mass) ? 0f : 1f / B.mass;

        // For now, use moment of inertia = mass (scalar) for simplicity
        float invInertiaA = invMassA; // TODO: replace with real tensor later
        float invInertiaB = invMassB;

        // Effective mass of the constraint
        float angularEffectA = Vector3.Cross(ra, n).sqrMagnitude * invInertiaA;
        float angularEffectB = Vector3.Cross(rb, n).sqrMagnitude * invInertiaB;

        float effectiveMass = invMassA + invMassB + angularEffectA + angularEffectB;
        if (effectiveMass <= 1e-6f) return;

        // Lagrange multiplier (impulse magnitude)
        float lambda = -(Cdot + b) / effectiveMass;

        // Impulse to apply
        Vector3 impulse = lambda * n;

        // Apply linear impulses
        if (!float.IsInfinity(A.mass)) A.velocity += impulse * invMassA;
        if (!float.IsInfinity(B.mass)) B.velocity -= impulse * invMassB;
    }

    public void ConstrainPlane(ChildJoint other)
    {
        JointNode otherJoint = other.joint;
    
        // Define plane from this (parent) joint
        Vector3 planeOrigin = position;
        Vector3 planeNormal = transform.up; // The normal is local X

        // Move child to the projection
        Vector3 toOther = otherJoint.position - planeOrigin;
        float distanceFromPlane = Vector3.Dot(toOther, planeNormal);
        Vector3 projectedPos =  otherJoint.position - distanceFromPlane * planeNormal;
        otherJoint.position = projectedPos;
    }
}
