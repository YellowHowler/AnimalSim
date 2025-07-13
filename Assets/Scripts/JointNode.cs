using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UtilFunctions;

public class JointNode : PhysicsBody
{
    [Header("Physical Properties")]
    public Transform body;
    private JointNode parentJoint;

    private float restLength;

    private float stiffness = 200f;
    private float damping = 5f;


    [Header("Angular Limits (degrees)")]
    public bool constraintRotation = false;
    public Vector2 yawLimits = new Vector2(-45f, 45f);   // Y-axis
    public Vector2 pitchLimits = new Vector2(-30f, 60f); // X-axis


    [Header("Target Angles (degrees)")]
    public bool useTargetAngle = false;
    public float targetYaw = 0f;
    public float targetPitch = 0f;

    private float targetAngleStiffness = 20f; // steering gain
    private float targetAngleDamping = 2f;    // damp oscillation


    [Header("Child Joint Visualization")]
    List<ChildJoint> childJoints;
    public Material lineMaterial;
    public Color lineColor = Color.green;
    public float lineWidth = 0.5f;
    private LineRenderer[] lines;


    public struct ChildJoint
    {
        public JointNode joint;
        public float restLength;

        public LineRenderer lr;

        public ChildJoint(JointNode joint, float restLength,
                          LineRenderer lr)
        {
            this.joint = joint;
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
            childJoints.Add(new ChildJoint(childJoint, restLength, lr));
        }
    }

    public void Start()
    {
        base.Start();

        InitChildJoints();

        parentJoint = null;
        if (transform.parent != null)
            parentJoint = transform.parent.GetComponent<JointNode>();
    }

    public void Update()
    {
        transform.parent = body;

        base.Update();

        if (transform.childCount > transform.NumChildrenWithComponent<LineRenderer>()) return;

        ApplyTargetAngleSteering();

        int solverIterations = 5;
        for (int iter = 0; iter < solverIterations; iter++)
        {
            foreach (ChildJoint childJoint in childJoints)
            {
                // 1. Enforce distance constraint
                ConstraintDistance(childJoint);

                // 2. Enforce angular constraint
                ConstraintAngle(childJoint.joint);

                childJoint.lr.SetPosition(0, childJoint.joint.position);
                childJoint.lr.SetPosition(1, position);
            }
        }
    }

    public void OnTriggerStay(Collider other)
    {
        base.OnTriggerStay(other);
    }

    private bool IsValidVector(Vector3 v)
    {
        return !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z));
    }

    public void ConstraintDistance(ChildJoint other)
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

    public void ConstraintPlane(ChildJoint other)
    {
        JointNode otherJoint = other.joint;

        // Define plane from this (parent) joint
        Vector3 planeOrigin = position;
        Vector3 planeNormal = transform.up; // The normal is local X

        // Move child to the projection
        Vector3 toOther = otherJoint.position - planeOrigin;
        float distanceFromPlane = Vector3.Dot(toOther, planeNormal);
        Vector3 projectedPos = otherJoint.position - distanceFromPlane * planeNormal;
        otherJoint.position = projectedPos;
    }

    private Vector3 ProjectForceWithinAngularLimits(Vector3 force)
    {
        if (parentJoint == null) return force;

        Vector3 toChild = position - parentJoint.position;
        Vector3 localDir = Quaternion.Inverse(parentJoint.rotation) * toChild.normalized;

        // Convert to spherical angles
        float yaw = Mathf.Atan2(localDir.x, localDir.z) * Mathf.Rad2Deg;
        float pitch = Mathf.Asin(localDir.y) * Mathf.Rad2Deg;

        // If within bounds, keep full force
        if (yaw >= yawLimits.x && yaw <= yawLimits.y &&
            pitch >= pitchLimits.x && pitch <= pitchLimits.y)
            return force;

        // Otherwise, remove the component of force pushing it further out
        Vector3 tangent = Vector3.Cross(toChild, Vector3.up).normalized;
        return Vector3.ProjectOnPlane(force, toChild.normalized);
    }

    public void ConstraintAngle(JointNode child)
    {
        if (!child.constraintRotation) return;
        if (child == null || parentJoint == null) return;

        Vector3 pParent = this.position;
        Vector3 pChild = child.position;

        Vector3 worldDelta = pChild - pParent;
        float restLength = worldDelta.magnitude;
        if (restLength < 1e-6f || float.IsNaN(restLength)) return;

        // Convert direction to parent local frame
        Quaternion parentRotInv = Quaternion.Inverse(this.rotation);
        if (float.IsNaN(parentRotInv.x)) return; // safety check for broken rotation

        Vector3 localDir = parentRotInv * worldDelta.normalized;
        if (!IsValidVector(localDir)) return;

        float yaw = Mathf.Atan2(localDir.x, localDir.z) * Mathf.Rad2Deg;
        float pitch = Mathf.Atan2(localDir.y, new Vector2(localDir.x, localDir.z).magnitude) * Mathf.Rad2Deg;

        // Check for violation
        float clampedYaw = Mathf.Clamp(yaw, child.yawLimits.x, child.yawLimits.y);
        float clampedPitch = Mathf.Clamp(pitch, child.pitchLimits.x, child.pitchLimits.y);

        if (Mathf.Approximately(yaw, clampedYaw) && Mathf.Approximately(pitch, clampedPitch))
            return; // within bounds

        // Convert clamped yaw/pitch back to a unit local vector
        float yawRad = clampedYaw * Mathf.Deg2Rad;
        float pitchRad = clampedPitch * Mathf.Deg2Rad;

        Vector3 clampedLocalDir = new Vector3(
            Mathf.Sin(yawRad) * Mathf.Cos(pitchRad),
            Mathf.Sin(pitchRad),
            Mathf.Cos(yawRad) * Mathf.Cos(pitchRad)
        );

        if (!IsValidVector(clampedLocalDir)) return;

        // Convert to world space
        Vector3 targetWorldDir = this.rotation * clampedLocalDir;
        if (!IsValidVector(targetWorldDir)) return;

        Vector3 targetPosition = pParent + targetWorldDir * restLength;
        if (!IsValidVector(targetPosition)) return;

        // Apply symmetric position correction (split between parent and child)
        Vector3 correction = targetPosition - pChild;
        if (!IsValidVector(correction)) return;

        float wA = float.IsInfinity(this.mass) ? 0f : 1f / this.mass;
        float wB = float.IsInfinity(child.mass) ? 0f : 1f / child.mass;
        float wSum = wA + wB;
        if (wSum < 1e-6f) return;

        Vector3 correctionA = -correction * (wA / wSum);
        Vector3 correctionB =  correction * (wB / wSum);

        if (IsValidVector(correctionA) && !float.IsInfinity(this.mass))
            this.position += correctionA;

        if (IsValidVector(correctionB) && !float.IsInfinity(child.mass))
            child.position += correctionB;
    }

    public void ApplyTargetAngleSteering()
    {
        if (!useTargetAngle || parentJoint == null) return;

        // --- Compute vector from parent to child
        Vector3 toChild = position - parentJoint.position;
        float currentLength = toChild.magnitude;
        if (currentLength < 1e-6f) return;

        // --- Local direction from parent to child
        Vector3 localDir = Quaternion.Inverse(parentJoint.rotation) * toChild.normalized;

        // --- Yaw: horizontal angle in XZ plane
        Vector3 flatDir = new Vector3(localDir.x, 0f, localDir.z);
        float currentYaw = Vector3.SignedAngle(Vector3.forward, flatDir, Vector3.up);

        // --- Pitch: vertical angle from flatDir up to full localDir
        float currentPitch = Vector3.SignedAngle(flatDir.normalized, localDir, Vector3.right);

        // --- Compute angular error
        float yawError = Mathf.DeltaAngle(currentYaw, targetYaw);
        float pitchError = Mathf.DeltaAngle(currentPitch, targetPitch);

        // --- Convert yaw/pitch to direction
        float newYaw = currentYaw + yawError;
        float newPitch = currentPitch + pitchError;

        float yawRad = newYaw * Mathf.Deg2Rad;
        float pitchRad = newPitch * Mathf.Deg2Rad;

        Vector3 desiredLocalDir = new Vector3(
            Mathf.Sin(yawRad) * Mathf.Cos(pitchRad),
            Mathf.Sin(pitchRad),
            Mathf.Cos(yawRad) * Mathf.Cos(pitchRad)
        );

        Vector3 desiredWorldDir = parentJoint.rotation * desiredLocalDir;
        Vector3 targetPosition = parentJoint.position + desiredWorldDir * currentLength;

        // --- Predict future position for smoother behavior
        Vector3 predictedPosition = position + velocity * Time.deltaTime;
        Vector3 toTarget = targetPosition - predictedPosition;

        // --- Decompose into tangent and radial components
        Vector3 radial = (position - parentJoint.position).normalized;
        Vector3 radialComponent = Vector3.Project(toTarget, radial);
        Vector3 tangentComponent = Vector3.ProjectOnPlane(toTarget, radial);

        // --- Blend amount based on how far off we are
        float errorMagnitude = Mathf.Abs(pitchError) + Mathf.Abs(yawError);
        float radialRatio = Mathf.InverseLerp(0f, 60f, errorMagnitude);
        radialRatio = Mathf.Clamp(radialRatio, 0.3f, 1.0f); // force some upward drive

        Vector3 blendedDelta = tangentComponent + radialRatio * radialComponent;

        // --- PD controller
        float maxTargetSpeed = 2.0f;
        Vector3 desiredVelocity = Vector3.ClampMagnitude(blendedDelta * targetAngleStiffness, maxTargetSpeed);
        Vector3 correctiveVelocity = desiredVelocity - velocity;
        Vector3 correctiveAccel = correctiveVelocity * targetAngleStiffness;

        float maxAccel = 50.0f;
        correctiveAccel = Vector3.ClampMagnitude(correctiveAccel, maxAccel);

        // --- Apply force
        AddForce(correctiveAccel * mass);

        // --- Apply reaction to parent
        if (!float.IsInfinity(parentJoint.mass))
            parentJoint.AddForce(-correctiveAccel * mass);
    }



}
