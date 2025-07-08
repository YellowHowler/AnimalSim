using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UtilFunctions;

public class PhysicsBody : MonoBehaviour
{
    [HideInInspector] public Vector3 position
    {
        get { return transform.position; }
        set { transform.position = value; }
    }
    
    [HideInInspector] public Quaternion rotation
    {
        get { return transform.rotation; }
        set { transform.rotation = value; }
    }
    
    [HideInInspector] public Vector3 velocity;
    private const float minSpeed = 0.01f;  // minimum allowed speed

    [HideInInspector] public Vector3 angularVelocity;
    private const float minAngularSpeed = 0.001f;  // minimum allowed angular speed

    [HideInInspector] public Vector3 force;
    [HideInInspector] public Vector3 torque;

    [HideInInspector] public Vector3 acceleration;
    [HideInInspector] public Vector3 angularAcceleration;

    public float minSeparationDistance = 1.0f;  // safe distance between centers
    [HideInInspector] public float collisionStiffness = 50.0f;             // repulsion strength (like a spring constant)

    public bool isKinematic = true;
    public float mass = 1.0f;

    public Vector3 gravity = new Vector3(0, -3f, 0);
    private float dt;

    public void Awake()
    {
        dt = Time.fixedDeltaTime;

        acceleration = Vector3.zero;
        angularAcceleration = Vector3.zero;
        force = Vector3.zero;
        torque = Vector3.zero;

        if (!isKinematic) mass = Mathf.Infinity;

        // Create trigger collider if none exists
        BoxCollider box = gameObject.GetOrAddComponent<BoxCollider>();
        box.isTrigger = true;

        // Create rigidbody if none exists
        Rigidbody rb = gameObject.GetOrAddComponent<Rigidbody>();
        rb.useGravity = false;
    }

    public void Start()
    {
        
    }

    public void Update()
    {
        dt = Time.deltaTime;

        AddForce(mass * gravity);
    }

    public void LateUpdate()
    {
        acceleration = force / mass;
        velocity += acceleration * dt;
        velocity = ClampMagnitude(velocity, minSpeed);
        position += velocity * dt;

        angularAcceleration = torque / mass;
        angularVelocity += angularAcceleration * dt;
        angularVelocity = ClampMagnitude(angularVelocity, minAngularSpeed);
        float angleDeg = angularVelocity.magnitude * Mathf.Rad2Deg * dt;
        if (angleDeg != 0)
        {
            Quaternion deltaRotation = Quaternion.AngleAxis(angleDeg, angularVelocity.normalized);
            rotation = deltaRotation * rotation;
        }

        force = Vector3.zero;
        torque = Vector3.zero;
    }
    
    public void AddForce(Vector3 force, Vector3 relativePosition)
    {
        if (!isKinematic) return;

        // If relative position is zero, just apply full linear force
        if (relativePosition == Vector3.zero)
        {
            this.force += force;
            return;
        }

        // Decompose force
        Vector3 r = relativePosition;
        Vector3 rNorm = r.normalized;

        Vector3 forceParallel = Vector3.Dot(force, rNorm) * rNorm;
        Vector3 forcePerpendicular = force - forceParallel;

        // Linear acceleration from parallel force
        this.force += forceParallel;

        // Torque from perpendicular force
        Vector3 torque = Vector3.Cross(r, forcePerpendicular);
        AddTorque(torque);
    }

    public void AddForce(Vector3 force)
    {
        AddForce(force, Vector3.zero);
    }

    public void AddTorque(Vector3 torque)
    {
        if (!isKinematic) return;

        this.torque += torque;
    }

    public void OnTriggerStay(Collider other)
    {
        PhysicsBody otherBody = other.GetComponent<PhysicsBody>();
        if (otherBody == null) return;

        Collider myCollider = GetComponent<Collider>();

        if (Physics.ComputePenetration(
            myCollider, transform.position, transform.rotation,
            other, other.transform.position, other.transform.rotation,
            out Vector3 normal, out float penetrationDepth))
        {
            if (penetrationDepth <= 1e-6f) return; // tiny overlap, ignore

            // Position correction to avoid penetration
            float invMassA = float.IsInfinity(mass) ? 0f : 1f / mass;
            float invMassB = float.IsInfinity(otherBody.mass) ? 0f : 1f / otherBody.mass;
            float totalInvMass = invMassA + invMassB;

            if (totalInvMass > 0f)
            {
                float correctionA = invMassA / totalInvMass;
                float correctionB = invMassB / totalInvMass;

                Vector3 correction = penetrationDepth * normal;

                if (!float.IsInfinity(mass))
                    transform.position += correctionA * correction;

                if (!float.IsInfinity(otherBody.mass))
                    other.transform.position -= correctionB * correction;
            }

            // Zero out velocity towards each other
            Vector3 relativeVelocity = velocity - otherBody.velocity;
            float approachSpeed = Vector3.Dot(relativeVelocity, normal);

            if (approachSpeed < 0f)
            {
                Vector3 impulse = approachSpeed * normal;
                if (totalInvMass > 0f)
                {
                    float ratioA = invMassA / totalInvMass;
                    float ratioB = invMassB / totalInvMass;

                    if (!float.IsInfinity(mass))
                        velocity -= ratioA * impulse;

                    if (!float.IsInfinity(otherBody.mass))
                        otherBody.velocity += ratioB * impulse;
                }
            }

            // Compute spring-like repulsion force
            float forceMagnitude = penetrationDepth * collisionStiffness;
            Vector3 force = forceMagnitude * normal;

            // Apply equal and opposite forces
            AddForce(force);
            otherBody.AddForce(-force);
        }
    }
}