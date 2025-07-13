using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UtilFunctions;

public class SwordControl : MonoBehaviour
{
    public Transform swordTarget; //TEMP
    public Transform swordTarget2;
    public Transform sword;

    public float swordMass = 1.0f;

    public Transform projectionRef;
    public float armLength = 1.6f;
    public Transform arm1Target;
    public Transform arm2Target;
    public Transform shoulder1;
    public Transform shoulder2;
    public float swordOffset = 0.05f;

    public Vector2 xRange = new Vector2(-2, 2);
    public Vector2 yRange = new Vector2(-2, 2);

    private float screenToSwordRatio = 7f;

    private Vector2 prevDragPos;
    private float dragTime = 0;
    // private Vector3 prevDeltaWorld;
    // private Vector3 startSwordPos;
    // private Vector3 swordPos;
    // private Vector3 targetSwordPos;

    // private float dt;
    // private float dragTime;
    // private Vector3 swordVelocity = Vector3.zero;
    // private float swordAccel = 40;
    // private float swordMaxAccel = 50;

    private Vector2 swordForce;
    private float swordMaxForce = 300;
    private Vector2 swordAcc;
    private Vector2 swordVel;
    private Vector2 swordPos;

    private Vector2 swordTargetPos;
    private Vector2 prevSwordTargetPos;

    private float dt;

    private Vector2 totDeltaWorld;
    private Vector2 totCurForce;

    private Vector2 deltaWorld;

    // Start is called before the first frame update
    void Start()
    {
        //Time.timeScale = 0.3f;
        prevSwordTargetPos = Vector2.zero;
        swordTargetPos = Vector2.zero;
        swordForce = Vector2.zero;
        swordAcc = Vector2.zero;
        swordVel = Vector2.zero;
        swordPos = Vector2.zero;

        totDeltaWorld = Vector2.zero;
        totCurForce = Vector2.zero;

        prevDragPos = new Vector2(Input.mousePosition.x, Input.mousePosition.y);

        dt = 0.001f;
    }

    private Vector2 ScreenToWorld(Vector2 screenPos)
    {
        screenPos = new Vector2(screenPos.x - Screen.width / 2, screenPos.y - Screen.height / 2);
        Vector2 worldPos = new Vector2(screenPos.x / Screen.width, screenPos.y / Screen.height) * screenToSwordRatio;

        return worldPos;
    }

    private void FixedUpdate()
    {
        dt = Time.fixedDeltaTime;

        swordForce = Vector2.zero;

        Vector2 screenPos = new Vector2(Input.mousePosition.x, Input.mousePosition.y);
        prevSwordTargetPos = swordTargetPos;
        swordTargetPos = ScreenToWorld(screenPos);

        ClampSwordTarget();
        StepSword();
    }

    private Vector2 DeltaWorldToForce(Vector2 deltaWorld)
    {
        return deltaWorld * 500;
    }

    private void AddForce(Vector2 force)
    {
        swordForce += force;
    }

    private void ClampSwordTarget()
    {
        swordTargetPos = Vector2.ClampMagnitude(swordTargetPos, 4);
        swordPos = Vector2.ClampMagnitude(swordPos, 4.2f);
    }

    private void StepSword()
    {
        Vector2 toTarget = swordTargetPos - swordPos;
        float dist = toTarget.magnitude;

        float dirSimilarity = Vector2.Dot(toTarget.normalized, swordVel.normalized);
        swordForce = toTarget * 20;
        //swordForce *= Mathf.Clamp(0.05f / Mathf.Pow(dist, 8), 1, 1000);

        float swordTargetAcc = ((swordTargetPos - prevSwordTargetPos).magnitude) / dt * 15;

        swordAcc = swordForce / swordMass;
        // Vector2 velDir = swordVel.normalized;
        // if (swordVel.magnitude < 1e-5f) velDir = Vector3.zero;
        // float accAlongVel = Vector3.Dot(swordAcc, velDir);
        // float maxAcc = swordTargetAcc / dt;
        // float clampedAccAlongVel = Mathf.Min(accAlongVel, maxAcc);
        // Vector2 accParallel = clampedAccAlongVel * velDir;
        // Vector2 accPerpendicular = swordAcc - (accAlongVel * velDir);
        // swordAcc = accParallel + accPerpendicular;

        // Apply force
        swordVel += swordAcc * dt;
        swordPos += swordVel * dt;

        if (Vector2.Distance(swordTargetPos, swordPos) < 0.1f)
        {
            print("snap");
            swordPos = swordTargetPos;
            swordVel = Vector2.zero;
            swordForce = Vector2.zero;
        }

        // print("swordPos" + swordPos);
        // print("swordVel" + swordVel);

        MoveObjects();
    }

    private void MoveObjects()
    {
        float handOffset = 0.2f;

        Vector3 offset = new Vector3(0, projectionRef.localPosition.y, swordOffset);
        Vector3 targetPos = (Vector3)swordPos + offset;
        float radius = Mathf.Abs(projectionRef.localPosition.z + swordOffset);
        targetPos = (targetPos - projectionRef.localPosition).normalized * radius + projectionRef.localPosition;

        arm1Target.localPosition = targetPos;
        arm2Target.localPosition = targetPos;

        swordTarget.localPosition = (Vector3)swordTargetPos + offset;
        swordTarget2.localPosition = (Vector3)swordPos + offset;
    }
    
    private void OnDrawGizmos()
    {
        if (swordTarget != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(swordTarget.position, swordTarget.position + (Vector3)swordForce);

            Gizmos.color = Color.blue;
            Gizmos.DrawLine(swordTarget.position, swordTarget.position + 5 * (Vector3)swordVel);

            Gizmos.color = Color.green;
            Gizmos.DrawLine(swordTarget.position, swordTarget.position + 10 * (Vector3)deltaWorld);
        }
    }
}
