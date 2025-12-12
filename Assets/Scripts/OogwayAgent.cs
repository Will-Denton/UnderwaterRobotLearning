using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class UnderwaterForwardAgent : Agent
{
    [Header("References")]
    public Rigidbody rb;
    public Transform target;
    [Tooltip("8 Thruster components")]
    public T200Thruster16V[] thrusters;

    // For reward shaping
    private float prevDistance;

    // Start pose
    private Vector3 startPosition;
    private Quaternion startRotation;

    public override void Initialize()
    {
        if (rb == null)
            rb = GetComponent<Rigidbody>();

        startPosition = transform.position;
        startRotation = transform.rotation;
    }

    public override void OnEpisodeBegin()
    {
        // Reset physics
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // Reset pose
        transform.position = startPosition;
        transform.rotation = startRotation;

        // Place target along +X axis at random distance
        if (target != null)
        {
            float dist = Random.Range(8f, 15f); // meters
            target.position = startPosition + new Vector3(dist, 0f, 0f);
            target.position = new Vector3(target.position.x, startPosition.y, startPosition.z);

            prevDistance = Vector3.Distance(transform.position, target.position);
        }
        else
        {
            prevDistance = 0f;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (rb == null || target == null)
        {
            sensor.AddObservation(new float[10]);
            return;
        }

        // Relative target position in local space
        Vector3 localTargetPos = transform.InverseTransformPoint(target.position);
        sensor.AddObservation(localTargetPos);

        // Local linear velocity
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        sensor.AddObservation(localVel);

        // Local angular velocity
        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
        sensor.AddObservation(localAngVel);

        // Uprightness
        float upDot = Vector3.Dot(transform.up.normalized, Vector3.up);
        sensor.AddObservation(upDot); 
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // --- Apply thruster commands ---
        
        var continuous = actions.ContinuousActions;

        // 8 continuous actions -> 8 thrusters
        float totalEffort = 0f;
        float verticalEffort = 0f;

        if (thrusters != null)
        {
            int n = Mathf.Min(thrusters.Length, continuous.Length);
            for (int i = 0; i < n; i++)
            {
                float cmd = Mathf.Clamp(continuous[i], -1f, 1f);
                thrusters[i].SetCommand(cmd);

                float sq = cmd * cmd;
                totalEffort += sq;

                if (i < 4)
                    verticalEffort += sq;
            }
        }

        if (target == null)
            return;

        // --- Reward shaping ---

        // REWARD 1: Progress toward target
        float distance = Vector3.Distance(transform.position, target.position);
        float distanceReduction = prevDistance - distance;
        AddReward(distanceReduction);

        // REWARD 2: Small time penalty to encourage efficiency
        AddReward(-0.0005f);

        // REWARD 3: Uprightness
        float upDot = Vector3.Dot(transform.up.normalized, Vector3.up);
        float uprightTolerance = 0.985f; // Allow up to about ~10° tilt before reward starts dropping as cos(10°) ≈ 0.9848

        float tiltPenalty = 0f;
        if (upDot < uprightTolerance)
        {
            float violation = (uprightTolerance - upDot) / uprightTolerance; // How far below the tolerance we are, normalized to [0,1]
            violation = Mathf.Clamp01(violation);
            tiltPenalty = violation; // Penalize more as we tilt more
        }
        AddReward(-0.01f * tiltPenalty);

        // REWARD 4: Heading toward target
        Vector3 toTarget = target.position - transform.position;
        Vector3 toTargetXZ = Vector3.ProjectOnPlane(toTarget, Vector3.up);
        Vector3 forwardXZ  = Vector3.ProjectOnPlane(transform.right, Vector3.up); // +X is forward

        if (toTargetXZ.sqrMagnitude > 1e-4f && forwardXZ.sqrMagnitude > 1e-4f)
        {
            float headingDot = Vector3.Dot(forwardXZ.normalized, toTargetXZ.normalized); // [-1, 1]
            float headingReward = Mathf.Max(0f, headingDot);
            AddReward(0.01f * headingReward);
        }

        // REWARD 5: Effort penalty to encourage efficiency
        AddReward(-0.0002f * totalEffort);
        AddReward(-0.001f * verticalEffort);

        // Success condition
        if (distance < 1.0f)
        {
            AddReward(1.0f);
            EndEpisode();
            return;
        }

        // Bounds check
        Vector3 p = transform.position;
        if (Mathf.Abs(p.x) > 50f || Mathf.Abs(p.z) > 50f || p.y < -20f || p.y > 20f)
        {
            AddReward(-1.0f);
            EndEpisode();
            return;
        }

        prevDistance = distance;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuous = actionsOut.ContinuousActions;
        for (int i = 0; i < continuous.Length; i++)
            continuous[i] = 0f;
    }
}
    