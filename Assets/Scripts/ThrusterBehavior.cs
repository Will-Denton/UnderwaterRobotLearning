using UnityEngine;

[DisallowMultipleComponent]
[AddComponentMenu("T200 Thruster")]
public class T200Thruster16V : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Vehicle rigidbody")]
    public Rigidbody vehicleRigidbody;

    [Header("Command")]
    [Tooltip("Normalized command in [-1, 1]. 1=fwd, -1=rev. Set via SetCommand().")]
    [Range(-1f, 1f)] public float command = 0f;


    // 16 V datasheet thrusts (kgf * 9.80665 N/kgf)
    const float FWD_MAX_N = 5.25f * 9.80665f; // ≈ 51.49 N
    const float REV_MAX_N = 4.10f * 9.80665f; // ≈ 40.20 N

    void Reset()
    {
        if (vehicleRigidbody == null)
            vehicleRigidbody = GetComponentInParent<Rigidbody>();
    }

    void OnValidate()
    {
        if (vehicleRigidbody == null)
            vehicleRigidbody = GetComponentInParent<Rigidbody>();
    }

    public void SetCommand(float uNorm) => command = Mathf.Clamp(uNorm, -1f, 1f);

    void FixedUpdate()
    {
         if (!vehicleRigidbody) return;

        float clampedCommand = Mathf.Clamp(command, -1f, 1f);
        float maxN = clampedCommand >= 0f ? FWD_MAX_N : REV_MAX_N;

        vehicleRigidbody.AddForceAtPosition(transform.forward * clampedCommand * maxN, transform.position, ForceMode.Force);
    }

#if UNITY_EDITOR
    void OnDrawGizmos()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position, transform.position + transform.forward * 0.3f);
        Gizmos.DrawSphere(transform.position, 0.015f);
    }
#endif
}