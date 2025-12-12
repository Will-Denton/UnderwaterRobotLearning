using UnityEngine;

[DisallowMultipleComponent]
[AddComponentMenu("Oogway/Buoyancy And Water Physics")]
public class BuoyancyAndWaterPhysics : MonoBehaviour
{
    [Header("References")]
    public Rigidbody rb;
    [Tooltip("Where buoyant force is applied. If null, uses the rigidbody's world center of mass.")]
    public Transform centerOfBuoyancy;

    [Header("Water / Buoyancy")]
    [Tooltip("Water density (kg/m^3).")]
    public float waterDensity = 1025f;

    [Tooltip("Upward boyuancy force (N)")]
    public float buoyancyOffsetNewton = 2f;

    [Header("Hydrodynamic Drag")]
    [Tooltip("Quadratic drag coefficients per local axis (N per (m/s)^2). X=surge, Y=sway, Z=heave")]
    public Vector3 quadDragCoefficients = new Vector3(60f, 80f, 120f);

    [Tooltip("Angular linear damping per local axis (N·m per rad/s).")]
    public Vector3 angularDamping = new Vector3(5f, 5f, 5f);

    [Header("Ambient Current")]
    [Tooltip("Constant world-space water current velocity (m/s).")]
    public Vector3 currentWorldVelocity = Vector3.zero;

    const float g = 9.81f;
    private float displacedVolume;

    void Reset()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
    }

    void OnValidate()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
        displacedVolume = rb.mass / waterDensity;
    }

    void FixedUpdate()
    {
        if (rb == null) return;

        // --- Buoyancy ---
        Vector3 buoyancy = Vector3.up * (waterDensity * g * displacedVolume + buoyancyOffsetNewton);
        Vector3 cob = (centerOfBuoyancy != null) ? centerOfBuoyancy.position : rb.worldCenterOfMass;

        rb.AddForceAtPosition(buoyancy, cob, ForceMode.Force);

        // --- Quadratic translational drag ---
        Vector3 vRelWorld = rb.linearVelocity - currentWorldVelocity;
        Vector3 vRelLocal = transform.InverseTransformDirection(vRelWorld);

        // F local = -sign(v) * C * v^2
        Vector3 fDragLocal = new Vector3(
            -Mathf.Sign(vRelLocal.x) * quadDragCoefficients.x * vRelLocal.x * vRelLocal.x,
            -Mathf.Sign(vRelLocal.y) * quadDragCoefficients.y * vRelLocal.y * vRelLocal.y,
            -Mathf.Sign(vRelLocal.z) * quadDragCoefficients.z * vRelLocal.z * vRelLocal.z
        );

        Vector3 fDragWorld = transform.TransformDirection(fDragLocal);
        rb.AddForce(fDragWorld, ForceMode.Force);

        // --- Rotational damping ---
        Vector3 wLocal = transform.InverseTransformDirection(rb.angularVelocity);
        Vector3 tDampLocal = new Vector3(
            -angularDamping.x * wLocal.x,
            -angularDamping.y * wLocal.y,
            -angularDamping.z * wLocal.z
        );
        Vector3 tDampWorld = transform.TransformDirection(tDampLocal);
        rb.AddTorque(tDampWorld, ForceMode.Force);
    }

#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        if (rb == null) return;
        Vector3 p = (centerOfBuoyancy != null) ? centerOfBuoyancy.position : rb.worldCenterOfMass;
        Gizmos.color = Color.blue;
        Gizmos.DrawWireSphere(p, 0.03f);
    }
#endif
}