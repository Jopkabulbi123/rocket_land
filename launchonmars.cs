using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class MLRocketController : Agent
{
    [SerializeField] private float thrustForce = 1000f;
    [SerializeField] private float thrustOffsetRange = 0.3f;
    [SerializeField] private float thrustDistance = 2f;
    [SerializeField] private LayerMask groundLayer;
    [SerializeField] private float maxRotationSpeed = 90f;
    private const float MOON_GRAVITY = -1.62f;
    [SerializeField] private float maxFuel = 100f;
    [SerializeField] private float fuelConsumptionRate = 10f;
    [SerializeField] private float groundCheckDistance = 5f;
    [SerializeField] private float autoAlignThreshold = 0.95f;
    private float currentFuel;
    [SerializeField] private ParticleSystem thrustParticles;
    [SerializeField] private Color debugLineColor = Color.yellow;
    public GameObject thrustPoint;
    private Rigidbody rb;
    private Vector3 initialPosition;
    private Quaternion initialRotation;
    private AreaSelector areaSelector;
    private float maxDistance;
    private bool isLanded = false;
    private Vector3 targetCenter;
    private bool isAreaSelected = false;
    private Vector3 lastThrustDirection;
    private bool isInLandingPhase = false;
    private Vector3 lastPosition;
    private float bestDistance;
    private Vector3 targetThrustPos;
    private bool isThrustActive = false;
    private bool isAutoAligning = false;
    private VisualDebug visualDebug;

    public override void Initialize()
    {
        Time.timeScale = 1f;
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false;
        initialPosition = transform.position;
        initialRotation = transform.rotation;
        areaSelector = FindObjectOfType<AreaSelector>();
        rb.drag = 0.5f;
        rb.angularDrag = 0.5f;
        rb.maxAngularVelocity = maxRotationSpeed;
        rb.constraints = RigidbodyConstraints.FreezeAll;
        lastThrustDirection = Vector3.up;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        visualDebug = GetComponent<VisualDebug>();
    }

    void Update()
    {
        if (areaSelector != null && areaSelector.IsAreaFixed() && !isAreaSelected)
        {
            isAreaSelected = true;
            targetCenter = areaSelector.GetSelectedCenter();
            maxDistance = Vector3.Distance(initialPosition, targetCenter);
            bestDistance = maxDistance;
            rb.constraints = RigidbodyConstraints.None;
        }
        if (visualDebug != null)
    {
        float fuelPercentage = currentFuel / maxFuel;
        float velocityMagnitude = rb.velocity.magnitude;
        float height = transform.position.y - targetCenter.y;
        float alignment = Vector3.Dot(transform.up, Vector3.up);
        
        visualDebug.UpdateHUD(fuelPercentage, velocityMagnitude, height, alignment);
        
        Vector3 thrustPos = transform.TransformPoint(thrustPoint.transform.localPosition);
        Vector3 thrustDir = transform.TransformDirection(thrustPoint.transform.localPosition).normalized;
        
        visualDebug.UpdateVisuals(
            transform.position,
            targetCenter,
            rb.velocity,
            thrustPos,
            thrustDir
        );
    }
    }

    public override void OnEpisodeBegin()
    {
        currentFuel = maxFuel;
        isLanded = false;
        transform.position = initialPosition;
        transform.rotation = initialRotation;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        thrustPoint.transform.localPosition = new Vector3(0, -thrustDistance, 0);
        lastThrustDirection = Vector3.up;
        isInLandingPhase = false;
        lastPosition = initialPosition;
        bestDistance = maxDistance;
        isAreaSelected = false;
        rb.constraints = RigidbodyConstraints.FreezeAll;
        
        if (thrustParticles != null && thrustParticles.isPlaying)
        {
            thrustParticles.Stop();
        }
        isThrustActive = false;

        if (areaSelector != null && areaSelector.IsAreaFixed())
        {
            isAreaSelected = true;
            targetCenter = areaSelector.GetSelectedCenter();
            rb.constraints = RigidbodyConstraints.None;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 toTarget = targetCenter - transform.position;
        float horizontalDistance = new Vector2(toTarget.x, toTarget.z).magnitude;
        
        sensor.AddObservation(toTarget.normalized);
        sensor.AddObservation(transform.position - targetCenter);
        sensor.AddObservation(rb.velocity / 10f);
        sensor.AddObservation(rb.angularVelocity / maxRotationSpeed);
        sensor.AddObservation(transform.up);
        sensor.AddObservation(currentFuel / maxFuel);
        sensor.AddObservation(horizontalDistance / maxDistance);
        sensor.AddObservation(isInLandingPhase);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (!isAreaSelected || isLanded) 
        {
            StopThrustParticles();
            return;
        }

        if (Physics.Raycast(transform.position, Vector3.down, out RaycastHit hit, groundCheckDistance, groundLayer))
        {
            float currentAlignment = Vector3.Dot(transform.up, Vector3.up);
            if (currentAlignment >= autoAlignThreshold && currentAlignment < 1.0f)
            {
                isAutoAligning = true;
                AutoAlign();
                return;
            }
        }

        isAutoAligning = false;

        float thrust = Mathf.Clamp01(actions.ContinuousActions[0]);
        float thrustX = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float thrustZ = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        float thrustMultiplier = 1.0f;

        targetThrustPos = new Vector3(
            thrustX * thrustOffsetRange,
            -thrustDistance,
            thrustZ * thrustOffsetRange
        );

        thrustPoint.transform.localPosition = Vector3.Lerp(
            thrustPoint.transform.localPosition,
            targetThrustPos,
            Time.fixedDeltaTime * 10f
        );

        bool hasRotation = Mathf.Abs(thrustX) > 0.1f || Mathf.Abs(thrustZ) > 0.1f;
        
        if (currentFuel > 0f)
        {
            StartThrustParticles();
            if (thrust > 0 || hasRotation)
            {
                if (hasRotation && thrust <= 0)
                {
                    thrust = 1.0f;
                }

                Vector3 mainThrustDirection = transform.up;
                rb.AddForce(mainThrustDirection * thrustForce * thrust * thrustMultiplier * Time.fixedDeltaTime, 
                    ForceMode.Force);

                if (hasRotation)
                {
                    Vector3 xRotation = transform.right * thrustZ;
                    Vector3 zRotation = -transform.forward * thrustX;
                    Vector3 torque = (xRotation + zRotation) * thrustForce * thrust * 0.1f;
                    rb.AddTorque(torque * thrustMultiplier * Time.fixedDeltaTime);
                }

                currentFuel -= thrust * fuelConsumptionRate * Time.fixedDeltaTime;
                lastThrustDirection = mainThrustDirection;
            }
            else
            {
                currentFuel -= 0.1f * fuelConsumptionRate * Time.fixedDeltaTime;
            }
        }
        else
        {
            StopThrustParticles();
        }

        rb.AddForce(Vector3.up * MOON_GRAVITY * rb.mass, ForceMode.Acceleration);
        CalculateRewards();
    }

    private void AutoAlign()
    {
        Vector3 currentUp = transform.up;
        Vector3 targetUp = Vector3.up;
        
        Quaternion targetRotation = Quaternion.FromToRotation(currentUp, targetUp) * transform.rotation;
        
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.fixedDeltaTime * 5f);
        
        rb.angularVelocity = Vector3.Lerp(rb.angularVelocity, Vector3.zero, Time.fixedDeltaTime * 10f);
        
        rb.AddForce(Vector3.up * thrustForce * 0.1f * Time.fixedDeltaTime, ForceMode.Force);
        
        Vector3 horizontalVelocity = new Vector3(rb.velocity.x, 0f, rb.velocity.z);
        rb.velocity = new Vector3(
            Mathf.Lerp(rb.velocity.x, 0f, Time.fixedDeltaTime * 3f),
            rb.velocity.y,
            Mathf.Lerp(rb.velocity.z, 0f, Time.fixedDeltaTime * 3f)
        );
    }

    private void StartThrustParticles()
    {
        if (thrustParticles != null && !thrustParticles.isPlaying)
        {
            thrustParticles.Play();
            isThrustActive = true;
        }
    }

    private void StopThrustParticles()
    {
        if (thrustParticles != null && thrustParticles.isPlaying)
        {
            thrustParticles.Stop();
            isThrustActive = false;
        }
    }
    private void StabilizeForLanding()
{
    rb.angularVelocity = Vector3.Lerp(rb.angularVelocity, Vector3.zero, Time.fixedDeltaTime * 3f);

    Vector3 targetUp = Vector3.up;
    Vector3 torqueToAlign = Vector3.Cross(transform.up, targetUp);
    rb.AddTorque(torqueToAlign * thrustForce * 0.1f * Time.fixedDeltaTime, ForceMode.Force);

    Vector3 horizontalVelocity = new Vector3(rb.velocity.x, 0f, rb.velocity.z);
    rb.velocity = new Vector3(
        Mathf.Lerp(rb.velocity.x, 0f, Time.fixedDeltaTime * 2f),
        rb.velocity.y,
        Mathf.Lerp(rb.velocity.z, 0f, Time.fixedDeltaTime * 2f)
    );

    if (rb.velocity.y > 0)
    {
        rb.AddForce(Vector3.down * rb.mass * 2f, ForceMode.Force);
    }
}


    private bool IsWithinLandingArea(Vector3 position)
    {
        if (areaSelector.fixedLineRenderer == null) return false;

        Vector3[] corners = new Vector3[5];
        areaSelector.fixedLineRenderer.GetPositions(corners);

        Vector2[] polygonPoints = new Vector2[4];
        for (int i = 0; i < 4; i++)
        {
            polygonPoints[i] = new Vector2(corners[i].x, corners[i].z);
        }

        return IsPointInPolygon(new Vector2(position.x, position.z), polygonPoints);
    }

    private bool IsPointInPolygon(Vector2 point, Vector2[] polygon)
    {
        bool inside = false;
        for (int i = 0, j = polygon.Length - 1; i < polygon.Length; j = i++)
        {
            if (((polygon[i].y > point.y) != (polygon[j].y > point.y)) &&
                (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / 
                (polygon[j].y - polygon[i].y) + polygon[i].x))
            {
                inside = !inside;
            }
        }
        return inside;
    }
   private void CalculateRewards()
{
    Vector3 toTarget = targetCenter - transform.position;
    float distanceToTarget = toTarget.magnitude;
    float normalizedDistance = distanceToTarget / maxDistance;

    float progressReward = (maxDistance - distanceToTarget) / maxDistance;
    AddReward(progressReward * 0.3f);

    float velocityTowardTarget = Vector3.Dot(rb.velocity.normalized, toTarget.normalized);
    AddReward(velocityTowardTarget * 0.2f);

    float alignmentThreshold = 0.3f;
    if (normalizedDistance < alignmentThreshold) {
        float alignmentImportance = Mathf.InverseLerp(alignmentThreshold, 0f, normalizedDistance);
        float currentAlignment = Vector3.Dot(transform.up, Vector3.up);
        AddReward(currentAlignment * alignmentImportance * 0.2f);
    }

    if (Physics.Raycast(transform.position, Vector3.down, out RaycastHit hit, 1f, groundLayer))
    {
        if (IsWithinLandingArea(transform.position))
        {
            float landingAlignment = Vector3.Dot(transform.up, Vector3.up);
            
            if (landingAlignment > 0.95f)
            {
                AddReward(5f);
            }
            else if (landingAlignment > 0.8f)
            {
                AddReward(2f);
            }
            else
            {
                AddReward(-2f);
            }
        }
        else
        {
            AddReward(-3f);
        }
        EndEpisode();
    }

    if (currentFuel <= 0)
    {
        AddReward(-2f);
        EndEpisode();
    }

    AddReward(-0.001f);
}
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
    
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");
    
        continuousActions[1] = horizontal;
        continuousActions[2] = vertical;

        bool shouldThrust = Input.GetKey(KeyCode.Space) || 
                          Mathf.Abs(horizontal) > 0.1f || 
                          Mathf.Abs(vertical) > 0.1f;
        
        continuousActions[0] = shouldThrust ? 1.0f : 0f;
    }
}