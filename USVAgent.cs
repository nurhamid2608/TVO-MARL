using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class USVAgent : Agent
{
    [Header("Scene Refs")]
    public Transform baseZone;
    public Transform attacker;

    [Header("Motion")]
    public float moveSpeed = 6f;
    public float turnSpeed = 200f;
    public int maxStepOverride = 3000;

    [Header("Encirclement")]
    public float targetRadius = 4f;

    // ===== Reward scales (increased and rebalanced) =====
    [Header("Reward Scales")]
    [Tooltip("General per-step shaping reward scale.")]
    public float kShape = 0.0025f;          // increased from 0.0012
    [Tooltip("Per-step orbit/rotation-direction reward scale.")]
    public float kSpin = 0.0020f;
    [Tooltip("Per-step group coverage reward scale.")]
    public float kCov = 0.08f;              // increased from 0.06
    [Tooltip("Bonus when coverage improves between steps.")]
    public float kDeltaCov = 0.02f;

    // --- smoothing & speed control ---
    [Header("Smoothing & Speed")]
    public float targetTangentialSpeed = 3.5f; // m/s approx
    [Range(0f, 1f)] public float turnSmooth = 0.15f;
    [Range(0f, 1f)] public float thrustSmooth = 0.15f;
    public float radialVelPenalty = 0.01f;
    private float turnSm, thrustSm;

    // --- separation (anti-crowding) ---
    [Header("Separation")]
    public float sepRadius = 2.2f;
    public float comfyRadius = 3.5f;
    public float sepPenalty = 0.03f;
    public float comfyBonus = 0.005f;

    // --- anti-reward-hacking / coverage gating ---
    [Header("Anti-Reward-Hacking")]
    public float radiusTolerance = 0.8f;             // ring tolerance
    public float minAngularCoverageDeg = 180f;       // reduced from 270 to make the condition easier
    public float inRadiusRequiredFraction = 0.60f;   // reduced from 0.75
    public int successHoldSteps = 8;                 // require brief persistence

    private Rigidbody rb;
    private int trueStepCount = 0;
    private float lastDistToAttacker;
    private float lastCoverage = 0f;
    private int coverageHoldCounter = 0;

    // cached USVs
    private GameObject[] cachedUSVs;
    private int refreshCounter = 0;
    private const int refreshEveryN = 20;

    // episode information
    public int CurrentEpisode { get; private set; } = 0;
    public string LastEpisodeEndReason { get; private set; } = "N/A";

    // stuck detection
    private Vector3 lastPos;
    private int stillCounter = 0;

    // default per-episode rotation direction (consistent CCW for stability)
    private int episodeSpinDir = +1; // -1 (CW), +1 (CCW)

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        MaxStep = maxStepOverride;
        cachedUSVs = GameObject.FindGameObjectsWithTag("Defender");
    }

    public override void OnEpisodeBegin()
    {
        CurrentEpisode++;
        trueStepCount = 0;
        coverageHoldCounter = 0;
        LastEpisodeEndReason = "New";

        if (attacker != null)
            lastDistToAttacker = Vector3.Distance(transform.position, attacker.position);

        cachedUSVs = GameObject.FindGameObjectsWithTag("Defender");
        lastPos = transform.position;
        stillCounter = 0;
        turnSm = thrustSm = 0f;

        // set the initial orbit direction to CCW for consistency
        episodeSpinDir = +1;

        // reset the coverage tracker
        var m = EncirclementManager.Instance;
        lastCoverage = (m != null) ? m.AngularCoverage : 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // total: 16 floats
        sensor.AddObservation(transform.localPosition);                 // 3
        sensor.AddObservation(transform.forward);                       // 3

        if (attacker != null)
        {
            Vector3 relPos = attacker.localPosition - transform.localPosition;
            sensor.AddObservation(relPos.normalized);                   // 3
            sensor.AddObservation(attacker.forward);                    // 3
            sensor.AddObservation(Vector3.Distance(transform.position, attacker.position)); // 1
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(0f);
        }

        sensor.AddObservation(baseZone.localPosition);                  // 3
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        trueStepCount++;
        if (trueStepCount >= maxStepOverride)
        {
            LastEpisodeEndReason = "MaxStep Timeout";
            EndEpisode();
            return;
        }

        // --- smooth discrete control ---
        int a = actions.DiscreteActions[0];
        float thrustCmd = (a == 1) ? 1f : 0f;
        float turnCmd = (a == 2) ? 1f : (a == 3) ? -1f : 0f;

        turnSm = Mathf.Lerp(turnSm, turnCmd, turnSmooth);
        thrustSm = Mathf.Lerp(thrustSm, thrustCmd, thrustSmooth);

        transform.Rotate(Vector3.up, turnSm * turnSpeed * Time.deltaTime);
        rb.MovePosition(transform.position + transform.forward * (thrustSm * moveSpeed) * Time.deltaTime);

        // --- base time penalty (kept small to avoid dominating the reward) ---
        AddReward(-0.00005f);

        // --- radius stabilization reward (always active, small but consistent) ---
        if (attacker != null)
        {
            float rNow = Vector3.Distance(transform.position, attacker.position);
            float rErr = Mathf.Abs(rNow - targetRadius);
            float rStab = 1f - Mathf.Clamp01(rErr / Mathf.Max(0.1f, targetRadius));
            AddReward(0.5f * kShape * rStab);
        }

        // --- gated approach reward (small) ---
        float distToAttacker = Vector3.Distance(transform.position, attacker.position);
        float deltaDist = lastDistToAttacker - distToAttacker;
        if (lastDistToAttacker > targetRadius + radiusTolerance &&
            distToAttacker >= targetRadius - radiusTolerance)
        {
            AddReward(0.5f * kShape * Mathf.Clamp(deltaDist, -0.5f, 0.5f));
        }
        lastDistToAttacker = distToAttacker;

        // --- speed shaping: tangential speed near target, small radial velocity ---
        if (attacker != null)
        {
            Vector3 toAtk = transform.position - attacker.position;
            Vector3 tan = Vector3.Cross(Vector3.up, toAtk).normalized;
            float vTan = Vector3.Dot(rb.linearVelocity, tan);                  // signed
            float vRad = Vector3.Dot(rb.linearVelocity, toAtk.normalized);

            float tangentialMatch = 1f - Mathf.Clamp01(Mathf.Abs(Mathf.Abs(vTan) - targetTangentialSpeed) / Mathf.Max(0.1f, targetTangentialSpeed));
            AddReward(kShape * tangentialMatch);
            AddReward(-radialVelPenalty * Mathf.Clamp01(Mathf.Abs(vRad) / (targetTangentialSpeed + 1e-3f)));
        }

        // --- local shaping (consistent rotation direction) ---
        int desiredSpin = GetDesiredSpin(); // -1,0,+1
        ComputeEncirclementLocalReward(distToAttacker, desiredSpin);
        ComputeCollaborativeReward(distToAttacker);
        ComputeAngleFormationReward(distToAttacker);

        // --- wall and stuck penalties (softened) ---
        ApplyWallPenalty();
        PenalizeStuck();

        // --- group rewards via manager and success condition ---
        ComputeAngularCoverageGroupReward();     // use manager as the primary source
        SeparationReward();                      // anti-crowding

        // --- failure: base breached ---
        if (Vector3.Distance(attacker.position, baseZone.position) < 1.5f)
        {
            AddReward(-1.0f);
            LastEpisodeEndReason = "Base Breached";
            EndEpisode();
        }

        if (++refreshCounter % refreshEveryN == 0)
            cachedUSVs = GameObject.FindGameObjectsWithTag("Defender");
    }

    private int GetDesiredSpin()
    {
        var m = EncirclementManager.Instance;
        if (m != null && m.ConsensusSpin != 0) return m.ConsensusSpin;
        return episodeSpinDir; // per-episode fallback (CCW)
    }

    private void ComputeEncirclementLocalReward(float distToAttacker, int desiredSpin)
    {
        if (attacker == null || distToAttacker > 10f) return;

        Vector3 toAttacker = transform.position - attacker.position;
        float radius = toAttacker.magnitude;
        float radiusError = Mathf.Abs(radius - targetRadius);
        float rewardRadius = Mathf.Clamp01(1f - radiusError / Mathf.Max(0.1f, targetRadius));

        // --- rotation direction: signed value (not absolute) ---
        Vector3 tangentDir = Vector3.Cross(Vector3.up, toAttacker).normalized;
        Vector3 v = rb.linearVelocity.sqrMagnitude > 1e-6f ? rb.linearVelocity.normalized : Vector3.zero;
        float signedTang = Vector3.Dot(v, tangentDir);  // -1..+1

        float orbitAlign = (desiredSpin == 0) ? Mathf.Abs(signedTang)
                                              : Mathf.Max(0f, signedTang * desiredSpin);

        AddReward(kShape * rewardRadius + kSpin * orbitAlign);
    }

    private void ComputeCollaborativeReward(float distToAttacker)
    {
        if (distToAttacker > targetRadius * 1.5f) return;

        var allUSVs = cachedUSVs;
        if (allUSVs == null || allUSVs.Length == 0) return;

        float totalDistance = 0f;
        int count = 0;

        foreach (GameObject other in allUSVs)
        {
            if (other == null || other == this.gameObject) continue;
            float d = Vector3.Distance(transform.position, other.transform.position);
            totalDistance += d;
            count++;
        }

        if (count > 0)
        {
            float avgDist = totalDistance / count;
            float targetSpacing = 6f;
            float spacingError = Mathf.Abs(avgDist - targetSpacing);
            float rewardSpacing = Mathf.Clamp01(1f - spacingError / Mathf.Max(0.1f, targetSpacing));
            AddReward(kShape * rewardSpacing);
        }
    }

    // Revised: evaluate the nearest angular neighbors in both CW and CCW directions
    private void ComputeAngleFormationReward(float distToAttacker)
    {
        if (attacker == null || distToAttacker > targetRadius * 1.5f) return;
        var ds = cachedUSVs;
        if (ds == null || ds.Length < 3) return;

        Vector3 myVec = transform.position - attacker.position;
        float myAng = Mathf.Atan2(myVec.z, myVec.x) * Mathf.Rad2Deg;
        if (myAng < 0f) myAng += 360f;

        float bestCWGap = 360f, bestCCWGap = 360f;
        foreach (var other in ds)
        {
            if (other == null || other == gameObject) continue;
            Vector3 ov = other.transform.position - attacker.position;
            float oa = Mathf.Atan2(ov.z, ov.x) * Mathf.Rad2Deg;
            if (oa < 0f) oa += 360f;

            float diff = oa - myAng;
            if (diff < 0f) diff += 360f; // [0,360)
            if (diff > 0f && diff < bestCWGap) bestCWGap = diff;

            float diffCCW = 360f - diff;
            if (diffCCW > 0f && diffCCW < bestCCWGap) bestCCWGap = diffCCW;
        }
        if (bestCWGap >= 360f || bestCCWGap >= 360f) return;

        float errCW = Mathf.Abs(bestCWGap - 120f);
        float errCCW = Mathf.Abs(bestCCWGap - 120f);
        float score = 1f - Mathf.Clamp01((errCW + errCCW) / 120f); // 0..1

        AddReward(0.5f * kShape * score);
    }

    private void ApplyWallPenalty()
    {
        float arenaHalfSize = 10f;
        float wallMargin = 0.7f;

        float distToLeft = Mathf.Abs(transform.position.x + arenaHalfSize);
        float distToRight = Mathf.Abs(transform.position.x - arenaHalfSize);
        float distToBottom = Mathf.Abs(transform.position.z + arenaHalfSize);
        float distToTop = Mathf.Abs(transform.position.z - arenaHalfSize);

        float minWallDist = Mathf.Min(distToLeft, distToRight, distToTop, distToBottom);
        if (minWallDist < wallMargin) AddReward(-0.005f); // reduced from -0.01
    }

    private void PenalizeStuck()
    {
        float moved = (transform.position - lastPos).magnitude;
        lastPos = transform.position;
        if (moved < 0.02f) stillCounter++; else stillCounter = 0;
        if (stillCounter > 40) AddReward(-0.005f); // reduced from -0.02
    }

    // ---- Primary group angular coverage reward + delta-coverage bonus ----
    private void ComputeAngularCoverageGroupReward()
    {
        var m = EncirclementManager.Instance;
        if (m == null) return;

        float coverage = m.AngularCoverage;    // 0..360
        float fracInRadius = m.FracInRadius;   // 0..1

        // Gate the reward so it is active only when most USVs are within the target ring
        float gate = Mathf.Clamp01((fracInRadius - inRadiusRequiredFraction) / (1f - inRadiusRequiredFraction + 1e-6f));
        float reward = kCov * (coverage / 360f) * gate;
        AddReward(reward);

        // Delta coverage reward is given only when coverage increases
        float dCov = Mathf.Max(0f, coverage - lastCoverage);
        AddReward(kDeltaCov * (dCov / 10f));   // smooth normalization
        lastCoverage = coverage;

        if (m.IsHighlyClustered) AddReward(-0.005f);

        if (coverage >= minAngularCoverageDeg && fracInRadius >= inRadiusRequiredFraction)
        {
            coverageHoldCounter++;
            if (coverageHoldCounter >= successHoldSteps)
            {
                AddReward(1.0f);
                LastEpisodeEndReason = "Encirclement Achieved";
                EndEpisode();
            }
        }
        else
        {
            coverageHoldCounter = 0;
        }
    }

    // ---- Separation reward to avoid pushing each other ----
    private void SeparationReward()
    {
        var ds = cachedUSVs;
        if (ds == null) return;
        foreach (var other in ds)
        {
            if (other == null || other == gameObject) continue;
            float d = Vector3.Distance(transform.position, other.transform.position);
            if (d < sepRadius)
                AddReward(-sepPenalty * (sepRadius - d) / Mathf.Max(0.001f, sepRadius));
            else if (d < comfyRadius)
                AddReward(comfyBonus * (d - sepRadius) / Mathf.Max(0.001f, (comfyRadius - sepRadius)));
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.transform.CompareTag("Wall"))
        {
            AddReward(-0.3f);
            LastEpisodeEndReason = "Hit Wall";
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actions = actionsOut.DiscreteActions;
        actions[0] = 0;
        if (Input.GetKey(KeyCode.W)) actions[0] = 1; // thrust
        if (Input.GetKey(KeyCode.D)) actions[0] = 2; // yaw right
        if (Input.GetKey(KeyCode.A)) actions[0] = 3; // yaw left
    }
}
