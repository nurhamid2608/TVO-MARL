using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

[DisallowMultipleComponent]
public class EncirclementEvaluator : MonoBehaviour
{
    [Header("Scene Refs")]
    public EncirclementManager encirclementManager;   // required
    public USVAgent primaryAgent;                     // uses StepCount and thresholds from this agent (optional but recommended)
    public Transform attacker;                        // reference for angular and rotation metrics

    [Header("Episode Settings")]
    [Tooltip("Number of steps per episode used for evaluation.")]
    public int episodeMaxSteps = 8000;

    [Tooltip("If true, episode results are latched when the episode ends.")]
    public bool latchOnEpisodeEnd = true;

    [Header("Thresholds (fallback if not taken from USVAgent)")]
    public float minAngularCoverageDeg = 180f;
    [Range(0f, 1f)] public float inRadiusRequiredFraction = 0.60f;
    public int successHoldSteps = 8;

    [Header("Live Metrics (read-only)")]
    [SerializeField] private int currentStep;
    [SerializeField] private float angularCoverage;
    [SerializeField] private float maxAngularGap;
    [SerializeField] private bool isHighlyClustered;
    [SerializeField] private float fracInRadius;
    [SerializeField] private int usvCount;

    [Header("Episode Results (latched)")]
    [SerializeField] private bool episodeCompleted;
    [SerializeField] private int convergenceStep = -1;          // -1 = not yet converged
    [SerializeField] private int convergenceHoldCounter;
    [SerializeField] private float signedRotationsAvg;          // (+CCW, -CW)
    [SerializeField] private float absRotationsAvg;             // unsigned
    [SerializeField] private float signedRotationsSum;          // total across all USVs
    [SerializeField] private float absRotationsSum;

    // --- internal state for rotation counting ---
    private readonly List<Transform> trackedUSVs = new();
    private Dictionary<Transform, float> lastAngle = new Dictionary<Transform, float>();
    private Dictionary<Transform, float> cumSignedAngle = new Dictionary<Transform, float>();

    private bool latched; // prevents overwriting after completion

    void Awake()
    {
        if (encirclementManager == null)
            Debug.LogError("[EncirclementEvaluator] EncirclementManager has not been assigned.");
    }

    void OnEnable()
    {
        if (Academy.IsInitialized)
            Academy.Instance.OnEnvironmentReset += HandleEnvReset;
    }

    void OnDisable()
    {
        if (Academy.IsInitialized)
            Academy.Instance.OnEnvironmentReset -= HandleEnvReset;
    }

    void Start()
    {
        // Pull thresholds from USVAgent when available
        PullThresholdsFromAgent();
        BuildUSVList();
        InitAngleTrackers();

        // If primaryAgent is unavailable, continue using a manual step counter
        if (primaryAgent == null)
            Debug.LogWarning("[EncirclementEvaluator] primaryAgent is not assigned. currentStep will be counted manually instead of using Agent.StepCount.");
    }

    void FixedUpdate()
    {
        if (encirclementManager == null || attacker == null) return;

        // Update live metrics from the manager
        angularCoverage = encirclementManager.AngularCoverage;
        maxAngularGap = encirclementManager.MaxGap;
        isHighlyClustered = encirclementManager.IsHighlyClustered;
        fracInRadius = encirclementManager.FracInRadius;
        usvCount = encirclementManager.USVCount;

        // Prefer Agent.StepCount to stay synchronized with ML-Agents decisions
        currentStep = (primaryAgent != null) ? primaryAgent.StepCount : currentStep + 1;

        // Compute rotations from the cumulative signed angle
        UpdateRotations();

        // Detect convergence when coverage and in-radius conditions persist for several steps
        UpdateConvergence();

        // Finalize the episode at the target step
        if (!episodeCompleted && currentStep >= episodeMaxSteps)
        {
            FinalizeEpisodeResults();
        }
    }

    // ==== Public getters (optional, for access from other components) ====
    public int GetConvergenceStep() => convergenceStep;
    public float GetAngularCoverage() => angularCoverage;
    public float GetMaxGap() => maxAngularGap;
    public bool GetIsHighlyClustered() => isHighlyClustered;
    public float GetSignedRotAvg() => signedRotationsAvg;
    public float GetAbsRotAvg() => absRotationsAvg;

    [ContextMenu("Reset Evaluation (manual)")]
    public void ResetEvaluation()
    {
        latched = false;
        episodeCompleted = false;
        convergenceStep = -1;
        convergenceHoldCounter = 0;
        signedRotationsAvg = absRotationsAvg = 0f;
        signedRotationsSum = absRotationsSum = 0f;
        BuildUSVList();
        InitAngleTrackers();
    }

    // ==== Internal helpers ====

    private void HandleEnvReset()
    {
        // Reset evaluation when the environment starts a new episode
        ResetEvaluation();
    }

    private void PullThresholdsFromAgent()
    {
        if (primaryAgent == null) return;
        // Pull public values from USVAgent to keep evaluation consistent with the success reward
        minAngularCoverageDeg = primaryAgent.minAngularCoverageDeg;
        inRadiusRequiredFraction = primaryAgent.inRadiusRequiredFraction;
        successHoldSteps = primaryAgent.successHoldSteps;
    }

    private void BuildUSVList()
    {
        trackedUSVs.Clear();
        if (encirclementManager.usvs != null)
            trackedUSVs.AddRange(encirclementManager.usvs);
        // Remove null references
        trackedUSVs.RemoveAll(t => t == null);
    }

    private void InitAngleTrackers()
    {
        lastAngle.Clear();
        cumSignedAngle.Clear();

        foreach (var d in trackedUSVs)
        {
            float a = GetAngleDeg(d.position);
            lastAngle[d] = a;
            cumSignedAngle[d] = 0f;
        }
    }

    private float GetAngleDeg(Vector3 worldPos)
    {
        Vector3 v = worldPos - attacker.position;
        float ang = Mathf.Atan2(v.z, v.x) * Mathf.Rad2Deg;
        if (ang < 0f) ang += 360f;
        return ang;
    }

    private void UpdateRotations()
    {
        // Keep the USV list up to date when auto-discovery is enabled in the manager
        if (Time.frameCount % 30 == 0)
        {
            BuildUSVList();
            // Add trackers for newly discovered USVs
            foreach (var d in trackedUSVs)
            {
                if (!lastAngle.ContainsKey(d))
                {
                    lastAngle[d] = GetAngleDeg(d.position);
                    cumSignedAngle[d] = 0f;
                }
            }
        }

        float sumSigned = 0f, sumAbs = 0f; int n = 0;

        foreach (var d in trackedUSVs)
        {
            if (d == null) continue;

            float prev = lastAngle[d];
            float now = GetAngleDeg(d.position);

            // DeltaAngle returns a signed angular difference in the range [-180, +180]
            float dAng = Mathf.DeltaAngle(prev, now);
            lastAngle[d] = now;

            // Accumulate the signed angle for this USV
            cumSignedAngle[d] += dAng;

            // Convert accumulated angle to rotations (degrees / 360)
            float rotSigned = cumSignedAngle[d] / 360f;
            float rotAbs = Mathf.Abs(rotSigned);

            sumSigned += rotSigned;
            sumAbs += rotAbs;
            n++;
        }

        if (n > 0)
        {
            signedRotationsSum = sumSigned;
            absRotationsSum = sumAbs;
            signedRotationsAvg = sumSigned / n;   // (+) CCW, (−) CW
            absRotationsAvg = sumAbs / n;
        }
        else
        {
            signedRotationsSum = absRotationsSum = 0f;
            signedRotationsAvg = absRotationsAvg = 0f;
        }
    }

    private void UpdateConvergence()
    {
        bool meetsCoverage = angularCoverage >= minAngularCoverageDeg;
        bool meetsInRadius = fracInRadius >= inRadiusRequiredFraction;

        if (meetsCoverage && meetsInRadius)
        {
            convergenceHoldCounter++;
            if (convergenceStep < 0 && convergenceHoldCounter >= successHoldSteps)
            {
                // Record the first step at which the success condition becomes stable
                convergenceStep = Mathf.Max(0, currentStep - successHoldSteps + 1);
            }
        }
        else
        {
            convergenceHoldCounter = 0;
        }
    }

    private void FinalizeEpisodeResults()
    {
        episodeCompleted = true;

        if (latchOnEpisodeEnd)
            latched = true;

        // Optional concise summary in the Console
        Debug.Log($"[EncirclementEvaluator] EPISODE DONE @ step={currentStep} | " +
                  $"Coverage={angularCoverage:F1}°, MaxGap={maxAngularGap:F1}°, " +
                  $"FracInRadius={fracInRadius:F2}, ConvergedAt={convergenceStep}, " +
                  $"RotAvgSigned={signedRotationsAvg:F2}, RotAvgAbs={absRotationsAvg:F2}");
    }
}
