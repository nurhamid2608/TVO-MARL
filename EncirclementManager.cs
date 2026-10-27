using System.Collections.Generic;
using UnityEngine;

public class EncirclementManager : MonoBehaviour
{
    [Header("Refs")]
    public Transform attacker;

    [Header("Encirclement Settings")]
    public float targetRadius = 4f;
    public float radiusTolerance = 0.8f;

    [Header("Discovery")]
    public bool autoDiscoverUSVs = true;
    public string usvTag = "Defender";
    public List<Transform> usvs = new List<Transform>();

    // Output metrics
    public float AngularCoverage { get; private set; } = 0f; // range: 0..360
    public float MaxGap { get; private set; } = 360f;        // range: 0..360
    public float FracInRadius { get; private set; } = 0f;    // range: 0..1
    public int InRadiusCount { get; private set; } = 0;
    public int USVCount { get; private set; } = 0;
    public bool IsHighlyClustered { get; private set; } = false;

    [Header("Cluster Heuristic")]
    public float clusterGapThresholdDeg = 220f;

    [Header("Spin Consensus")]
    public int ConsensusSpin { get; private set; } = 0; // -1 (CW), 0 (none), +1 (CCW)
    public float spinAgreeThreshold = 0.5f;  // easier threshold
    public int spinHoldFrames = 30;          // shorter hysteresis
    private int spinHoldCounter = 0;

    public static EncirclementManager Instance { get; private set; }

    void Awake()
    {
        if (Instance != null && Instance != this)
            Debug.LogWarning("Multiple EncirclementManager instances detected. Using the latest.");
        Instance = this;
    }

    void Start() => RefreshUSVs();

    public void RefreshUSVs()
    {
        usvs.RemoveAll(t => t == null);
        if (autoDiscoverUSVs)
        {
            var found = GameObject.FindGameObjectsWithTag(usvTag);
            usvs.Clear();
            foreach (var go in found) usvs.Add(go.transform);
        }
    }

    void FixedUpdate()
    {
        if (attacker == null) return;
        if (autoDiscoverUSVs && (Time.frameCount % 30 == 0))
            RefreshUSVs();
        ComputeCoverageAndConsensus();
    }

    private void ComputeCoverageAndConsensus()
    {
        List<float> angles = new List<float>(usvs.Count);
        int inRadius = 0;

        float sumSign = 0f, sumAbs = 0f; int nSpin = 0;

        foreach (var d in usvs)
        {
            if (d == null) continue;
            Vector3 toD = d.position - attacker.position;
            float r = toD.magnitude;
            if (r <= targetRadius + radiusTolerance &&
                r >= Mathf.Max(0.1f, targetRadius - radiusTolerance))
                inRadius++;

            float ang = Mathf.Atan2(toD.z, toD.x) * Mathf.Rad2Deg;
            if (ang < 0f) ang += 360f;
            angles.Add(ang);

            var rb = d.GetComponent<Rigidbody>();
            if (rb != null)
            {
                Vector3 tan = Vector3.Cross(Vector3.up, toD).normalized;
                float t = Vector3.Dot(rb.linearVelocity, tan);  // −..+
                if (!float.IsNaN(t))
                {
                    sumSign += Mathf.Sign(t);
                    sumAbs += Mathf.Min(1f, Mathf.Abs(t));
                    nSpin++;
                }
            }
        }

        USVCount = angles.Count;
        InRadiusCount = inRadius;
        FracInRadius = USVCount > 0 ? (float)InRadiusCount / USVCount : 0f;

        if (angles.Count < 2)
        {
            MaxGap = 360f;
            AngularCoverage = 0f;
            IsHighlyClustered = true;
            UpdateConsensus(0, 0f);
            return;
        }

        angles.Sort();
        float maxGap = 0f;
        for (int i = 0; i < angles.Count - 1; i++)
            maxGap = Mathf.Max(maxGap, angles[i + 1] - angles[i]);
        maxGap = Mathf.Max(maxGap, (angles[0] + 360f) - angles[angles.Count - 1]);

        MaxGap = maxGap;
        AngularCoverage = 360f - maxGap;
        IsHighlyClustered = MaxGap > clusterGapThresholdDeg;

        // spin consensus
        float meanAbs = (nSpin > 0) ? (sumAbs / nSpin) : 0f;
        int proposed = 0;
        if (nSpin > 0 && meanAbs >= spinAgreeThreshold && Mathf.Abs(sumSign) >= 1f)
            proposed = (sumSign > 0f) ? +1 : -1;

        UpdateConsensus(proposed, meanAbs);
    }

    private void UpdateConsensus(int proposed, float meanAbs)
    {
        if (proposed != 0)
        {
            if (ConsensusSpin == proposed) spinHoldCounter = spinHoldFrames;
            else if (spinHoldCounter <= 0) { ConsensusSpin = proposed; spinHoldCounter = spinHoldFrames; }
            else spinHoldCounter--;
        }
        else
        {
            if (spinHoldCounter > 0) spinHoldCounter--;
            else ConsensusSpin = 0;
        }
    }
}
