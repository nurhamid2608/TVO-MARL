using UnityEngine;
using Unity.MLAgents;

public class EpisodeManager : MonoBehaviour
{
    public Transform attacker;
    public USVAgent[] usvAgents;

    private Vector3[] usvStartPos;
    private Quaternion[] usvStartRot;

    private bool resetTriggered = false;

    void Start()
    {
        // Store the initial position and rotation of each USV
        usvStartPos = new Vector3[usvAgents.Length];
        usvStartRot = new Quaternion[usvAgents.Length];

        for (int i = 0; i < usvAgents.Length; i++)
        {
            usvStartPos[i] = usvAgents[i].transform.position;
            usvStartRot[i] = usvAgents[i].transform.rotation;
        }

        Academy.Instance.OnEnvironmentReset += ResetEpisode;
    }

    void Update()
    {
        // Check whether all agents have finished (StepCount = 0)
        bool allDone = true;

        foreach (var def in usvAgents)
        {
            if (def.StepCount > 0)
            {
                allDone = false;
                break;
            }
        }

        if (allDone && !resetTriggered)
        {
            ResetEpisode();
            resetTriggered = true;
        }

        if (!allDone)
        {
            resetTriggered = false;
        }
    }

    void ResetEpisode()
    {
        Debug.Log("<color=cyan>[EpisodeManager]</color> ResetEpisode() called.");

        // Spawn the attacker at a random position
        float randX = Random.Range(0f, 10f);
        float randZ = Random.Range(-10f, 10f);
        Vector3 spawnPos = new Vector3(randX, 0f, randZ);

        attacker.position = spawnPos;
        attacker.rotation = Quaternion.identity;

        var rbAtk = attacker.GetComponent<Rigidbody>();
        if (rbAtk != null)
        {
            rbAtk.linearVelocity = Vector3.zero;
            rbAtk.angularVelocity = Vector3.zero;
        }

        // Reset all USVs to their initial positions
        for (int i = 0; i < usvAgents.Length; i++)
        {
            usvAgents[i].transform.position = usvStartPos[i];
            usvAgents[i].transform.rotation = usvStartRot[i];

            var rb = usvAgents[i].GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
        }

        Debug.Log("<color=green>[EpisodeManager]</color> USVs and attacker reset completed.");
    }
}
