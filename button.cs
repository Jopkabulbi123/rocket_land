using UnityEngine;
using System.Collections;
using System.IO;

public class RocketDataTracker : MonoBehaviour
{
    private MLRocketController rocketController;
    private Rigidbody rocketRb;
    private Vector3 initialPosition;
    private Quaternion initialRotation;
    private float recordInterval = 5f;
    private string dataPath;
    private AreaSelector areaSelector;
    private CameraFollowYAxis cameraFollow;

    void Start()
    {
        rocketController = GetComponent<MLRocketController>();
        rocketRb = GetComponent<Rigidbody>();
        areaSelector = FindObjectOfType<AreaSelector>();
        cameraFollow = FindObjectOfType<CameraFollowYAxis>();
        initialPosition = transform.position;
        initialRotation = transform.rotation;
        
        dataPath = Path.Combine(Application.dataPath, "RocketData.txt");
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            SaveRocketData();
            ExitScene();
        }
    }
     void ExitScene()
    {
        #if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
        #else
            Application.Quit();
        #endif
    }

    void OnEnable()
    {
        StartCoroutine(RecordDataRoutine());
    }

    IEnumerator RecordDataRoutine()
    {
        while (true)
        {
            yield return new WaitForSeconds(recordInterval);
            SaveRocketData();
        }
    }

    void SaveRocketData()
    {
        string data = $"Time: {Time.time}\n" +
                     $"Position: {transform.position}\n" +
                     $"Rotation: {transform.rotation.eulerAngles}\n" +
                     $"Velocity: {rocketRb.velocity}\n" +
                     $"Height: {(transform.position.y * transform.position.y)/10000000} km\n" +
                     $"------------------------\n";

        File.AppendAllText(dataPath, data);
    }

    void ResetScene()
    {
        transform.position = initialPosition;
        transform.rotation = initialRotation;
        rocketRb.velocity = Vector3.zero;
        rocketRb.angularVelocity = Vector3.zero;
        
        if (rocketController != null)
        {
            rocketController.OnEpisodeBegin();
        }

        if (areaSelector != null)
        {
            areaSelector.ResetAll();
        }

        if (cameraFollow != null)
        {
            cameraFollow.ResetCamera();
        }
    }
}