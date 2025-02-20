using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class VisualDebug : MonoBehaviour
{
    [Header("Line Renderers")]
    [SerializeField] private LineRenderer targetLine;
    [SerializeField] private LineRenderer velocityLine;
    [SerializeField] private LineRenderer thrustLine;
    
    [Header("UI Elements")]
    [SerializeField] private TextMeshProUGUI fuelText;
    [SerializeField] private TextMeshProUGUI speedText;
    [SerializeField] private TextMeshProUGUI heightText;
    [SerializeField] private TextMeshProUGUI alignText;
    
    [Header("Visual Settings")]
    [SerializeField] private Color targetLineColor = Color.yellow;
    [SerializeField] private Color velocityLineColor = Color.red;
    [SerializeField] private Color thrustLineColor = Color.blue;
    [SerializeField] private float lineWidth = 0.1f;

    private void Start()
    {
        InitializeLineRenderer(targetLine, targetLineColor);
        InitializeLineRenderer(velocityLine, velocityLineColor);
        InitializeLineRenderer(thrustLine, thrustLineColor);
    }

    private void InitializeLineRenderer(LineRenderer lr, Color color)
    {
        if (lr != null)
        {
            lr.material = new Material(Shader.Find("Sprites/Default"));
            lr.startColor = color;
            lr.endColor = color;
            lr.startWidth = lineWidth;
            lr.endWidth = lineWidth;
        }
    }

    public void UpdateVisuals(Vector3 position, Vector3 targetCenter, Vector3 velocity, 
        Vector3 thrustPoint, Vector3 thrustDirection)
    {
        if (targetLine != null)
        {
            targetLine.SetPositions(new Vector3[] { position, targetCenter });
        }

        if (velocityLine != null)
        {
            velocityLine.SetPositions(new Vector3[] { position, position + velocity });
        }

        if (thrustLine != null)
        {
            Vector3 start = thrustPoint;
            Vector3 end = start - thrustDirection * 2f;
            thrustLine.SetPositions(new Vector3[] { start, end });
        }
    }

    public void UpdateHUD(float fuelPercentage, float speed, float height, float alignment)
    {
        if (fuelText != null)
            fuelText.text = $"Fuel: {(fuelPercentage * 100):F1}%";
        
        if (speedText != null)
            speedText.text = $"Speed: {speed:F1} m/s";
        
        if (heightText != null)
            heightText.text = $"Height: {height:F1} m";
        
        if (alignText != null)
            alignText.text = $"Align: {(alignment * 100):F1}%";
    }
}