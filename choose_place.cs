using UnityEngine;

public class AreaSelector : MonoBehaviour
{
    [Header("Area Selection")]
    public Material lineMaterial;
    public float squareSize = 200f;
    private LineRenderer previewLineRenderer;
    public LineRenderer fixedLineRenderer;
    public Vector3 selectionPoint { get; private set; }
    private bool isFixed = false;

    [Header("References")]
    public MonoBehaviour rocketAgent;
    public Camera mainCamera;
    
    private Vector3[] savedCorners = new Vector3[5]; 
    public bool IsAreaFixed()
    {
        return isFixed;
    }

    void Start()
    {
        InitializeLineRenderers();

        if (rocketAgent != null)
        {
            rocketAgent.enabled = false;
        }
    }

    void InitializeLineRenderers()
    {
        GameObject previewObject = new GameObject("PreviewBorder");
        previewObject.transform.parent = transform;
        previewLineRenderer = previewObject.AddComponent<LineRenderer>();
        SetupLineRenderer(previewLineRenderer);
        previewLineRenderer.enabled = false;

        GameObject fixedObject = new GameObject("FixedBorder");
        fixedObject.transform.parent = transform;
        fixedLineRenderer = fixedObject.AddComponent<LineRenderer>();
        SetupLineRenderer(fixedLineRenderer);
        fixedLineRenderer.enabled = false;
    }

    void SetupLineRenderer(LineRenderer lineRenderer)
    {
        lineRenderer.material = lineMaterial;
        lineRenderer.startWidth = 5f;
        lineRenderer.endWidth = 5f;
        lineRenderer.positionCount = 5;
        lineRenderer.loop = true;
        lineRenderer.startColor = Color.yellow;
        lineRenderer.endColor = Color.yellow;
    }

    void Update()
    {
        if (!isFixed)
        {
            HandleAreaSelection();
        }
    }

    void HandleAreaSelection()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                selectionPoint = hit.point;
                DrawBorder(hit.collider, previewLineRenderer);
                previewLineRenderer.enabled = true;
            }
        }
        else if (Input.GetMouseButtonDown(1) && previewLineRenderer.enabled)
        {
            FixCurrentBorder();
            EnableRocket();
        }
    }

    void DrawBorder(Collider terrainCollider, LineRenderer lineRenderer)
    {
        Vector3[] corners = new Vector3[5];
        Vector3[] cornerOffsets = new Vector3[]
        {
            new Vector3(-squareSize/2, 0, -squareSize/2),
            new Vector3(-squareSize/2, 0, squareSize/2),
            new Vector3(squareSize/2, 0, squareSize/2),
            new Vector3(squareSize/2, 0, -squareSize/2),
            new Vector3(-squareSize/2, 0, -squareSize/2)
        };

        for (int i = 0; i < corners.Length; i++)
        {
            Vector3 worldPoint = selectionPoint + cornerOffsets[i];
            Ray ray = new Ray(worldPoint + Vector3.up * 10, Vector3.down);

            if (terrainCollider.Raycast(ray, out RaycastHit hit, 20f))
            {
                corners[i] = hit.point + Vector3.up * 0.5f;
            }
            else
            {
                corners[i] = worldPoint + Vector3.up * 0.5f;
            }
        }

        lineRenderer.SetPositions(corners);
        savedCorners = corners;
    }

    void FixCurrentBorder()
    {
        fixedLineRenderer.SetPositions(savedCorners); 
        fixedLineRenderer.enabled = true;
        previewLineRenderer.enabled = false;
        isFixed = true;
    }

    void EnableRocket()
    {
        if (rocketAgent != null)
        {
            rocketAgent.enabled = true;
        }
    }

    public void ResetAll()
    {
        isFixed = false;
        fixedLineRenderer.enabled = false;
        previewLineRenderer.enabled = false;
        if (rocketAgent != null)
        {
            rocketAgent.enabled = false;
        }
    }

    public Vector3 GetSelectedCenter()
    {
        if (!isFixed) return Vector3.zero;
        return selectionPoint;
    }
}