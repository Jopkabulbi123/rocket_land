using UnityEngine;

public class CameraFollowYAxis : MonoBehaviour
{
    public Transform rocket;
    public float xOffset = 10f;
    public float yOffset = 2f;
    public float zOffset = 0f;

    private Vector3 initialPosition;
    private bool isFollowing = false;

    void Start()
    {
        initialPosition = transform.position;
    }

    void LateUpdate()
    {
        if (rocket != null)
        {
            if (!isFollowing && Input.GetMouseButtonDown(1))
            {
                isFollowing = true;
                transform.position = new Vector3(
                    rocket.position.x + xOffset,
                    rocket.position.y + yOffset,
                    rocket.position.z + zOffset
                );
            }

            if (isFollowing)
            {
                transform.position = new Vector3(
                    rocket.position.x + xOffset,
                    rocket.position.y + yOffset,
                    rocket.position.z + zOffset
                );

                transform.LookAt(rocket);
            }
        }
    }

    public void ResetCamera()
    {
        isFollowing = false;
        transform.position = initialPosition;
        transform.rotation = Quaternion.identity;
    }
}