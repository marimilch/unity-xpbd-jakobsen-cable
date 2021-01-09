using UnityEngine;
using System.Collections;

public class DefaultGrabHandler : MonoBehaviour
{
    public int jointNumber;
    public VerletCable verletCable;
    Camera mainCamera;
    Transform mt;

    Vector3 p1Start;
    Vector3 p2Start;

    Vector3 start;

    Vector3 mousePositionRaw;

    private void Start()
    {
        mainCamera = Camera.main;
        mt = mainCamera.transform;

        if (!verletCable)
        {
            throw new UnityException("Verlet Cable in parent missing.");
        }
    }

    private void Update()
    {
        mousePositionRaw = Input.mousePosition;
    }

    private void OnMouseDown()
    {
        start = MouseOnPlane();

        p1Start = verletCable.currentXs[jointNumber];

        //constraining two poisitions causes interseection with otther bodies
        //so we opt for one instead
        //p2Start = verletCable.currentXs[jointNumber + 1];

        //Debug.Log("Mouse start: " + start);
    }

    private void OnMouseDrag()
    {
        var delta = MouseOnPlane() - start;
        verletCable.SetGrab(jointNumber, p1Start + delta);
        //verletCable.SetGrab(jointNumber + 1, p2Start + delta);

        //Debug.Log("Dragged to: " + MouseOnPlane(transform.position.z));
    }

    private float calcDistance()
    {
        return transform.position.z - mt.position.z;
    }

    private void OnMouseUp()
    {
        verletCable.EndGrab(jointNumber);
        //verletCable.EndGrab(jointNumber + 1);
    }

    private Vector3 MouseOnPlane()
    {
        //Debug.Log("Mouse position raw: " + Input.mousePosition);

        //Debug.Log("Mouse position world: " + mousePos);

        return Vector3.ProjectOnPlane(
            mainCamera
            .ScreenPointToRay(mousePositionRaw)
            .GetPoint(calcDistance()),
            Vector3.forward
        );
    }
}
