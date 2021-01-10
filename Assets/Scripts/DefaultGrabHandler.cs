using UnityEngine;
using System.Collections;

public class DefaultGrabHandler : MonoBehaviour
{
    public int jointNumber;
    public JakobsenCable verletCable;
    Camera mainCamera;
    Transform mt;

    Vector3 p1Start;
    Vector3 p2Start;

    Vector3 start;

    Vector3 mousePositionRaw;

    //float initialStiffness;

    public float maxVelocityOnMove = .25f;

    private void Start()
    {
        mainCamera = Camera.main;
        mt = mainCamera.transform;
        //initialStiffness = verletCable.stiffness;

        if (!verletCable)
        {
            throw new UnityException("Jakobsen Cable in parent missing.");
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
        //verletCable.stiffness = 0f;

        //constraining two poisitions causes interseection with otther bodies
        //so we opt for one instead
        //p2Start = verletCable.currentXs[jointNumber + 1];

        //Debug.Log("Mouse start: " + start);
    }

    private void OnMouseDrag()
    {
        var delta = MouseOnPlane() - start;
        verletCable.SetGrab(jointNumber, p1Start + delta, false);
        verletCable.maxVelocity = maxVelocityOnMove;
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
        verletCable.maxVelocity = 0f;
        //verletCable.stiffness = initialStiffness;
        //verletCable.EndGrab(jointNumber + 1);
    }

    private Vector3 MouseOnPlane()
    {
        //Debug.Log("Mouse position raw: " + Input.mousePosition);

        //Debug.Log("Mouse position world: " + mousePos);

        //Debug.Log(mainCamera.ScreenToWorldPoint(mousePositionRaw));

        //mainCamera.ScreenToWorldPoint(mousePositionRaw) does not work,
        // so workaround with ray
        return IntersectionLinePlane(
            mainCamera.transform.position,
            mainCamera.ScreenPointToRay(mousePositionRaw).GetPoint(.01f),
            verletCable.currentXs[jointNumber],
            mainCamera.transform.TransformDirection(Vector3.forward)
        )
           ;

        //return Vector3.ProjectOnPlane(
        //    mainCamera
        //    .ScreenPointToRay(mousePositionRaw)
        //    .GetPoint(calcDistance()),
        //    Vector3.forward
        //);
    }

    Vector3 IntersectionLinePlane(
        Vector3 x1,
        Vector3 x2,
        Vector3 p0,
        Vector3 n
    )
    {
        var l0 = x1;
        var l = (x2 - x1).normalized;
        var dotln = Vector3.Dot(l, n);
        if (dotln == 0f)
        {
            return Vector3.zero;
        }
        var d = Vector3.Dot(p0 - l0, n) / dotln;

        return l0 + l * d;
    }
}
