using UnityEngine;
using System.Collections;

public class CablePointsProvider : MonoBehaviour
{
    LineRenderer lineRenderer;
    VerletCable verletCable;

    // Use this for initialization
    void Start()
    {
        lineRenderer = GetComponent<LineRenderer>();
        verletCable = GetComponent<VerletCable>();
        if (!lineRenderer || !verletCable)
        {
            throw new UnityException("Line Renderer or Verlet Cable missing.");
        }

        lineRenderer.positionCount = verletCable.GetNumberOfParticles();
        var diameter = 2f * verletCable.radius;
        lineRenderer.startWidth = diameter;
        lineRenderer.endWidth = diameter;
    }

    private void FixedUpdate()
    {
        lineRenderer.SetPositions(verletCable.currentXs);
    }
}
