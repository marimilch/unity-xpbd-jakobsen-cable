using UnityEngine;
using System.Collections;

public class CablePointsProvider : MonoBehaviour
{
    LineRenderer lineRenderer;
    Cable verletCable;

    // Use this for initialization
    void Start()
    {
        lineRenderer = GetComponent<LineRenderer>();

        //get first active CablePointsProvider
        verletCable = gameObject.GetActiveInterface<Cable>();

        if (!lineRenderer || verletCable == null)
        {
            throw new UnityException(
                "Line Renderer or Verlet " +
                "Cable missing or not active."
            );
        }

        lineRenderer.positionCount = verletCable.GetNumberOfParticles();
        var diameter = 2f * verletCable.GetRadius();
        lineRenderer.startWidth = diameter;
        lineRenderer.endWidth = diameter;
    }

    private void FixedUpdate()
    {
        lineRenderer.SetPositions(verletCable.GetParticles());
    }
}
