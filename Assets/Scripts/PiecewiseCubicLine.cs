using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PiecewiseCubicLine : MonoBehaviour
{
    [SerializeField] public Vector3[] controlPoints;
    [Range(1, 10000)]
    [SerializeField] private int resolution = 100;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public Vector3[] GetRenderedPoints()
    {
        return controlPoints;
    }
}
