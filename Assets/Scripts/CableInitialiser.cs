using UnityEngine;
using System.Collections;

public class CableInitialiser : MonoBehaviour
{
    public Vector3[] controlPoints;
    //editor view in CableInitialiserInspector.cs

    private void Reset()
    {
        controlPoints = new Vector3[]{
            new Vector3(1,0,0),
            new Vector3(2,0,0)
        };
    }
}
