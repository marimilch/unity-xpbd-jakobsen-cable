using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Functional;

public class XPBD : MonoBehaviour
{
    [Range(1,25)]
    [SerializeField] private uint solverIterations = 1;

    Vector3[] currentXs;
    float[] currentLambdas;

    Constraint[] constraints;

    // Start is called before the first frame update
    void Start()
    {
        currentXs = new Vector3[solverIterations];
        currentLambdas = new float[solverIterations];
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void XPBDSimulation(ref Rigidbody[] bsAll)
    {
        var xSnakes = Returns<Vector3>.Map<Rigidbody>(bsAll, (b) =>
        {
            return PredictRigidBodyPosSingle(b);
        });

        //apply x_snake vector position
        InitXNPlusOne(ref bsAll, ref xSnakes);

        currentLambdas[0] = 0f;

        for (int j = 0; j < constraints.Length; ++j)
        {
            for (int i = 0; i < solverIterations; ++i)
            {
                constraints[j].ProjectContraintsAll(
                    ref bsAll
                );
            }
        }    
    }

    void InitXNPlusOne(
        ref Rigidbody[] rbsAll,
        ref Vector3[] xs
    )
    {
        for (int i = 0; i < xs.Length; ++i)
        {
            rbsAll[i].MovePosition(
                 xs[i]
            );
        }
    }

    Vector3 PredictRigidBodyPosSingle(Rigidbody b)
    {
        var dTime = Time.fixedDeltaTime;

        //gravity is the only external force we have
        return
            b.position +
            dTime * b.velocity +
            dTime * dTime * b.mass * Physics.gravity;
    }
}
