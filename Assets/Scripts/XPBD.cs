using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Functional;

public class XPBD : MonoBehaviour
{
    [Range(1,25)]
    [SerializeField] private uint solverIterations = 1;

    Constraint[] constraints;
    Rigidbody[] rigidbodies;

    bool ready = false;

    public void Attach(
        Constraint[] constraints,
        Rigidbody[] rigidbodies,
        uint solverIterations = 1
    )
    {
        this.constraints = constraints;
        this.rigidbodies = rigidbodies;
        this.solverIterations = solverIterations;

        ready = true;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (ready)
        {
            XPBDSimulation();
        }
    }

    void XPBDSimulation()
    {
        var xSnakes = Returns<Vector3>.Map<Rigidbody>(rigidbodies, (b) =>
        {
            return PredictRigidBodyPosSingle(b);
        });

        //apply x_snake vector position
        InitXNPlusOne(ref rigidbodies, ref xSnakes);

        for (int j = 0; j < constraints.Length; ++j)
        {
            for (int i = 0; i < solverIterations; ++i)
            {
                constraints[j].ProjectContraintsAll(
                    ref rigidbodies
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
