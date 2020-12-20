using System;
using Functional;
using UnityEngine;
using NumericalVectors;
using ArrayTools;

public class Constraint
{
    //the constraint function itself
    Returns<float>.Expects<Vector3[]> f;

    //satisfied if equal or unequal to zero
    bool equal = true;

    int cardinality = 1;

    float stiffness = .5f;

    //internal vars
    float currentTotalLagrange = 0f;

    float alpha_snake = 0f;

    //output vars
    float deltaLambda;
    Vector3[] deltaXsFull;

    public Constraint(
        Returns<float>.Expects<Vector3[]> f,
        bool equal,
        int cardinality,
        float stiffness
    )
    {
        this.f = f;
        this.equal = equal;
        this.cardinality = cardinality;
        this.stiffness = stiffness;

        //calculate internals
        alpha_snake = stiffness / (Time.fixedDeltaTime * Time.fixedDeltaTime);
    }

    //project to all vertices
    public void ProjectContraintsAll(ref Rigidbody[] rbsAll)
    {
        for (int i = 0; i <= rbsAll.Length - cardinality; ++i)
        {
            var deltaXs = ProjectConstraint(
                Arr<Rigidbody>.Extract(rbsAll, i, cardinality)
            );

            ApplyDeltaXFrom(ref rbsAll, ref deltaXs, i);
        }
    }

    void ApplyDeltaXFrom(
        ref Rigidbody[] rbsAll,
        ref Vector3[] deltaXs,
        int offset
    )
    {
        for (int i = 0; i < deltaXs.Length; ++i)
        {
            rbsAll[i + offset].MovePosition(
                 rbsAll[i + offset].transform.position + deltaXs[i]
            );
        }
    }

    //project only to cardinal vertices
    Vector3[] ProjectConstraint(Rigidbody[] rbs)
    {
        var xis = GetLocations(rbs);
        var cxi = Derivates.Nabla(f, xis.Length)(xis);
        var cm = ApplyMassDivision(rbs, cxi);
        var cmc = ScalarEach(
            cm,
            cxi
        );


        deltaLambda = (-f(xis) - alpha_snake * currentTotalLagrange) /
            (cmc + alpha_snake);

        var deltaXs = Returns<Vector3>.Map<Vector3>(cm, (v) =>
        {
            return v * deltaLambda;
        });

        currentTotalLagrange += deltaLambda;

        return deltaXs;
    }

    //public Vector3[] GetDeltaXs()
    //{
    //    return deltaXs;
    //}

    //public float GetDeltaLambda()
    //{
    //    return deltaLambda;
    //}

    Vector3[] GetLocations(Rigidbody[] rbs)
    {
        var len = rbs.Length;
        var rs = new Vector3[len];

        for (int i = 0; i < len; i++)
        {
            rs[i] = rbs[i].transform.position;
        }

        return rs;
    }

    float ScalarEach(Vector3[] xis, Vector3[] yis)
    {
        var r = 0f;
        var lenYis = yis.Length;
        var lenXis = xis.Length;

        if (lenYis != lenXis)
        {
            throw new UnityException("yis and xis lengths do not match");
        }

        for (int i = 0; i < lenXis; ++i)
        {
            r += Vector3.Dot(xis[i], yis[i]);
        }

        return r;
    }

    Vector3[] ApplyMassDivision(Rigidbody[] rbs, Vector3[] xis)
    {
        var lenRbs = rbs.Length;
        var lenXis = xis.Length;

        if (lenRbs != lenXis)
        {
            throw new UnityException("rbs and xis lengths do not match");
        }

        var rs = new Vector3[lenXis];

        for (int i = 0; i < lenXis; ++i)
        {
            rs[i] = xis[i] / rbs[i].mass;
        }

        return rs;
    }
}
