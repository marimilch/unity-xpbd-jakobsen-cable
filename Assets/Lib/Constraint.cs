using System;
using Functional;
using NumericalVectors;
using UnityEngine;
using ArrayTools;

class Constraint
{
    // from the paper
    int cardinality;
    bool equal;
    float stiffness;

    //xpbd
    float lambda = 0f;
    bool extended;

    Returns<float>.Expects<Vector3[]> constraintFunction;
    Returns<Vector3[]>.Expects<Vector3[]> constraintSatisfier;
    Returns<Vector3[]>.Expects<Vector3[]> nabla;

    void BaseInit(
        int cardinality_,
        Returns<float>.Expects<Vector3[]> constraintFunction_,
        float stiffness_ = 1f,
        bool equal_ = true,
        bool extended_ = true
    )
    {
        stiffness = stiffness_;
        equal = equal_;
        cardinality = cardinality_;
        constraintFunction = constraintFunction_;
        extended = extended_;
    }

    public Constraint(
        int cardinality_,
        Returns<float>.Expects<Vector3[]> constraintFunction_,
        float stiffness_ = 1f,
        bool equal_ = true,
        bool extended_ = true
    )
    {
        BaseInit(cardinality_, constraintFunction_, stiffness_, equal_, extended_);
        CreateNablaAndSolver();
    }

    public Constraint(
        int cardinality_,
        Returns<float>.Expects<Vector3[]> constraintFunction_,
        Returns<Vector3[]>.Expects<Vector3[]> constraintSatisfier_,
        float stiffness_ = 1f,
        bool equal_ = true,
        bool extended_ = true
    )
    {
        BaseInit(cardinality_, constraintFunction_, stiffness_, equal_, extended_);
        constraintSatisfier = constraintSatisfier_;
    }

    public void ProjectConstraint(int i, ref Vector3[] ps)
    {
        //ensure required points available
        if (RequirePoints(cardinality, i, ref ps)) return;

        var vs = Arr<Vector3>.Extract(ps, i, cardinality, false);

        //equality constraint?
        if (!equal && constraintFunction(vs) < 0f) return;

        var newVs = constraintSatisfier(vs);

        for (int j = 0; j < cardinality; ++j)
        {
            ps[i + j] = newVs[j];
        }
    }

    void CreateNablaAndSolver()
    {
        nabla = Derivatives.Nabla(
            constraintFunction,
            cardinality
        );

        constraintSatisfier = (vs) =>
        {
            //get directional nablas
            var directionalNablas = nabla(vs);

            //catch zero (no projection needed then)
            var sum = SumOfVectorsQuad(directionalNablas);
            if (sum == 0f)
            {
                return vs;
            }

            var s = constraintFunction(vs) / sum;

            //for each point, that the constraint checks now
            for (int i = 0; i < cardinality; i++)
            {
                vs[i] -= s * directionalNablas[i];
            }

            return vs;
        };
    }

    float SumOfVectorsQuad(Vector3[] vs)
    {
        var result = 0f;
        foreach (var v in vs)
        {
            result += v.sqrMagnitude;
        }

        return result;
    }

    bool RequirePoints(int n, int i, ref Vector3[] ps)
    {
        if (i + n <= ps.Length)
        {
            return false;
        }

        return true;
    }
}