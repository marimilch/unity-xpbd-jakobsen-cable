using System;
using System.Collections.Generic;
using Functional;
using NumericalVectors;
using UnityEngine;
using ArrayTools;

class Constraint
{
    // statics
    public static float precision = .0001f;
    public static readonly float maxStiffnessExt = 100000f;
    public static readonly float minStiffnessExt = maxStiffnessExt / 100f;

    // from the paper
    int cardinality;
    bool equal;
    public float stiffness;

    //xpbd
    float[] lambdas;
    bool extended;

    //internal
    Returns<float>.Expects<Vector3[]> constraintFunction;
    Returns<Vector3[]>.Expects<Vector3[]> constraintSatisfier;
    Returns<Vector3[]>.Expects<Vector3[]> nabla;

    int particleIndex = 0;
    bool active = true;
    float deltaTSquared;
    int numExt;

    void BaseInit(
        int cardinality_,
        Returns<float>.Expects<Vector3[]> constraintFunction_,
        float stiffness_ = 1f,
        bool equal_ = true,
        bool extended_ = true,
        int numExt_ = 0
    )
    {
        deltaTSquared = Time.fixedDeltaTime * Time.fixedDeltaTime;
        stiffness = stiffness_;
        equal = equal_;
        cardinality = cardinality_;
        constraintFunction = constraintFunction_;
        extended = extended_;
        if (extended)
        {
            numExt = numExt_;
            lambdas = new float[numExt - cardinality + 1];
        }
    }

    public Constraint(
        int cardinality_,
        Returns<float>.Expects<Vector3[]> constraintFunction_,
        float stiffness_ = 1f,
        bool equal_ = true,
        bool extended_ = true,
        int numExt_ = 0
    )
    {
        BaseInit(cardinality_, constraintFunction_, stiffness_, equal_, extended_, numExt_);
        CreateNablaAndSolver();
    }

    public Constraint(
        int cardinality_,
        Returns<float>.Expects<Vector3[]> constraintFunction_,
        Returns<Vector3[]>.Expects<Vector3[]> constraintSatisfier_,
        float stiffness_ = 1f,
        bool equal_ = true,
        bool extended_ = true,
        int numExt_ = 0
    )
    {
        BaseInit(cardinality_, constraintFunction_, stiffness_, equal_, extended_, numExt_);
        constraintSatisfier = constraintSatisfier_;
    }

    public void ProjectConstraint(int i, ref Vector3[] ps)
    {
        //only when active
        if (!active) return;

        //ensure required points available
        if (RequirePoints(cardinality, i, ref ps)) return;

        particleIndex = i;

        var vs = Arr<Vector3>.Extract(ps, i, cardinality, false);

        //equality constraint?
        if (!equal && constraintFunction(vs) < 0f) return;

        var newVs = constraintSatisfier(vs);

        for (int j = 0; j < cardinality; ++j)
        {
            ps[i + j] = newVs[j];
        }
    }

    public void ResetLambda()
    {
        if (extended)
        {
            for (int i = 0; i < lambdas.Length; ++i)
            {
                lambdas[i] = 0f;
            };
        }
    }

    void CreateNablaAndSolver()
    {
        nabla = Derivatives.Nabla(
            constraintFunction,
            cardinality,
            precision
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

            float s;
            if (extended)
            {
                //recalculated, so it can be changed live
                var alphaSnake = 1f / (deltaTSquared *
                    (minStiffnessExt + XPBDStiffness())
                );

                s = XPBDScale(ref vs, alphaSnake, sum);

                lambdas[particleIndex] += s;
            } else
            {
                s = -stiffness * constraintFunction(vs) / sum;
            }

            //correct each particle's position
            for (int i = 0; i < cardinality; i++)
            {
                vs[i] += s * directionalNablas[i];
            }

            return vs;
        };
    }

    float XPBDScale(ref Vector3[] vs, float alphaSnake, float sum)
    {
        return (-constraintFunction(vs) - alphaSnake * lambdas[particleIndex]) /
                    (sum + alphaSnake);
    }

    float XPBDStiffness()
    {
        return stiffness * (maxStiffnessExt - minStiffnessExt);
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