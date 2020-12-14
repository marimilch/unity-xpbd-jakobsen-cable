using System;
using Functional;
using UnityEngine;

public class SplineTools
{
    public static Returns<Vector3>.Expects<float> CreateGluedFunction(
        Returns<Vector3>.Expects<float>[] fs
    )
    {
        if (fs.Length < 1)
        {
            throw new UnityException(
                "CreateGluedFunction did not receive enough functions"
            );
        }
        return (float t) =>
        {
            //edge cases first
            //first function
            if (t < 1f)
            {
                return fs[0](t);
            }

            //last function
            var lastIndex = fs.Length - 1;
            var fLength = (float)lastIndex;
            if (t >= fLength)
            {
                return fs[lastIndex](t - fLength);
            }

            //in between function
            var t_ = t % 1f;
            var fIndex = Mathf.FloorToInt(t);

            return fs[fIndex](t_);
        };
    }

    public static Returns<Vector3>.Expects<float> CreateNormalizedFunction(
        float start,
        float end,
        Returns<Vector3>.Expects<float> f
    )
    {
        return (t) =>
        {
            var t_ = start + t * (end - start);
            return f(t_);
        };
    }

    public static Vector3[] AliasFunctionDynamically(
        Returns<Vector3>.Expects<float> fNormalized,
        int numberOfControlPoints,
        int resBetweenControlPoints = 1
    )
    {
        //var stepLength =
        //    (end - start)/resolution
        //;
        var dynamicResolution = GetDynamicResolution
        (
            numberOfControlPoints,
            resBetweenControlPoints
        );

        return AliasFunction(fNormalized, dynamicResolution);
    }

    public static Vector3[] AliasFunction(
        Returns<Vector3>.Expects<float> fNormalized,
        int res
    )
    {
        var stepLength = 1f / res;
        var input = new Vector3[res + 1];

        var output = Returns<Vector3>.Map<Vector3>(input, (v, i) =>
        {
            //var x = start + i * stepLength;
            var x = i * stepLength;
            return fNormalized(x);
        });

        //aliasPoints = output;

        return output;
    }

    public static float GetFunLength(
        Returns<Vector3>.Expects<float> fNormalized,
        int res = 1000
    )
    {
        var ps = AliasFunction(fNormalized, res);

        return GetFunLength(ps);
    }

    public static float GetFunLength(
        Vector3[] ps
    )
    {
        var len = 0f;

        for (int i = 0; i < ps.Length - 1; i++)
        {
            len += (ps[i + 1] - ps[i]).magnitude;
        }

        return len;
    }

    static int GetDynamicResolution(int nControlPoints, int res = 1)
    {
        return res * (nControlPoints - 1);
    }
}
