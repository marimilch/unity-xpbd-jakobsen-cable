using System;
using Functional;
using UnityEngine;
using ArrayTools;

public class CatmullSpline
{
    static readonly float TANGENT_MAG_FACTOR = .25f;

    public static Returns<Vector3>.Expects<float> CreateCatmullSpline(Vector3[] controlPoints)
    {
        return CreateCubicBezierSplineFunction(
            CreateAutoCubicBezierPoints(controlPoints)
        );
    }

    static Vector3[] CreateAutoCubicBezierPoints(
        Vector3[] controlPoints
    )
    {
        ExceptionIfNotAtLeast(2, controlPoints);

        var len = controlPoints.Length;
        var endPointsLen = 2;
        var rLen = GetNumberOfCubicBezierPoints(len, endPointsLen);

        var r = new Vector3[rLen];

        //Set first two and last two points each
        var startTan = TANGENT_MAG_FACTOR * (controlPoints[1] - controlPoints[0]);
        r[0] = controlPoints[0];
        r[1] = controlPoints[0] + startTan;

        var lastIndex = len - 1;
        var lastRIndex = rLen - 1;
        var endTan = TANGENT_MAG_FACTOR * (controlPoints[lastIndex] - controlPoints[lastIndex - 1]);
        r[lastRIndex - 1] = controlPoints[lastIndex] - endTan;
        r[lastRIndex] = controlPoints[lastIndex];

        for (int i = 1; i < lastIndex; ++i)
        {
            var tangent = TANGENT_MAG_FACTOR * (
                controlPoints[i + 1] - controlPoints[i - 1]
            );

            var leftBound = 3 * (i - 1) + endPointsLen;

            r[leftBound] = controlPoints[i] - tangent;
            r[leftBound + 1] = controlPoints[i];
            r[leftBound + 2] = controlPoints[i] + tangent;
        }

        //Debug.Log("r size: " + rLen);
        //Debug.Log("len: " + len);
        return r;
    }

    static int GetNumberOfCubicBezierPoints(int lenControlPoints = 3, int endPointsLen = 2)
    {
        return (lenControlPoints - 2) * 3 + 2 * endPointsLen;
    }

    static Returns<Vector3>.Expects<float> CreateCubicBezierFunction(
        Vector3[] controlPoints
    )
    {
        ExceptionIfNotLength4(controlPoints);

        return (t) =>
        {
            var sum = Vector3.zero;
            var len = controlPoints.Length;
            var n = 3;

            for (int k = 0; k <= n; ++k)
            {
                sum += BernsteinPolynomial(n, k)(t) * controlPoints[k];
            }

            return sum;
        };
    }

    static void ExceptionIfNotAtLeast(int len, Vector3[] points)
    {
        if (points.Length < len)
        {
            throw new UnityException("Piecewise function needs at least " + len + " points.");
        }
    }

    static void ExceptionIfNotLength4(Vector3[] points)
    {
        ExceptionIfNotLength(4, points);
    }

    static void ExceptionIfNotLength(int len, Vector3[] points)
    {
        if (points.Length != len)
        {
            throw new UnityException("Piecewise function needs exactly " + len + " points.");
        }
    }

    static Returns<Vector3>.Expects<float> CreateCubicBezierSplineFunction(
        Vector3[] controlPoints
    )
    {
        //make sure there are 3n+1 points
        //not necessary since we use integer division afterwards
        //var cps = Arr<Vector3>.Extract(
        //    controlPoints_,
        //    0,
        //    len - ((len - 1) % 3)
        //);

        var fArrayLength = (controlPoints.Length - 1) / 3;
        var fArray = new Returns<Vector3>.Expects<float>[fArrayLength];

        for (int i = 0; i < fArrayLength; ++i)
        {
            var ps = Arr<Vector3>.Extract(controlPoints, 3 * i, 4);
            fArray[i] = CreateCubicBezierFunction(ps);
        }

        //var end = ((float)(cps.Length - 1)) / 3f + 1f;
        var end = (float)fArrayLength;

        //Debug.Log(end);

        return SplineTools.CreateNormalizedFunction(
            0f,
            end,
            SplineTools.CreateGluedFunction(
                fArray
            )
        );
    }

    static Returns<float>.Expects<float> BernsteinPolynomial(int n, int k)
    {
        return (t) =>
        {
            return
                BinomialCoefficient(n, k) *
                Mathf.Pow(t, k) *
                Mathf.Pow(1 - t, n - k)
            ;
        };
    }

    static float BinomialCoefficient(int n, int k)
    {
        if (n < k)
        {
            throw new UnityException("k > n in Binomial");
            //return 0f;
        }

        if (k == 0 || n == 0)
        {
            return 1f;
        }

        //float r = 1f;
        int nMinK = n - k;

        //for (int i = n; i > Mathf.Max(k, nMinK); --i)
        //{
        //    r *= i;
        //}

        //return r / Mathf.Min(k, nMinK);

        return Faculty(n, Mathf.Max(k, nMinK)) / Faculty(Mathf.Min(k, nMinK));
    }

    static float Faculty(int n, int k_ = 1)
    {
        float r = 1f;

        int k = Mathf.Max(1, k_);

        for (int i = n; i > k; --i)
        {
            r *= i;
        }

        return r;
    }
}
