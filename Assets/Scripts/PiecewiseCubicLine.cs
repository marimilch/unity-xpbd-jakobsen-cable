using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Functional;
using ArrayTools;

public class PiecewiseCubicLine : MonoBehaviour
{
    static readonly float TANGENT_MAG_FACTOR = .25f;

    [Header("Main Properties")]
    [SerializeField] private CurveType curveType = CurveType.Cubic;
    [Range(1, 100)]
    [SerializeField] private int resolution = 25;

    [Header("Curve Properties")]
    [SerializeField] public Vector3[] controlPoints;

    //[Header("Debug Information")]
    //[SerializeField] private Vector3[] aliasPoints;
    [SerializeField] private float[] bezierVals;
    //[SerializeField] private float[] binomials;

    // Start is called before the first frame update
    void Start()
    {

    }

    private void Reset()
    {
        controlPoints = new Vector3[]{
            new Vector3(1,0,0),
            new Vector3(2,0,0),
            new Vector3(3,0,0),
            new Vector3(4,0,0)
        };
    }

    void ExceptionIfNotLength(int len, Vector3[] points)
    {
        if (points.Length != len)
        {
            throw new UnityException("Piecewise function needs exactly " + len + " points.");
        }
    }

    void ExceptionIfNotAtLeast(int len, Vector3[] points)
    {
        if (points.Length < len)
        {
            throw new UnityException("Piecewise function needs at least " + len + " points.");
        }
    }

    void ExceptionIfNotLength4(Vector3[] points)
    {
        ExceptionIfNotLength(4, points);
    }

    Returns<Vector3>.Expects<float> CreateCubicLineFunction(Vector3[] points)
    {
        ExceptionIfNotAtLeast(4, points);

        return (x) =>
        {
            var yFun = CreateCubicFunction(
                GetCoefficientsForOneAxis(points)
            );

            var zFun = CreateCubicFunction(
               GetCoefficientsForOneAxis(points, true)
            );

            return new Vector3(x, yFun(x), zFun(x));
        };
    }

    Returns<Vector3>.Expects<float> CreateNormalizedCubicLineFunction(Vector3[] points) { 
        return CreateNormalizedFunction(
            points[0].x,
            points[3].x,
            CreateCubicLineFunction(points)
        );
    }

    Returns<float>.Expects<float> CreateCubicFunction(Vector4 cs)
    {
        return (float x) =>
        {
            return
                cs.x +
                cs.y * x +
                cs.z * Mathf.Pow(x, 2) +
                cs.w * Mathf.Pow(x, 3)
            ;
        };
    }

    Returns<Vector3>.Expects<float> CreateAutoCubicBezierFunction(Vector3[] controlPoints)
    {
        return CreateCubicBezierSplineFunction(
            CreateAutoCubicBezierPoints(controlPoints)
        );
            
    }

    Vector4 GetCoefficientsForOneAxis(Vector3[] points, bool getZ = false)
    {
        ExceptionIfNotAtLeast(4, points);

        var m = new Matrix4x4();

        var ys = new float[4];

        for (int i = 0; i < 4; ++i)
        {
            var p = points[i];
            var x = p.x;
            m.SetRow(i, new Vector4(
                1,
                x,
                Mathf.Pow(x, 2),
                Mathf.Pow(x, 3)
            ));

            ys[i] = getZ ? p.z : p.y;
        }

        var yVec = new Vector4(
            ys[0],
            ys[1],
            ys[2],
            ys[3]
        );

        var coefficients = GetAFromMAndY(m, yVec);
        //Debug.Log(coefficients);

        return coefficients;
    }

    Vector4 GetAFromMAndY(Matrix4x4 m, Vector4 y)
    {
        //if the there is no solution (determinant = 0) Vector4.zero is returned
        return m.inverse * y;
    }

    Returns<Vector3>.Expects<float> CreateNormalizedFunction(
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

    Vector3[] AliasFunction(
        Returns<Vector3>.Expects<float> fNormalized
    )
    {
        //var stepLength =
        //    (end - start)/resolution
        //;
        var dynamicResolution = resolution * (controlPoints.Length - 1);

        var stepLength = 1f / dynamicResolution;
        var input = new Vector3[dynamicResolution + 1];

        var output = Returns<Vector3>.Map<Vector3>(input, (v, i) =>
        {
            //var x = start + i * stepLength;
            var x = i * stepLength;
            return fNormalized(x);
        });

        //aliasPoints = output;

        return output;
    }

    //Vector3[] DoubleBetweenPoints(Vector3[] ps)
    //{
    //    var len = ps.Length;

    //    if (len <= 2)
    //    {
    //        return ps;
    //    }

    //    var rLen = (len - 2) * 2 + 2;
    //    var r = new Vector3[rLen];

    //    r[0] = ps[0];
    //    r[rLen - 1] = ps[len - 1];

    //    for (int i = 1; i < len - 1; i++)
    //    {
    //        var rIndex = 2 * i;
    //        r[rIndex - 1] = ps[i];
    //        r[rIndex] = ps[i];
    //    }

    //    return r;
    //}

    int GetNumberOfCubicBezierPoints(int lenControlPoints = 3, int endPointsLen = 2)
    {
        return (lenControlPoints - 2) * 3 + 2 * endPointsLen;
    }


    Vector3[] CreateAutoCubicBezierPoints(
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

    //Returns<Vector3>.Expects<float> CreateSplineFromCurve(
    //    Returns<Returns<Vector3>.Expects<float>>.Expects<Vector3[]> curveCreator,
    //    Vector3[] cps,
    //    int requiredControlPoints = 4
    //)
    //{

    //    var len = cps.Length;
    //    int count = Mathf.FloorToInt((cps.Length - requiredControlPoints) / (requiredControlPoints - 1));
    //    int end = requiredControlPoints + (count * (requiredControlPoints - 1));
    //    //var end = len / requiredControlPoints;
    //    //int end = len / requiredControlPoints;

    //    return CreateNormalizedFunction(0f, end,
    //        (t) =>
    //        {
    //            int cpsOffset = Mathf.Min(
    //                Mathf.FloorToInt( t * (requiredControlPoints - 1) ), len-1
    //            );

    //            float t_ = t % 1f;

    //            Vector3[] controlPointsToPass = Arr<Vector3>.Extract(
    //                cps,
    //                cpsOffset,
    //                requiredControlPoints
    //            );

    //            if (controlPointsToPass.Length - cpsOffset < requiredControlPoints)
    //            {
    //                return CreateDot(cps[len - 1])(t_);
    //            }

    //            return curveCreator(controlPointsToPass)(t_);
    //        }
    //    );
    //}

    Returns<Vector3>.Expects<float> CreateDot(Vector3 p)
    {
        return (t) =>
        {
            return p;
        };
    }

    Returns<Vector3>.Expects<float> CreateCubicBezierFunction(
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

    Returns<Vector3>.Expects<float> CreateCubicBezierSplineFunction(
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
            var ps = Arr<Vector3>.Extract(controlPoints, 3*i, 4);
            fArray[i] = CreateCubicBezierFunction(ps);
        }

        //var end = ((float)(cps.Length - 1)) / 3f + 1f;
        var end = (float) fArrayLength;

        //Debug.Log(end);

        return CreateNormalizedFunction(
            0f,
            end,
            CreateGluedFunction(
                fArray
            )
        );
    }

    Returns<Vector3>.Expects<float> CreateGluedFunction(
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

    //Returns<Vector3>.Expects<float> CreateCatmullCurveFunction(
    //    Vector3[] controlPoints
    //)
    //{
    //    ExceptionIfNotLength4(controlPoints);

    //    return (t) =>
    //    {
    //        var r = new Vector3[controlPoints.Length * 3];

    //        for (int i = 1; i < 3; ++i)
    //        {
    //            var tangent = TANGENT_MAG_FACTOR * (
    //                controlPoints[i + 1] - controlPoints[i - 1]
    //            );

    //            var leftBound = 3 * (i - 1);

    //            r[leftBound] = controlPoints[i] - tangent;
    //            r[leftBound + 1] = controlPoints[i];
    //            r[leftBound + 2] = controlPoints[i] + tangent;
    //        }

    //        return CreateCubicBezierFunction(r)(t);
    //    };
    //}

    Returns<float>.Expects<float> BernsteinPolynomial(int n, int k)
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

    float BinomialCoefficient(int n, int k)
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

    float Faculty(int n, int k_ = 1)
    {
        float r = 1f;

        int k = Mathf.Max(1, k_);

        for (int i = n; i > k; --i)
        {
            r *= i;
        }

        return r;
    }

    Returns<Vector3>.Expects<float> GetSelectedFunctionType()
    {
        Returns<Returns<Vector3>.Expects<float>>.Expects<Vector3[]> creator;
        switch (curveType)
        {
            
            case CurveType.Cubic:
                creator = CreateNormalizedCubicLineFunction;
                break;
            case CurveType.CatmullRom:
                //return CreateAutoCubicBezierFunction(controlPoints);
                creator = CreateAutoCubicBezierFunction;
                break;
            default:
                creator = CreateNormalizedCubicLineFunction;
                break;
        }

        //return CreateSplineFromCurve(creator, controlPoints, 4);
        return creator(controlPoints);
    }

    public Vector3[] GetRenderedPoints()
    {
        //ExceptionIfNotLength4(controlPoints);

        //RunTests();

        //return CreateAutoCubicBezierPoints(controlPoints);

        return AliasFunction(
            GetSelectedFunctionType()
        );
    }

    //private void RunTests()
    //{
    //    int n = 3;
    //    int k = 3;
    //    float step = 1/10f;

    //    bezierVals = new float[n * k * resolution];
    //    binomials = new float[n * k];

    //    for (int j = 0; j < n; ++j)
    //    {
    //        for (int k_ = 0; k_ < k; ++k_)
    //        {
    //            for (int i = 0; i < resolution; ++i)
    //            {
    //                bezierVals[j + k_ * n + i * k * n] =
    //                BernsteinPolynomial(j, k_)(i * step);
    //            }

    //            binomials[j + k_ * n] = BinomialCoefficient(j, k_);
    //        }
    //    }
    //}
}
