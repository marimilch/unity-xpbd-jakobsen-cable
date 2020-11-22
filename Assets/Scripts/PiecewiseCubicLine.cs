using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Functional;

public class PiecewiseCubicLine : MonoBehaviour
{
    [SerializeField] public Vector3[] controlPoints;

    [Range(1, 10000)]
    [SerializeField] private int resolution = 100;

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
            throw new UnityException("Piecewise function needs exactly four points.");
        }
    }

    void ExceptionIfNotLength4(Vector3[] points)
    {
        ExceptionIfNotLength(4, points);
    }

    Returns<Vector2>.Expects<float> createCubicLineFunction(Vector3[] points)
    {
        ExceptionIfNotLength4(points);

        return (x) =>
        {
            var yFun = createCubicFunction(
                GetCoefficientsForOneAxis(points)
            );

            var zFun = createCubicFunction(
               GetCoefficientsForOneAxis(points, true)
            );

            return new Vector2(yFun(x), zFun(x));
        };
    }

    Returns<float>.Expects<float> createCubicFunction(Vector4 cs)
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

    Vector4 GetCoefficientsForOneAxis(Vector3[] points, bool getZ = false)
    {
        ExceptionIfNotLength4(points);

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

    Vector3[] aliasFunction(
        float start,
        float end,
        Returns<Vector2>.Expects<float> f
    )
    {
        var stepLength =
            (end - start)/resolution
        ;

        var r = new Vector3[resolution + 1];

        return Returns<Vector3>.Map<Vector3>(r, (v, i) =>
        {
            var x = start + i * stepLength;
            var fx = f(x);
            return new Vector3(
                x,
                fx.x,
                fx.y
            );
        });
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public Vector3[] GetRenderedPoints()
    {
        ExceptionIfNotLength4(controlPoints);

        return aliasFunction(
            controlPoints[0].x,
            controlPoints[3].x,
            createCubicLineFunction(controlPoints)
        );
    }
}
