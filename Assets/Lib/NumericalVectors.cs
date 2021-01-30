using System;
using Functional;
using UnityEngine;

namespace NumericalVectors
{
    public class Derivatives
    {
        public static Returns<float>.Expects<float> Derivative(
            Returns<float>.Expects<float> f,
            float epsilon = .01f
        )
        {
            return (x) =>
            {
                return (f(x + epsilon) - f(x - epsilon)) / (2f * epsilon);
            };
        }

        public static Returns<float>.Expects<Vector3[]> PartialDerivative(
            Returns<float>.Expects<Vector3[]> f,
            int coordinate,
            float epsilon= .01f
        )
        {
            return (vs) =>
            {
                //which vector (integer division)
                var vi = coordinate / 3;
                var vij = coordinate % 3;

                //which component of it

                var dVec = new Vector3(
                    vij == 0 ? epsilon : 0f,
                    vij == 1 ? epsilon : 0f,
                    vij == 2 ? epsilon : 0f
                );

                var vsPlus = new Vector3[vs.Length];
                var vsMinus = new Vector3[vs.Length];

                vs.CopyTo(vsPlus, 0);
                vs.CopyTo(vsMinus, 0);

                vsPlus[vi] += dVec;
                vsMinus[vi] -= dVec;

                return (f(vsPlus) - f(vsMinus)) / (2f * epsilon);
            };
        }

        public static Returns<Vector3[]>.Expects<Vector3[]> Nabla(
            Returns<float>.Expects<Vector3[]> f,
            int arity,
            float epsilon = .01f
        )
        {
            var arity3 = arity * 3;
            var nabla = new Returns<float>.Expects<Vector3[]>[arity3];

            for (int i = 0; i < arity3; ++i)
            {
                nabla[i] = PartialDerivative(f, i, epsilon);
            }

            return (vs) =>
            {
                var rs = new Vector3[arity];

                for (int i = 0; i < arity; ++i)
                {
                    var i3 = i * 3;
                    var v = new Vector3(
                        nabla[i3](vs),
                        nabla[i3 + 1](vs),
                        nabla[i3 + 2](vs)
                    );

                    rs[i] = v;
                    //Debug.Log(rs[i] + " check2");
                }

                return rs;
            };
        }
    }
}
