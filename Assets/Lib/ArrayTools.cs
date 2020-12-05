using System;
using UnityEngine;
namespace ArrayTools
{
    public class Arr<T>
    {
        public static T[] Extract(
            T[] ts,
            int index,
            int len_,
            bool extend = true
        )
        {
            var len = extend ? len_ : Mathf.Min(len_, ts.Length - index);
            var r = new T[len];
            for(int i = 0; i < len; ++i)
            {
                var tIndex = extend ? Mathf.Min(
                    Mathf.Max(0, index + i),
                    ts.Length - 1
                ) : index + i;
                r[i] = ts[tIndex];
            }

            return r;
        }
    }
}
