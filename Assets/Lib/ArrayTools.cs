using System;
using UnityEngine;
namespace ArrayTools
{
    public class Arr<T>
    {
        public static T[] Extract(T[] ts, int index, int len_)
        {
            var len = Mathf.Min(len_, ts.Length - index);
            var r = new T[len];
            for(int i = 0; i < len; ++i)
            {
                r[i] = ts[index + i];
            }

            return r;
        }
    }
}
