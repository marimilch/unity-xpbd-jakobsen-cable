using System;
namespace Functional
{
    public class Returns<R>
    {
        public delegate R Expects();
        public delegate R Expects<T1>(T1 t);
        public delegate R Expects<T1, T2>(T1 t1, T2 t2);

        public static R[] Map<T>(T[] ts, Expects<T, int> map)
        {
            R[] rs = new R[ts.Length];
            for (int i = 0; i < ts.Length; i++)
            {
                rs[i] = map(ts[i], i);
            }

            return rs;
        }

        public static R[] Map<T>(T[] ts, Expects<T> map)
        {
            Expects<T, int> map_ = (e, i) =>
            {
                return map(e);
            };
            return Map<T>(ts, map_);
        }
    }
}
