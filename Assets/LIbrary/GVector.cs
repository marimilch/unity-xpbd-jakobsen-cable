using System;
public class GVector<T>
{
    T[] content;
    public GVector(int dimension)
    {
        content = new T[dimension];
    }

    private bool validRow(int row)
    {
        return row < 0 || row >= content.Length;
    }

    public T this[int row]
    {
        get
        {
            if (validRow(row))
            {
                return default(T);
            }

            return content[row];
        }
        set
        {
            if (validRow(row))
            {
                return;
            }

            content[row] = value;
        }
    }
}
