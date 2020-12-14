using System;
using UnityEngine;
public class ParticleArray
{
    const int nOfVars = 6;
    float[] pointsAndVelocities;

    public ParticleArray(int len)
    {
        pointsAndVelocities = new float[len * nOfVars];
    }

    public ParticleArray(Vector3[] ps)
    {
        var psLen = ps.Length;
        pointsAndVelocities = new float[psLen * nOfVars];

        for (int i = 0; i < psLen; ++i)
        {

        }
    }

    private int GetInternalIndex(int index)
    {
        return index * nOfVars;
    }

    public Particle GetParticle(int index_)
    {
        int index = GetInternalIndex(index_);
        return new Particle(
            new Vector3(
                pointsAndVelocities[index],
                pointsAndVelocities[index + 1],
                pointsAndVelocities[index + 2]
            ),
            new Vector3(
                pointsAndVelocities[index + 3],
                pointsAndVelocities[index + 4],
                pointsAndVelocities[index + 5]
            )
        );
    }
}
