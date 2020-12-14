using System;
using UnityEngine;
public class Particle
{
    public Vector3 position;
    public Vector3 velocity;

    public Particle(Vector3 _position, Vector3 _velocity)
    {
        position = _position; 
        velocity = _velocity;
    }
}
