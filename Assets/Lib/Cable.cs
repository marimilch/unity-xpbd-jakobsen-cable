using System;
using UnityEngine;

public interface Cable
{
    Vector3[] GetParticles();
    int GetNumberOfParticles();
    float GetRadius();

    void SetGrab(int i, Vector3 to, bool soft = true);

    void EndGrab(int i);

    bool IsGrabbed(int i);

    void SetMaxVelocity(float v);
}
