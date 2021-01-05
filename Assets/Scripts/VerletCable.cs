using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VerletCable : MonoBehaviour
{
    [Range(2, 100)]
    [Tooltip("Will determine how many Joints there will be")]
    [SerializeField] private int resolution = 4;
    int numberOfParticles;

    [Tooltip("The radius of the cable")]
    [SerializeField] float radius = .25f;

    [Range(0f, 0.05f)]
    [SerializeField] float dampening = .01f;

    [Range(0f, 2f)]
    [Tooltip("Pseudo sliding friction")]
    [SerializeField] float slidingFriction = .1f;

    [Range(0f, 2f)]
    [Tooltip("Pseudo static friction")]
    [SerializeField] float staticFriction = .1f;

    [Range(0, 25)]
    [Tooltip("The more iterations, the preciser " +
        "the result. Increase this, if the cable falls through" +
        "objects. (0 for Unity global configuration)")]
    [SerializeField] int solverIterations = 0;

    [Tooltip("Whether to show particle positions as cubes")]
    [SerializeField] bool debugMode = true;

    float dt = 0f;
    float dt_squared = 0f;
    float restDistance = 0f;

    Vector3[] currentXs;
    Vector3[] previousXs;
    Vector3[] accelerations;

    Transform[] debugPoints;

    void Start()
    {
        var line = GetComponent<CableInitialiser>();
        numberOfParticles = resolution + 1;

        currentXs = SplineTools.AliasFunction(
            CatmullSpline.CreateCatmullSpline(line.controlPoints),
            resolution
        );

        var cableLength = SplineTools.GetFunLength(currentXs);
        restDistance = cableLength / (float)resolution;

        //set to world coordinates
        for (int i = 0; i < numberOfParticles; ++i)
        {
            currentXs[i] = transform.TransformPoint(
                currentXs[i]
            );
        }

        previousXs = new Vector3[numberOfParticles];
        accelerations = new Vector3[numberOfParticles];

        currentXs.CopyTo(previousXs, 0);

        if (debugMode)
        {
            CreateDebugPoints();
        }

        dt = Time.fixedDeltaTime;
        dt_squared = dt * dt;
    }

    void FixedUpdate()
    {
        // Set to default if necessary here already,
        // so it can be changed while play mode.

        if (solverIterations < 1)
        {
            solverIterations = Physics.defaultSolverIterations;
        }

        Simulate();
        if (debugMode)
        {
            SetDebugPoints();
        }
    }

    void SetDebugPoints()
    {
        for (int i = 0; i < numberOfParticles; ++i)
        {
            debugPoints[i].position = currentXs[i];
        }
    }

    void Integrate()
    {
        for (int i = 0; i < numberOfParticles; ++i)
        {
            ref var x = ref currentXs[i];
            ref var x_ = ref previousXs[i];
            ref var a = ref accelerations[i];

            var temp = x;

            x =
                x * (2 - dampening) - (1 - dampening) * x_ +
                a * dt_squared
            ;

            x_ = temp;
            //x_ = (x - temp).magnitude > Physics.defaultContactOffset ? temp : x;
        }
    }

    void SetForces()
    {
        for (int i = 0; i < numberOfParticles; ++i)
        {
            accelerations[i] = Physics.gravity;
        }
    }

    void SatisfyConstraints()
    {
        for (int i = 0; i < numberOfParticles; ++i)
        {
            ref var p = ref currentXs[i];


            DistanceConstraint(i);
            DistanceConstraint(i, 1);
            ParticleCollisionConstraint(i);
            //CollisionConstraint(i);

            //project out
            //p = Vector3.Min(
            //    Vector3.Max(p, new Vector3(0, 0, 0)),
            //    new Vector3(100, 100, 100)
            //);
        }
    }

    //uses two points
    void DistanceConstraint(int i, int interD = 0)
    {
        //requirements
        if (RequirePoints(2 + interD, i)) return;

        var interD1 = interD + 1;

        //points
        ref var p1 = ref currentXs[i];
        ref var p2 = ref currentXs[i + interD1];

        var dir = p2 - p1;
        var cDir =
            .5f * (dir - dir.normalized * restDistance * interD1);

        p1 += cDir;
        p2 -= cDir;
    }

    //uses two points
    void CollisionConstraint(int i)
    {
        //requirements
        if (RequirePoints(2, i)) return;

        //points
        ref var p1 = ref currentXs[i];
        ref var p2 = ref currentXs[i + 1];

        ref var p1_ = ref previousXs[i];
        ref var p2_ = ref previousXs[i + 1];

        //discrete checking and serial projection
        var cs = Physics.OverlapCapsule(p1, p2, radius);
        for (int j = 0; j < cs.Length; j++)
        {
            ref var c = ref cs[j];

            //Debug.Log(c.gameObject);
            var c1 = c.ClosestPoint(p1);
            var c2 = c.ClosestPoint(p2);

            var n1 = p1 - c1;
            var n2 = p2 - c2;

            var radius_ = radius + Physics.defaultContactOffset;

            p1 += n1 + n1.normalized * radius_;
            p2 += n2 + n2.normalized * radius_;

            //to prevent bouncing
            p1_ += n1.normalized * radius_;
            p2_ += n2.normalized * radius_;
        }

    }

    //void ParticleCollisionConstraint(int i)
    //{
    //    //requires only one point, no check necessary

    //    //continuous collision detection for particles
    //    ref var p1 = ref currentXs[i];
    //    ref var p1_ = ref previousXs[i];

    //    RaycastHit hit;

    //    //also keep cable radius in mind
    //    var p1_p1 = p1 - p1_;

    //    if (Physics.CapsuleCast(p1_ , p1_p1r, out hit, p1_p1r.magnitude))
    //    {
    //        var proj = hit.normal * (hit.distance);
    //        p1 += proj;
    //        //p1_ += hit.normal * (radius + Physics.defaultContactOffset);
    //    }
    //}

    void ParticleCollisionConstraint(int i)
    {
        ref var p = ref currentXs[i];
        ref var p_ = ref previousXs[i];

        var cs = Physics.OverlapSphere(p, radius);
        for (int j = 0; j < cs.Length; j++)
        {
            ref var c = ref cs[j];

            var cp = c.ClosestPoint(p);

            var correction = cp - p;

            p += correction - correction.normalized * radius;

            Friction(i, correction);
        }
    }

    void Friction(int i, Vector3 correction)
    {
        //points
        ref var p = ref currentXs[i];
        ref var p_ = ref previousXs[i];

        //sliding friction
        var v = p - p_;
        p_ += slidingFriction * correction.magnitude * v;

        //static friction
        if (v.magnitude < staticFriction)
        {
            p_ = p;
        }
    }

    bool RequirePoints(int n, int i)
    {
        if (i + n <= numberOfParticles)
        {
            return false;
        }

        return true;
    }

    void Simulate()
    {
        SetForces();
        Integrate();
        for (int i = 0; i < solverIterations; ++i)
        {
            SatisfyConstraints();
        }   
    }

    void CreateDebugPoints()
    {
        debugPoints = new Transform[numberOfParticles];

        for (int i = 0; i < numberOfParticles; ++i)
        {
            //Create Game Object with Capsule Collider
            var o = GameObject.CreatePrimitive(PrimitiveType.Cube);
            o.transform.localScale = 2f * new Vector3(
                radius, radius, radius
            );

            o.GetComponent<BoxCollider>().enabled = false;

            debugPoints[i] = o.transform;

            o.transform.parent = transform;
        }
    }
}
