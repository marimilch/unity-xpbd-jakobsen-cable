using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VerletCable : MonoBehaviour
{
    [Range(2, 100)]
    [Tooltip("Will determine how many Joints there will be. " +
        "The more joints, the softer the cable will look.")]
    [SerializeField] private int resolution = 4;
    int numberOfParticles;

    [Tooltip("The radius of the cable")]
    [SerializeField] float radius = .25f;

    [Tooltip("Increasing this will slow down (dampen) any movement.")]
    [Range(0f, 0.05f)]
    [SerializeField] float dampening = .01f;

    [Range(0f, .1f)]
    [Tooltip("Pseudo sliding friction")]
    [SerializeField] float slidingFriction = .05f;

    [Range(0f, .1f)]
    [Tooltip("Pseudo static friction")]
    [SerializeField] float staticFriction = .01f;

    [Range(0, 25)]
    [Tooltip("The more iterations, the preciser " +
        "the result. Increase this, if the cable falls through " +
        "objects (or use a different collision check than discrete. " +
        "The cable will appear " +
        "stiffer with an increasing value " +
        "(0 for Unity global configuration)")]
    [SerializeField] int solverIterations = 0;

    [Tooltip("Whether to show particle positions as cubes")]
    [SerializeField] bool debugMode = true;

    float dt = 0f;
    float dt_squared = 0f;
    float restDistance = 0f;

    float distanceCorrection = .001f;

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

    void PreventVibration()
    {
        for (int i = 0; i < numberOfParticles; ++i)
        {
            ref var x = ref currentXs[i];
            ref var x_ = ref previousXs[i];
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
            ParticleCollisionConstraintConSim(i);
            //ParticleCollisionConstraintContinuousSerial(i);
            //ParticleCollisionConstraintContinuousSimultaneous(i);
            //JointCollisionConstraint(i);

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
    void JointCollisionConstraint(int i)
    {
        //requirements
        if (RequirePoints(2, i)) return;

        //points
        ref var p1 = ref currentXs[i];
        ref var p2 = ref currentXs[i + 1];

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

            p1 += n1 - n1.normalized * radius;
            p2 += n2 - n2.normalized * radius;
        }

    }

    void ParticleCollisionConstraintContinuousSerial(int i)
    {
        //requires only one point, no check necessary

        //continuous collision detection for particles
        ref var p1 = ref currentXs[i];
        ref var p1_ = ref previousXs[i];

        RaycastHit hit;

        //also keep cable radius in mind
        var p1_p1 = p1 - p1_;

        if (Physics.SphereCast(p1_, radius, p1_p1, out hit, p1_p1.magnitude))
        {
            var correction = p1_p1 - (hit.normal * (hit.distance));
            p1 -= correction;

            Friction(i, correction);
            //Debug.Log(correction);
            //p1_ += hit.normal * (radius + Physics.defaultContactOffset);
        }
    }

    void ParticleCollisionConstraintContinuousSimultaneous(int i)
    {
        //requires only one point, no check necessary

        //continuous collision detection for particles
        ref var p1 = ref currentXs[i];
        ref var p1_ = ref previousXs[i];

        var cs = Physics.OverlapCapsule(p1_, p1, radius);

        //also keep cable radius in mind
        var p1_p1 = p1 - p1_;

        for (int j = 0; j < cs.Length; j++)
        {
            ref var c = ref cs[j];

            var cp = c.ClosestPoint(p1);

            var correction = cp - p1;

            p1 += correction - correction.normalized * radius;

            Friction(i, correction);
        }
    }

    //void ParticleCollisionConstraint(int i)
    //{
    //    ref var p = ref currentXs[i];
    //    ref var p_ = ref previousXs[i];

    //    var cs = Physics.OverlapSphere(p, radius);
    //    for (int j = 0; j < cs.Length; j++)
    //    {
    //        ref var c = ref cs[j];

    //        var cpOuter = c.ClosestPoint(p_);
    //        //var cpInner = c.ClosestPoint(p);

    //        var correctionSurface = cpOuter - p;
    //        //var correctionRadius = (cpInner - p).normalized * radius;

    //        //var radDir = Mathf.Sign(Vector3.Dot(correction, p - p_));
    //        //var radDir = 1f;

    //        p = p + correctionSurface;

    //        Debug.Log(correctionSurface);
    //        //Friction(i, correction);
    //    }
    //}

    void ParticleCollisionConstraintConSim(int i)
    {
        //requires only one point, no check necessary

        //continuous collision detection for particles
        ref var p1 = ref currentXs[i];
        ref var p1_ = ref previousXs[i];

        var p1_p1 = p1 - p1_;

        var hits = Physics.SphereCastAll(p1_, radius, p1_p1, p1_p1.magnitude);

        for (int j = 0; j < hits.Length; ++j)
        {
            ref var hit = ref hits[j];
            if (hit.distance == 0f) continue; //according to Unity Docs
            //var correction = p1_p1 - (hit.normal * (hit.distance));
            //var rp = hit.normal * (hit.distance + radius);
            p1 = hit.point + hit.normal * (radius + distanceCorrection);

            //Friction(i, rp);
            //Debug.Log(correction);
            //p1_ += hit.normal * (radius + Physics.defaultContactOffset);
        }
    }

    void Friction(int i, Vector3 correction)
    {
        //points
        ref var p = ref currentXs[i];
        ref var p_ = ref previousXs[i];

        //velocity
        var v = p - p_;

        //static friction
        if (v.magnitude < staticFriction)
        {
            p_ = p;
        }

        //sliding friction
        var cm = correction.magnitude;
        p_ += ((cm * slidingFriction) / (cm + 1f)) * v;
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
        //PreventVibration();
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
