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

    [Range(0f, 2f)]
    [Tooltip("Pseudo sliding friction")]
    [SerializeField] float slidingFriction = .01f;

    [Range(.01f, .1f)]
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

    //to prevent fallig through in collisions
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
            //JointCollisionConstraintConSim(i);

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
    void JointCollisionConstraintConSim(int i)
    {
        //requirements
        if (RequirePoints(2, i)) return;

        //requires only one point, no check necessary

        //continuous collision detection for particles
        ref var p1 = ref currentXs[i];
        ref var p1_ = ref previousXs[i];
        ref var p2 = ref currentXs[i+1];
        ref var p2_ = ref previousXs[i+1];

        var p1_p1 = p1 - p1_;
        var p2_p2 = p1 - p1_;
        var midVec = (p1_p1 + p2_p2) / 2f;
        var between = (p2_ - p1_).magnitude;

        var hits = Physics.CapsuleCastAll(p1_, p2_, radius, midVec, midVec.magnitude);

        for (int j = 0; j < hits.Length; ++j)
        {
            ref var hit = ref hits[j];
            if (hit.distance == 0f) continue; //according to Unity Docs
            //var correction = p1_p1 - (hit.normal * (hit.distance));
            var rp = hit.normal * (radius + distanceCorrection);
            var lhd = ((rp - p1_).magnitude) / between;
            var rhd = 1f - lhd;

            p1 = hit.point + (lhd * rp);
            p2 = hit.point + (rhd * rp);

            //Friction(i, rp);
            //Debug.Log(correction);
            //p1_ += hit.normal * (radius + Physics.defaultContactOffset);
        }

    }

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
            //var rp = hit.normal * (hit.distance);
            //p1 = rp + p1_;
            var tangent = Vector3.ProjectOnPlane(p1 - hit.point, hit.normal);
            p1 = hit.point + hit.normal * (radius + distanceCorrection) + tangent;

            Friction(
                i,
                hit,
                tangent
            );



            //Slopes(i, hit);
            //Debug.Log(correction);
            //p1_ += hit.normal * (radius + Physics.defaultContactOffset);
        }
    }

    void Friction(int i, RaycastHit hit, Vector3 tangent)
    {
        //points
        ref var p = ref currentXs[i];
        ref var p_ = ref previousXs[i];

        //penetration depth
        var d = (p - (hit.point + tangent)).magnitude;

        //velocity
        var v = p - p_;

        //prevent negative direction with static friction
        if (v.magnitude < staticFriction)
        {
            p_ = p;
            return;
        }

        //decrease velocity "from behind"
        p_ += slidingFriction * d * v / solverIterations;

        //sliding friction
        //var cm = hit.distance;
        //p_ += ((cm * slidingFriction) / (cm + 1f)) * v;
    }

    void Slopes(int i, RaycastHit hit)
    {
        //points
        ref var p = ref currentXs[i];
        ref var p_ = ref previousXs[i];

        //slope
        var s = Vector3.ProjectOnPlane(hit.normal, p - p_);
        p += s;

        //static friction
        //if (v.magnitude < staticFriction)
        //{
        //    p_ = p;
        //}

        //sliding friction
        //var cm = hit.distance;
        //p_ += ((cm * slidingFriction) / (cm + 1f)) * v;
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
