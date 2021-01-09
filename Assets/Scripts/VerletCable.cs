using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VerletCable : MonoBehaviour
{
    [Tooltip("Will determine how many Joints there will be. " +
        "The more joints, the softer the cable will look.")]
    [SerializeField] private int resolution = 10;
    int numberOfParticles;

    [Tooltip("The radius of the cable")]
    public float radius = .25f;

    [Tooltip("Increasing this will slow down (dampen) any movement.")]
    [Range(0f, 0.05f)]
    [SerializeField] float dampening = 0f;

    [Range(0f, 2f)]
    [Tooltip("Pseudo sliding friction")]
    [SerializeField] float slidingFriction = .5f;

    [Range(.01f, .1f)]
    [Tooltip("Pseudo static friction")]
    [SerializeField] float staticFriction = .01f;

    [Range(0, 1f)]
    [SerializeField] float stiffness = .1f;
    int stiffnessFactor;

    Vector3 grav = Physics.gravity;

    [Range(0, 25)]
    [Tooltip("The more iterations, the preciser " +
        "the result. Increase this, if the cable falls through " +
        "objects (though unlikely). The cable will appear " +
        "stiffer with an increasing value " +
        "(0 for Unity global configuration)")]
    [SerializeField] int solverIterations = 0;

    [Tooltip("The layer to ignore for collision check.")]
    [SerializeField] string ignoreLayer = "CableClickColliders";

    [Tooltip("Whether to show particle positions as cubes")]
    [SerializeField] bool debugMode = false;

    float dt = 0f;
    float dt_squared = 0f;
    float restDistance = 0f;

    //dont collide with grabber colliders by ignoring player layer
    int layerMask;

    //to prevent fallig through in collisions
    float distanceCorrection = .001f;

    //for grabbing
    Vector3[] constrainedPositions;
    bool[] hasConstrainedPosition;

    [HideInInspector]
    public Vector3[] currentXs;
    Vector3[] previousXs;
    
    //Vector3[] accelerations;

    Transform[] debugPoints;

    public int GetNumberOfParticles(){
        return resolution + 1;
    }

    void Start()
    {
        var line = GetComponent<CableInitialiser>();
        numberOfParticles = GetNumberOfParticles();

        //all except player
        layerMask = ~LayerMask.GetMask(ignoreLayer);

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
        constrainedPositions = new Vector3[numberOfParticles];
        hasConstrainedPosition = new bool[numberOfParticles];
        //accelerations = new Vector3[numberOfParticles];

        currentXs.CopyTo(previousXs, 0);

        if (debugMode)
        {
            CreateDebugPoints();
        }

        dt = Time.fixedDeltaTime;
        dt_squared = dt * dt;
    }

    public void SetGrab(int i, Vector3 to)
    {
        hasConstrainedPosition[i] = true;
        constrainedPositions[i] = to;
    }

    public void EndGrab(int i)
    {
        hasConstrainedPosition[i] = false;
    }

    public bool IsGrabbed(int i)
    {
        return hasConstrainedPosition[i];
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
            //ref var a = ref accelerations[i];
            //var a = Physics.gravity;

            var temp = x;

            //no gravity if position is constrained
            var deltaPosGrav = !IsGrabbed(i) ? grav * dt_squared : Vector3.zero;

            x =
                x * (2 - dampening) - (1 - dampening) * x_ +
                deltaPosGrav
            ;

            x_ = temp;
            //x_ = (x - temp).magnitude > Physics.defaultContactOffset ? temp : x;
        }
    }

    void SetForces()
    {
        //for (int i = 0; i < numberOfParticles; ++i)
        //{
        //    accelerations[i] = Physics.gravity;
        //}
    }

    void SatisfyConstraints()
    {
        for (int i = 0; i < numberOfParticles; ++i)
        {
            ref var p = ref currentXs[i];

            if (IsGrabbed(i))
            {
                PositionContraint(i);
            }

            DistanceConstraint(i);
            DistanceConstraint(i, 1);

            //cable stiffness
            //of course we could precompute it, but that way,
            //it can be changed live
            stiffnessFactor = (int) (stiffness * resolution);
            //Debug.Log(stiffnessFactor);
            if (stiffnessFactor >= 2)
            {
                DistanceConstraint(i, stiffnessFactor);
            }

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

    //void AngleConstraint(int i)
    //{
    //    //requirements
    //    if (RequirePoints(3, i)) return;

    //    //points
    //    ref var p1 = ref currentXs[i];
    //    ref var p2 = ref currentXs[i + 1];
    //    ref var p3 = ref currentXs[i + 2];


    //}

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

        var p1p1m = (p2 - p1).magnitude;

        var p1_p1 = p1 - p1_;
        var p2_p2 = p2 - p2_;
        var midVec = (p1_p1 + p2_p2) / 2f;

        var hits = Physics.CapsuleCastAll(
            p1_ * (p1p1m / 2f),
            p2_ * (p1p1m / 2f),
            radius,
            midVec,
            midVec.magnitude
        );

        for (int j = 0; j < hits.Length; ++j)
        {
            ref var hit = ref hits[j];
            if (hit.distance == 0f) continue; //according to Unity Docs

            var hd1m = (p1 - hit.point).magnitude;
            var hd2m = (p2 - hit.point).magnitude;
            var sumMag = hd1m + hd2m;

            var t1 = Vector3.ProjectOnPlane(p1 - hit.point, hit.normal) *
                (hd1m / sumMag) ;
            var t2 = Vector3.ProjectOnPlane(p2 - hit.point, hit.normal) *
                (hd2m / sumMag) ;

            p1 = hit.point + hit.normal * (radius + distanceCorrection) +
                t1;
            p2 = hit.point + hit.normal * (radius + distanceCorrection) +
                t2;

            //Friction(i, hit, t1);
            //Friction(i + 1, hit, t2);
        }
    }

    void JointCollisionConstraintDisSim(int i)
    {
        //requirements
        if (RequirePoints(2, i)) return;

        //continuous collision detection for particles
        ref var p1 = ref currentXs[i];
        ref var p2 = ref currentXs[i + 1];

        var p1p2 = p2 - p1;

        var hits = Physics.SphereCastAll(p1, radius, p1p2, p1p2.magnitude);

        for (int j = 0; j < hits.Length; ++j)
        {
            ref var hit = ref hits[j];
            if (hit.distance == 0f) continue; //according to Unity Docs
            var tangent = Vector3.ProjectOnPlane(p1 - hit.point, hit.normal);
            p1 = hit.point + hit.normal * (radius + distanceCorrection) +
                tangent;

            var penetrationDepth = (p1 - (hit.point + tangent)).magnitude;

            Friction(i, penetrationDepth, tangent);
        }
    }

    void ParticleCollisionConstraintConSim(int i)
    {
        //requires only one point, no check necessary

        //continuous collision detection for particles
        ref var p1 = ref currentXs[i];
        ref var p1_ = ref previousXs[i];

        var p1_p1 = p1 - p1_;

        var hits = Physics.SphereCastAll(p1_, radius, p1_p1, p1_p1.magnitude, layerMask);

        for (int j = 0; j < hits.Length; ++j)
        {
            ref var hit = ref hits[j];
            if (hit.distance == 0f) continue; //according to Unity Docs
            var tangent = Vector3.ProjectOnPlane(p1 - hit.point, hit.normal);
            p1 = hit.point + hit.normal * (radius + distanceCorrection) +
                tangent;

            var penetrationDepth = (p1 - (hit.point + tangent)).magnitude;

            Friction(i, penetrationDepth, tangent);
        }
    }

    void PositionContraint(int i)
    {
        ref var p1 = ref currentXs[i];

        p1 = constrainedPositions[i];
    }

    void ParticleCollisionConstraintDisSim(int i)
    {
        //requires only one point, no check necessary

        //continuous collision detection for particles
        ref var p1 = ref currentXs[i];
        ref var p1_ = ref previousXs[i];

        var p1_p1 = p1 - p1_;

        //broad search
        var cols = Physics.OverlapSphere(p1, radius);

        for (int j = 0; j < cols.Length; ++j)
        {
            ref var col = ref cols[j];
            var ray = new Ray(p1_, p1_p1.normalized);
            RaycastHit hit;

            var bullseye = col.Raycast(ray, out hit, p1_p1.magnitude);

            if (!bullseye)
            {
                continue;
            }

            Debug.Log(i + ", " + j + ", " + hit.point);

            if (hit.distance == 0f) continue; //according to Unity Docs
            var tangent = Vector3.ProjectOnPlane(p1 - hit.point, hit.normal);
            //p1 = hit.point + hit.normal * (radius + distanceCorrection) +
            //    tangent;
            p1 = hit.point;

            //Friction(i, hit, tangent);
        }
    }

    void Friction(int i, float penetrationDepth, Vector3 tangent)
    {
        //points
        ref var p = ref currentXs[i];
        ref var p_ = ref previousXs[i];

        //penetration depth
        var d = penetrationDepth;

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
