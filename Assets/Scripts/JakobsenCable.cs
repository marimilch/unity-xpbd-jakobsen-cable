using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JakobsenCable : MonoBehaviour
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
    [SerializeField] float slidingFriction = .25f;

    [Range(.1f, .5f)]
    [Tooltip("Pseudo static friction")]
    [SerializeField] float staticFriction = .01f;

    [Tooltip("Prevents overly long cables due to high speeds. Set to 0 to disable.")]
    public float maxVelocity = 0f;

    [Tooltip("Enables continuous collision check on " +
        "grabbed particles to prevent tunneling when moved " +
        "through colliders.")]
    public bool preventTunnelingOnGrabbed = true;

    [Range(0, 1f)]
    public float stiffness = .1f;
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

    CapsuleCollider cc;
    
    //Vector3[] accelerations;

    Transform[] debugPoints;

    public int GetNumberOfParticles(){
        return resolution + 1;
    }

    void Start()
    {
        var line = GetComponent<CableInitialiser>();
        numberOfParticles = GetNumberOfParticles();

        AddCollisionHelper();

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

    void AddCollisionHelper()
    {
        var go = new GameObject();
        go.name = "Collision Helper";
        go.transform.SetParent(transform);
        cc = go.AddComponent<CapsuleCollider>();

        //Set facing direction to z-axis (forward)
        cc.direction = 2; //2 is the z-axis
        cc.radius = radius; //set radius
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
                PositionConstraint(i);
            }

            ParticleCollisionConstraintConSim(i);

            JointCollisionNative(i);

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

            //JointCollisionConstraintConSim(i);

            

            if (maxVelocity != 0)
            {
                VelocityConstraint(i);
            }
            //ParticleCollisionConstraintConSim(i);

            //project out
            //p = Vector3.Min(
            //    Vector3.Max(p, new Vector3(0, 0, 0)),
            //    new Vector3(100, 100, 100)
            //);
        }
    }

    void VelocityConstraint(int i)
    {
        ref var p1 = ref currentXs[i];
        ref var p1_ = ref previousXs[i];

        var p1_p1 = p1 - p1_;

        if (p1_p1.magnitude > maxVelocity)
        {
            p1_ = p1 - p1_p1.normalized * maxVelocity;
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

    void SetCapsuleFromTo(Vector3 start, Vector3 end)
    {
        var capParent = cc.transform;

        //Set mid of capsule to mid of in-between vector
        var delta = end - start;
        capParent.position = start + (delta / 2f);

        capParent.LookAt(end);
        cc.height = delta.magnitude + 2f * radius;
    }

    void JointCollisionNative(int i)
    {
        //requirements
        if (RequirePoints(2, i)) return;

        //discrete collision detection for particles
        ref var p1 = ref currentXs[i];
        ref var p1_ = ref previousXs[i];
        ref var p2 = ref currentXs[i + 1];
        ref var p2_ = ref previousXs[i + 1];

        //current velocities
        var p1_p1 = p1 - p1_;
        var p2_p2 = p2 - p2_;

        var cols = Physics.OverlapCapsule(p1, p2, radius, layerMask);
        SetCapsuleFromTo(p1, p2);

        for (int j = 0; j < cols.Length; ++j)
        {
            //Debug.Log("triggered.");
            var col = cols[j];

            if (col == cc)
                continue; // skip ourself

            Vector3 otherPosition = col.gameObject.transform.position;
            Quaternion otherRotation = col.gameObject.transform.rotation;

            Vector3 direction;
            float distance;

            bool overlapped = Physics.ComputePenetration(
                cc, cc.transform.position, cc.transform.rotation,
                col, otherPosition, otherRotation,
                out direction, out distance
            );

            var correctionVector = direction * distance;

            //var t1 = Vector3.Dot(direction, p1_p1) / distance * p1_p1;
            //var t2 = Vector3.Dot(direction, p2_p2) / distance * p1_p1;

            if (overlapped)
            {
                var p1c = p1 + correctionVector;
                var p2c = p2 + correctionVector;

                var t1 = Vector3.ProjectOnPlane(p1_p1, direction);
                var t2 = Vector3.ProjectOnPlane(p2_p2, direction);

                FrictionForNative(i, distance, t1);
                FrictionForNative(i + 1, distance, t2);

                p1 = p1c;
                p2 = p2c;
            }
        }
    }

    void FrictionForNative(int i, float distance, Vector3 t1)
    {
        ref var p1 = ref currentXs[i];
        ref var p1_ = ref previousXs[i];

        var p1_p1 = p1 - p1_;

        if (p1_p1.magnitude < staticFriction)
        {
            p1_ = p1;
            return;
        }

        p1_ += t1 * slidingFriction * distance;
    }

    //uses two points
    //void JointCollisionConstraintConSim(int i)
    //{
    //    //requirements
    //    if (RequirePoints(2, i)) return;

    //    //requires only one point, no check necessary

    //    //continuous collision detection for particles
    //    ref var p1 = ref currentXs[i];
    //    ref var p1_ = ref previousXs[i];
    //    ref var p2 = ref currentXs[i+1];
    //    ref var p2_ = ref previousXs[i+1];

    //    var p1p2 = p2 - p1;

    //    var p1_p1 = p1 - p1_;
    //    var p2_p2 = p2 - p2_;
    //    var midVec = p1 - p1_ + (p2 - p1 - p2_ + p1_) / 2f;

    //    var p1m = p1 - midVec;
    //    var p2m = p2 - midVec;

    //    var hits = Physics.CapsuleCastAll(
    //        p1m,
    //        p2m,
    //        radius,
    //        midVec,
    //        midVec.magnitude,
    //        layerMask
    //    );

    //    for (int j = 0; j < hits.Length; ++j)
    //    {
    //        ref var hit = ref hits[j];
    //        if (hit.distance == 0f) continue; //according to Unity Docs

    //        var t1 = Vector3.ProjectOnPlane(p1 - hit.point, hit.normal);
    //        p1 = hit.point + hit.normal * (radius + distanceCorrection) +
    //            t1;

    //        var pd1 = (p1 - (hit.point + t1)).magnitude;

    //        var t2 = Vector3.ProjectOnPlane(p2 - hit.point, hit.normal);
    //        p2 = hit.point + hit.normal * (radius + distanceCorrection) +
    //            t2;

    //        var pd2 = (p2 - (hit.point + t2)).magnitude;

    //        //Friction(i, penetrationDepth, tangent);

    //        //var projectDist = Vector3.Dot(hit.point - p1, p1p2) / p1p2.magnitude;
    //        //var p = p1 + p1p2.normalized * projectDist;
    //        //var pq = hit.point - p;

    //        //p1 += pq + pq.normalized * (distanceCorrection*2f + radius);
    //        //p2 += pq + pq.normalized * (distanceCorrection*2f + radius);

    //        //Debug.Log(pq);

    //        //Friction(i, hit, t1);
    //        //Friction(i + 1, hit, t2);
    //    }
    //}

    //void JointCollisionConstraintDisSim(int i)
    //{
    //    //requirements
    //    if (RequirePoints(2, i)) return;

    //    //continuous collision detection for particles
    //    ref var p1 = ref currentXs[i];
    //    ref var p1_ = ref previousXs[i];
    //    ref var p2 = ref currentXs[i + 1];
    //    ref var p2_ = ref previousXs[i + 1];

    //    var p1p2 = p2 - p1;

    //    var p1_p1 = p1 - p1_;
    //    var p2_p2 = p2 - p2_;
    //    var midVecDir = ((p1_p1 + p2_p2) / 2f).normalized;

    //    var hits = Physics.SphereCastAll(p1, radius, p1p2, p1p2.magnitude);

    //    for (int j = 0; j < hits.Length; ++j)
    //    {
    //        ref var hit = ref hits[j];
    //        if (hit.distance == 0f) continue; //according to Unity Docs
    //        var tangent = Vector3.ProjectOnPlane(p1 - hit.point, hit.normal);
    //        var penetrationDepth = (p1 - (hit.point + tangent)).magnitude;

    //        p1 = hit.point + hit.normal * (radius + distanceCorrection) +
    //            tangent;

    //        Friction(i, penetrationDepth, tangent);
    //    }
    //}

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

            //Friction(i, penetrationDepth, tangent);
        }

        //return hits.Length > 0;
    }

    void PositionConstraint(int i)
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
        p_ += slidingFriction * d * tangent / solverIterations;

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
