using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StiffCable : MonoBehaviour
{
    [Range(1, 100)]
    [Tooltip("Will determine how many Joints there will be")]
    [SerializeField] private int resolution = 4;

    [SerializeField] private float cableRadius = .1f;

    [Tooltip("Maximum angle per metere in degrees")]
    [SerializeField] private float maxAnglePerMeter = .25f;
    [SerializeField] private float kgPerMeter = .25f;

    float restDistance = 0f;
    float weightPerJoint = 0f;
    Vector3[] currentForcesBetween;

    //Vector3[] jointPoints;
    Transform[] capParents;
    Rigidbody[] rigidbodies;

    int numberOfParticles = 0;
    int numberOfJoints = 0;

    CableInitialiser line;

    void Start()
    {
        line = GetComponent<CableInitialiser>();
        var jointPoints = SplineTools.AliasFunction(
            CatmullSpline.CreateCatmullSpline(line.controlPoints),
            resolution
        );
        var cableLength = SplineTools.GetFunLength(jointPoints);

        //Create Capsule Collider for each line
        numberOfParticles = jointPoints.Length;
        numberOfJoints = numberOfParticles - 1;
        capParents = new Transform[numberOfParticles];
        rigidbodies = new Rigidbody[numberOfParticles];

        restDistance = cableLength / (float) numberOfJoints;
        weightPerJoint = cableLength * kgPerMeter / (float)resolution;

        //Initialize all child object colliders
        for (int i = 0; i < numberOfParticles; ++i)
        {
            var newT = CreateTransformWithCap();
            capParents[i] = newT;
            SetParticleCapsule(
                ref newT,
                jointPoints[i]
            );

            rigidbodies[i] = newT.GetComponent<Rigidbody>();
        }

        //Initialize length of currentForcesBetween variable
        currentForcesBetween = new Vector3[numberOfJoints];

    }

    //Vector3 CalcSpringForce(Vector3 p, Vector3 q, float rdf)
    //{
    //    var d = q - p;
    //    return springConstant * (
    //        d.magnitude - rdf * restDistance
    //    ) * d.normalized;
    //}

    //Vector3 CalcDampForce(Vector3 pVel, Vector3 qVel)
    //{
    //    return dampConstant * (qVel - pVel);
    //}

    Vector3 CalcDistance(Vector3 p, Vector3 q)
    {
        var dir = q - p;
        return .5f * ( dir - dir.normalized * restDistance );
    }

    void UpdateCurrentForces(int distanceFactor = 1)
    {
        var e = distanceFactor - 1;

        for (int i = e; i < numberOfJoints - e; i++)
        {
            var pT = capParents[i];
            var qT = capParents[i + distanceFactor];

            currentForcesBetween[i] = CalcDistance(
                pT.position,
                qT.position
            );
        }
    }

    void ApplyCurrentForces(int reach = 1)
    {
        //end particles (between forces are in the mid of array)
        for (int i = 0; i < reach; i++)
        {
            var bi = i + reach - 1;
            rigidbodies[i].MovePosition(
                capParents[i].position + currentForcesBetween[bi]
            );
            var lastIndex = numberOfParticles - 1 - i;
            rigidbodies[lastIndex].MovePosition(
                capParents[lastIndex].position - currentForcesBetween[numberOfJoints - 1 - bi]
            );
        }

        for (int i = reach; i < numberOfParticles - reach; i++)
        {
            rigidbodies[i].MovePosition(
                capParents[i].position + 
                currentForcesBetween[i] - currentForcesBetween[i - reach]
            );
        }
    }

    Transform CreateTransformWithCap()
    {
        //Create Game Object with Capsule Collider
        var o = new GameObject();
        o.name = "Particle";
        var cc = o.AddComponent<CapsuleCollider>();
        var t = o.transform;

        //Add RigidBody with Collider
        var rb = o.AddComponent<Rigidbody>();
        rb.constraints =
            RigidbodyConstraints.FreezeRotationX |
            RigidbodyConstraints.FreezeRotationY |
            RigidbodyConstraints.FreezeRotationZ
        ;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        rb.mass = weightPerJoint;

        //Set facing direction to z-axis (forward)
        cc.direction = 2; //2 is the z-axis
        cc.radius = cableRadius; //set radius
        cc.height = 0;

        //Set this Game Object as Parent
        t.parent = transform;

        return t;
    }

    void SetParticleCapsule(ref Transform capParent, Vector3 pos)
    {
        var cap = capParent.GetComponent<CapsuleCollider>();
        capParent.localPosition = pos;
        cap.radius = cableRadius;
        cap.height = 0;
    }

    // Update is called once per frame
    void FixedUpdate()
    {

        UpdateCurrentForces(1);
        ApplyCurrentForces(1);

        //Debug.Log("------------------");

        //SetParticleCapsule

        //Update capsule positions
        //for (int i = 0; i < numberOfParticles; ++i)
        //{
        //    SetParticleCapsule(
        //        ref capParents[i],
        //        jointPoints[i]
        //    );
        //}
    }
}
