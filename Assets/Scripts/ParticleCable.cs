using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ParticleCable : MonoBehaviour
{
    [Range(1, 100)]
    [Tooltip("Will determine how many Joints there will be")]
    [SerializeField] private int resolution = 4;

    [SerializeField] private float cableRadius = .1f;

    [Range(1, 1000)]
    [Tooltip("Will determine how strong the spring effect of the cable is")]
    [SerializeField] private float springConstant = .1f;

    [Range(0, 100)]
    [Tooltip("Will determine how strong the dampening effect of the cable is")]
    [SerializeField] private float dampConstant = .1f;

    [SerializeField] private float kgPerMeter = .25f;


    float restDistance = 0f;
    float weightPerJoint = 0f;
    int maxReach = 2;
    ForceMode forceMode = ForceMode.Force;
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
        weightPerJoint = cableLength * kgPerMeter / (float) resolution;

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

    Vector3 CalcSpringForce(Vector3 p, Vector3 q, float rdf)
    {
        var d = q - p;
        return springConstant * (
            d.magnitude - rdf * restDistance
        ) * d.normalized;
    }

    Vector3 CalcDampForce(Vector3 pVel, Vector3 qVel)
    {
        return dampConstant * (qVel - pVel);
    }

    void UpdateCurrentForces(int distanceFactor = 1)
    {
        var e = distanceFactor - 1;

        for (int i = e; i < numberOfJoints - e; i++)
        {
            var pT = capParents[i];
            var qT = capParents[i + distanceFactor];

            var sf = CalcSpringForce(pT.position, qT.position, distanceFactor);
            var df = CalcDampForce(
                rigidbodies[i].velocity,
                rigidbodies[i + distanceFactor].velocity
            );

            currentForcesBetween[i] = sf + df;
        }
    }

    void ApplyCurrentForces(int reach = 1)
    {
        //end particles (between forces are in the mid of array)
        for (int i = 0; i < reach; i++)
        {
            var bi = i + reach - 1;
            rigidbodies[i].AddForce(
                currentForcesBetween[bi],
                forceMode
            );
            rigidbodies[numberOfParticles - 1 - i].AddForce(
                -currentForcesBetween[numberOfJoints - reach - bi],
                forceMode
            );
        }

        for (int i = reach; i < numberOfParticles - reach; i++)
        {
            rigidbodies[i].AddForce(
                currentForcesBetween[i] - currentForcesBetween[i - reach],
                forceMode
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
        for (int i = 0; i < maxReach; i++)
        {
            UpdateCurrentForces( i + 1 );
            //Debug.Log(currentForcesBetween[numberOfJoints - 1]);
            //Debug.Log(currentForcesBetween[numberOfJoints - 2]);
            ApplyCurrentForces( i + 1 );
        }

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
