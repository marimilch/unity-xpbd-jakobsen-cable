using UnityEngine;
using System.Collections;

public class XPBDCable : MonoBehaviour
{
    [Range(1, 100)]
    [Tooltip("Will determine how many Joints there will be")]
    [SerializeField] private int resolution = 4;

    [SerializeField] private float cableRadius = .1f;

    [SerializeField] private float kgPerMeter = .25f;

    float restDistance = 0f;
    float weightPerJoint = 0f;
    int maxReach = 2;

    Transform[] capParents;
    Rigidbody[] rigidbodies;

    int numberOfParticles = 0;
    int numberOfJoints = 0;

    CableInitialiser line;

    // Use this for initialization
    void Start()
    {
        var xpbd = gameObject.AddComponent<XPBD>();

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

        restDistance = cableLength / (float)numberOfJoints;
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

        var constraints = CreateConstraints();

        xpbd.Attach(constraints, rigidbodies, 3);
    }

    Constraint[] CreateConstraints()
    {
        var cs = new Constraint[1];

        cs[0] = new Constraint((vs) =>
        {
            var d = Mathf.Abs((vs[1] - vs[0]).magnitude) - restDistance;
            //Debug.Log("Distance delta: " + d);
            return d;
        }, true, 2, 0.00000001f);

        //cs[1] = new Constraint((vs) =>
        //{
        //    var d = Mathf.Abs((vs[2] - vs[0]).magnitude) - restDistance*2;
        //    //Debug.Log("Distance delta: " + d);
        //    return d;
        //}, true, 3, 0.00000001f);

        return cs;
    }

    Transform CreateTransformWithCap()
    {
        //Create Game Object with Capsule Collider
        var o = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        o.transform.localScale = 2f * new Vector3(
            cableRadius, cableRadius, cableRadius
        );

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
        rb.useGravity = false;

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
    void Update()
    {

    }
}
