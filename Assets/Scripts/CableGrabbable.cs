using UnityEngine;
using System.Collections;

public class CableGrabbable : MonoBehaviour
{
    int resolution;
    float diameter;

    [Tooltip("When enabled, the clickable joints will have a material.")]
    [SerializeField] bool debugMode = false;

    Transform[] clickParents;
    Vector3[] currentPoints;
    int numberOfBones = 0;
    VerletCable verletCable;

    //[SerializeField] MonoBehaviour grabHandler;

    // Use this for initialization
    void Start()
    {
        verletCable = GetComponent<VerletCable>();
        if (!verletCable)
        {
            throw new UnityException("Verlet Cable missing.");
        }

        //if (!(grabHandler is IGrabHandler))
        //{
        //    throw new UnityException("Grab Handler doess not implement IGrabHandler.");
        //}

        //Create Capsule Collider for each line
        numberOfBones = verletCable.GetNumberOfParticles() - 1;
        clickParents = new Transform[numberOfBones];
        diameter = verletCable.radius * 2f;

        //Initialize all child object colliders
        for (int i = 0; i < numberOfBones; ++i)
        {
            clickParents[i] = CreateClickable(i);
        }
    }

    // Update is called once per frame
    void Update()
    {
        currentPoints = verletCable.currentXs;
        for (int i = 0; i < numberOfBones; ++i)
        {
            SetClickableFromTo(
                ref clickParents[i],
                currentPoints[i],
                currentPoints[i + 1]
            );
        }
    }

    Transform CreateClickable(int i)
    {
        //Create Game Object with Capsule Collider
        var o = GameObject.CreatePrimitive(PrimitiveType.Cube);
        //Destroy( o.GetComponent<BoxCollider>() );
        if (!debugMode)
        {
            Destroy(o.GetComponent<MeshRenderer>());
        }

        o.name = "Cable Grab Handle " + i;

        var gh = o.AddComponent<DefaultGrabHandler>();
        gh.verletCable = verletCable;
        gh.jointNumber = i;

        var t = o.transform;

        //Set this Game Object as Parent
        t.parent = transform;

        return t;
    }

    void SetClickableFromTo(ref Transform capParent, Vector3 start, Vector3 end)
    {
        //Set mid of capsule to mid of in-between vector
        var delta = end - start;
        var delta2 = delta / 2f;
        capParent.position = start + delta2;

        capParent.LookAt(end);
        capParent.localScale = new Vector3(
            diameter,
            diameter,
            delta.magnitude
        );

        //cap.height = delta.magnitude + 2f * cableRadius;
    }
}
