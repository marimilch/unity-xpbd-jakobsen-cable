using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CableColliderCreator : MonoBehaviour
{
    [Range(1, 100)]
    [Tooltip("If set to 0, the resolution of the rendering will be used (not recommended)")]
    [SerializeField] private int resolution = 4;

    [SerializeField] private float cableRadius = .1f;

    Vector3[] jointPoints;
    Transform[] capParents;
    int numberOfBones = 0;
    CableSpline line;

    void Awake()
    {
        line = GetComponent<CableSpline>();

        //Create Capsule Collider for each line
        numberOfBones = line.GetDynamicResolution(resolution);
        capParents = new Transform[numberOfBones];

        //Initialize all child object colliders
        for (int i = 0; i < numberOfBones; ++i)
        {
            capParents[i] = CreateTransformWithCap();
        }
    }

    Transform CreateTransformWithCap()
    {
        //Create Game Object with Capsule Collider
        var o = new GameObject();
        o.name = "Cable Bone";
        var cc = o.AddComponent<CapsuleCollider>();
        var t = o.transform;

        //Set facing direction to z-axis (forward)
        cc.direction = 2; //2 is the z-axis
        cc.radius = cableRadius; //set radius

        //Set this Game Object as Parent
        t.parent = transform;

        return t;
    }

    void SetCapsuleFromTo(ref Transform capParent, Vector3 start, Vector3 end)
    {
        //Set mid of capsule to mid of in-between vector
        var delta = end - start;
        capParent.localPosition = start + (delta / 2);

        capParent.LookAt( transform.TransformPoint(end) );
        var cap = capParent.GetComponent<CapsuleCollider>();
        cap.height = delta.magnitude + 2f * cableRadius;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        //Get aliased points
        jointPoints = line.GetRenderedPoints(resolution);

        //Update capsule positions
        for (int i = 0; i < numberOfBones; ++i)
        {
            SetCapsuleFromTo(
                ref capParents[i],
                jointPoints[i],
                jointPoints[i+1]
            );
        }
    }
}
