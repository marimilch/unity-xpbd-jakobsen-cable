using UnityEngine;
using System.Collections;

public class PauseAfter : MonoBehaviour
{
    [SerializeField] float waitingTime = 1f;

    // Use this for initialization
    void Start()
    {
        gameObject.GetActiveInterface<Cable>().SetGrab(0, Vector3.zero, false);
        StartCoroutine("WaitThenPause");
    }

    IEnumerator WaitThenPause(){
        yield return new WaitForSeconds(waitingTime);
        Debug.Break();
    }
}
