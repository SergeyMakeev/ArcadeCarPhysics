using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraTrigger : MonoBehaviour {

    GameObject cameraObject;
    public float showTime = 1.0f;
    float timeToDisable = 0.0f;
    bool shouldDisableCamera = false;

    void Start () {

        cameraObject = transform.Find("Camera").gameObject;
        cameraObject.SetActive(false);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject)
        {
            if (other.gameObject.GetComponent<ArcadeCar>() == null)
            {
                return;
            }
        }

        cameraObject.SetActive(true);
        timeToDisable = 0.0f;
        shouldDisableCamera = false;
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject)
        {
            if (other.gameObject.GetComponent<ArcadeCar>() == null)
            {
                return;
            }
        }

        timeToDisable = showTime;
        shouldDisableCamera = true;
    }

    void Update () {
        if (timeToDisable > 0.0f)
        {
            timeToDisable -= Time.deltaTime;
        }
        else
        {
            timeToDisable = 0.0f;
            if (shouldDisableCamera)
            {
                cameraObject.SetActive(false);
                shouldDisableCamera = false;
            }

        }


    }
}
