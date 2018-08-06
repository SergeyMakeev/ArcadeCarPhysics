using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraCar : MonoBehaviour
{
    public GameObject target;
    public float distance = 10.0f;

    public float targetHeightOffset = 0.0f;
    public float cameraHeightOffset = 0.0f;

    Camera cameraComponent = null;
    ArcadeCar carComponent = null;

    public float yaw = 0.0f;


    Vector3 curPos;


    //x - speed in km/h
    //y - fov in degrees
    [Tooltip("Y - Fov (degrees). X - Vehicle speed (km/h)")]
    public AnimationCurve fovCurve = AnimationCurve.Linear(0.0f, 60.0f, 120.0f, 40.0f);

    void Start()
    {
        cameraComponent = GetComponent<Camera>();
        if (target != null)
        {
            carComponent = target.GetComponent<ArcadeCar>();
        }

        curPos = transform.position;
    }

    void Update()
    {
        if (Input.GetKey(KeyCode.Alpha1) || Input.GetKey(KeyCode.F1))
        {
            yaw = 60.0f;
        }

        if (Input.GetKey(KeyCode.Alpha2) || Input.GetKey(KeyCode.F2))
        {
            yaw = -60.0f;
        }

        if (Input.GetKey(KeyCode.Alpha3) || Input.GetKey(KeyCode.F3))
        {
            yaw = 0.0f;
        }

        if (Input.GetKey(KeyCode.Alpha4) || Input.GetKey(KeyCode.F4))
        {
            yaw = 180.0f;
        }


        //float limitDegrees = steerAngleLimit.Evaluate(speedKmH);

        Vector3 curPosTmp = curPos;
        Vector3 tgtPos = target.transform.position;

        tgtPos.y = 0.0f;
        curPosTmp.y = 0.0f;

        Vector3 dir2D = curPosTmp - tgtPos;

        float len = dir2D.magnitude;
        dir2D.Normalize();

        Vector3 camPos = curPosTmp;
        if (len > distance)
        {
            camPos = tgtPos + dir2D * distance;
        }

        camPos.y = target.transform.position.y + cameraHeightOffset;
        transform.position = camPos;

        Vector3 targetPt = target.transform.position;
        targetPt.y += targetHeightOffset;

        Vector3 lookDir = targetPt - camPos;

        Quaternion rot = Quaternion.LookRotation(lookDir, Vector3.up);

        transform.rotation = rot;

        // apply fov
        if (carComponent != null)
        {
            float speed = carComponent.GetSpeed();
            float speedKmH = speed * 3.6f;

            float fov = fovCurve.Evaluate(speedKmH);

            //Debug.Log(string.Format("speed {0}, speed km/h {1}, fov {2}", speed, speedKmH, fov));

            cameraComponent.fieldOfView = fov;
        }

        curPos = transform.position;

        transform.RotateAround(targetPt, Vector3.up, yaw);

    }
}
