using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SuspensionTest : MonoBehaviour {

    public float amplitude = 0.3f;
    public float speed = 1.0f;

    Vector3 originalPos;
    float time = 0.0f;

    GameObject dynamic;
    GameObject cameraObject;

    void Start()
    {
        dynamic = transform.Find("Dynamic").gameObject;
        cameraObject = transform.Find("SpecCamera").gameObject;


        cameraObject.SetActive(false);
        originalPos = dynamic.transform.position;
        time = 0.0f;
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

        cameraObject.SetActive(false);
    }

    void Update()
    {

        float y = originalPos.y + (Mathf.Cos(time) - 1.0f) * (amplitude * 0.5f);
        dynamic.transform.position = new Vector3(originalPos.x, y, originalPos.z);

        time += Time.deltaTime * speed;
    }

}
