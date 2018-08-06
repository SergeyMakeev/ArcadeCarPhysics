using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovingBlock : MonoBehaviour {

    public float amplitude = 0.3f;
    public float speed = 0.5f;
    public float initialTime = 0.0f;


    Vector3 originalPos;
    float time = 0.0f;


    // Use this for initialization
    void Start () {

        originalPos = transform.position;
        time = initialTime;
    }

    // Update is called once per frame
    void Update () {

        float y = originalPos.y + (Mathf.Cos(time) - 1.0f) * (amplitude * 0.5f);
        transform.position = new Vector3(originalPos.x, y, originalPos.z);

        time += Time.deltaTime * speed;


    }
}
