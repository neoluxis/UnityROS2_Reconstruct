using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MainCamera : MonoBehaviour
{
    public Camera main_cam;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        CameraMove();
    }

    void CameraMove()
    {
        int times = 4;
        if (Input.GetKey(KeyCode.UpArrow))
        {
            main_cam.transform.Translate(Vector3.forward * Time.deltaTime * times);
        }

        if (Input.GetKey(KeyCode.DownArrow))
        {
            main_cam.transform.Translate(Vector3.back * Time.deltaTime * times);
        }

        if (Input.GetKey(KeyCode.LeftArrow))
        {
            main_cam.transform.Translate(Vector3.left * Time.deltaTime * times);
        }

        if (Input.GetKey(KeyCode.RightArrow))
        {
            main_cam.transform.Translate(Vector3.right * Time.deltaTime * times);
        }
    }
}