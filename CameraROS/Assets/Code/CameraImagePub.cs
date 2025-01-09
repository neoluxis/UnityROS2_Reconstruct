using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Camera;
using System.IO;
using RosMessageTypes.Sensor;


public class CameraImagePub : MonoBehaviour
{
    // ROS连接对象
    private ROSConnection _ros;

    // 图像发布主题
    public string topicName = "camera_image";

    // 相机对象
    public GameObject camera_obj;
    public Camera cam1;
    public Camera cam2;

    // 图像发布间隔
    public float pub_time = 1.0f; // s
    public int img_width = 640;
    public int img_height = 480;

    private float timeElapsed = 0.0f;

    private CompressedImageMsg image_msg;

    private RenderTexture rt1, rt2;
    private Texture2D t2d;

    int num = 0; //截图计数

    bool mouseEnable = false;

    // Start is called before the first frame update
    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<CompressedImageMsg>(topicName);

        t2d = new Texture2D(img_width * 2, img_height, TextureFormat.RGB24, false);
        
        rt1 = new RenderTexture(img_width, img_height, 24);
        rt2 = new RenderTexture(img_width, img_height, 24);
        cam1.targetTexture = rt1;
        cam2.targetTexture = rt2;
    }

    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > this.pub_time)
        {
            timeElapsed = 0.0f;
            readCamera();
        }

        Camera_Move();
        Camera_Rotate();
    }

    void readCamera()
    {
        RenderTexture.active = rt1;
        t2d.ReadPixels(new Rect(0, 0, rt1.width, rt1.height), 0, 0);
        t2d.Apply();
        RenderTexture.active = null;

        RenderTexture.active = rt2;
        t2d.ReadPixels(new Rect(0, 0, rt2.width, rt2.height), img_width, 0);
        t2d.Apply();
        RenderTexture.active = null;

        byte[] byt = t2d.EncodeToPNG();

        num += 1;

        image_msg = new CompressedImageMsg();
        image_msg.format = "png";
        image_msg.data = byt;

        _ros.Publish(topicName, image_msg);
        Debug.Log("Image " + num + " published");
    }

    void Camera_Move()
    {
        /**
         * 使用wasd控制相机移动
         */
        int times = 4;
        if (Input.GetKey(KeyCode.W))
        {
            camera_obj.transform.Translate(Vector3.forward * Time.deltaTime * times);
        }

        if (Input.GetKey(KeyCode.S))
        {
            camera_obj.transform.Translate(Vector3.back * Time.deltaTime * times);
        }

        if (Input.GetKey(KeyCode.A))
        {
            camera_obj.transform.Translate(Vector3.left * Time.deltaTime * times);
        }

        if (Input.GetKey(KeyCode.D))
        {
            camera_obj.transform.Translate(Vector3.right * Time.deltaTime * times);
        }

        if (Input.GetKey(KeyCode.LeftShift))
        {
            camera_obj.transform.Translate(Vector3.down * Time.deltaTime * times);
        }

        if (Input.GetKey(KeyCode.Space))
        {
            camera_obj.transform.Translate(Vector3.up * Time.deltaTime * times);
        }
    }

    void Camera_Rotate()
    {
        if (Input.GetKeyDown(KeyCode.M))
        {
            mouseEnable = !mouseEnable;

            if (mouseEnable)
            {
                Cursor.lockState = CursorLockMode.Locked;
                Cursor.visible = false;
            }
            else
            {
                Cursor.lockState = CursorLockMode.None;
                Cursor.visible = true;
            }
        }

        if (Input.GetMouseButton(0) || mouseEnable)
        {
            float x = Input.GetAxis("Mouse X");
            float y = Input.GetAxis("Mouse Y");

            camera_obj.transform.Rotate(Vector3.up, x * 2 * (mouseEnable ? 1 : -1));
            camera_obj.transform.Rotate(Vector3.right, y * 2 * (mouseEnable ? -1 : 1));

            if (camera_obj.transform.eulerAngles.z != 0)
            {
                camera_obj.transform.Rotate(Vector3.forward, -camera_obj.transform.eulerAngles.z);
            }
        }
    }
}