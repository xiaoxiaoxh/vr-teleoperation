using System;
using UnityEngine;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.Text;
using System.Net.Http;

[Serializable]
public class HandMessage
{
    public float[] q;
    public float[] pos;
    public float[] quat;
    public float[] thumbTip;
    public float[] indexTip;
    public float[] middleTip;
    public float[] ringTip;
    public float[] pinkyTip;
    public float squeeze;
    public int cmd;
    public bool[] buttonState;
    public HandMessage()
    {
        q = new float[24];
        cmd = 0;
        pos = new float[3];
        quat = new float[4];
        thumbTip = new float[7];
        indexTip = new float[7];
        middleTip = new float[7];
        ringTip = new float[7];
        pinkyTip = new float[7];
        buttonState = new bool[5];
    }

    public void TransformToAlignSpace()
    {
        if (WorldAlign.instance)
        {
            Vector3 vector3 = WorldAlign.instance.GetPosition(new Vector3(pos[0], pos[1], pos[2]));
            pos[0] = vector3.x;
            pos[1] = vector3.y;
            pos[2] = vector3.z;
            Quaternion quaternion = WorldAlign.instance.GetRotation(new Quaternion(quat[1], quat[2], quat[3], quat[0]));
            quat[0] = quaternion.w;
            quat[1] = quaternion.x;
            quat[2] = quaternion.y;
            quat[3] = quaternion.z;
        }
    }
}

[Serializable]
public class Message
{
    public float timestamp;
    public bool valid;
    public HandMessage rightHand;
    public HandMessage leftHand;
    public Message()
    {
        timestamp = Time.time;
        valid = false;
        rightHand = new HandMessage();
        leftHand = new HandMessage();
    }

    public void reset()
    {
        valid = false;
        rightHand.cmd = 0;
        leftHand.cmd = 0;
    }
}

public class HandDataCollector : MonoBehaviour
{
    public static HandDataCollector instance;
    public string ip;
    public int port;
    public Socket sender;

    //public bool mode = true;
    public int Hz = 30;

    public TMPro.TextMeshProUGUI showText;
    public Transform UIroot;


    public OVRSkeleton ovrhand_right;
    public OVRSkeleton ovrhand_left;

    public Transform controller_right;
    public Transform controller_left;

    public Mykeyboard mykeyboard;


    public Transform t;

    public GameObject toHide;
    public bool enable = true;
    public bool pressed_x = false;
    public bool pressed_y = false;
    public bool pressed_a = false;
    public bool pressed_b = false;
    public bool go_home_command = false;

    public static Message message;
    public bool LRinverse = false;

    protected void Start()
    {
        instance = this;
        sender = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        showText.transform.parent.GetChild(1).GetComponent<TMPro.TextMeshProUGUI>().text = ip;
        message = new Message();
        //InvokeRepeating("CollectAndSend", 1f / Hz, 1f / Hz);
        Time.fixedDeltaTime = 1f / Hz;
        //Physics.simulationMode = SimulationMode.Script;
        //StartCoroutine(WhileCollectAndSend());
    }

    private void FixedUpdate()
    {
        CollectAndSend();
    }

    bool calibrationMode = false;
    bool cold = false;
    async void SwitchMode()
    {
        if (cold) return;
        cold = true;
        calibrationMode = !calibrationMode;
        WorldAlign.instance.SwitchAlign(calibrationMode);
        await Task.Delay(500);
        cold = false;
    }
    public void Update()
    {
        if (OVRInput.Get(OVRInput.RawButton.X) && OVRInput.Get(OVRInput.RawButton.A))
        {
            SwitchMode();
        }
        if (calibrationMode) return;

        if (OVRInput.GetDown(OVRInput.RawButton.X))
        {
            pressed_x = true;
            Debug.Log("Button Pressed X");
        }
        if (OVRInput.GetDown(OVRInput.RawButton.A))
        {
            pressed_a = true;
            Debug.Log("Button Pressed A");
        }

        if (OVRInput.GetDown(OVRInput.RawButton.LThumbstick))
        {
            toHide.SetActive(!enable);
            enable = !enable;
        }

        if (OVRInput.GetDown(OVRInput.RawButton.RThumbstick))
        {
            go_home_command = true;
        }

        if (OVRInput.GetDown(OVRInput.RawButton.Y))
        {
            pressed_y = true;
            Debug.Log("Button Pressed Y");
        }


        if (OVRInput.GetDown(OVRInput.RawButton.B))
        {
            pressed_b = true;
            Debug.Log("Button Pressed B");
        }



    }


    public float[] Transform2List(Transform tip)
    {
        var res = new float[7];
        var p = tip.position;
        var q = tip.rotation;
        res[0] = p.x;
        res[1] = p.y;
        res[2] = p.z;

        res[3] = q.w;
        res[4] = q.x;
        res[5] = q.y;
        res[6] = q.z;

        return res;
    }


    public void CollectAndSend()
    {

        message.reset();
        Cotroller_collect();
        if (!message.valid)
        {
            showText.text = "message is not valid";
            return;
        }

        //新增空间转换
        message.leftHand.TransformToAlignSpace();
        message.rightHand.TransformToAlignSpace();

        message.timestamp = Time.time;

        string mes = JsonUtility.ToJson(message);
        byte[] bodyRaw = Encoding.UTF8.GetBytes(mes);
        string url = $"http://{ip}:{port}/unity";
        var content = new ByteArrayContent(bodyRaw);
        client.PostAsync(url, content);
    }
    HttpClient client = new HttpClient();


    public void Cotroller_collect()
    {

        UIroot.position = controller_right.position - new Vector3(0, 0.2f, 0);
        UIroot.LookAt(Camera.main.transform);


        message.rightHand.pos[0] = controller_right.position.x;
        message.rightHand.pos[1] = controller_right.position.y;
        message.rightHand.pos[2] = controller_right.position.z;

        message.rightHand.quat[0] = controller_right.rotation.w;
        message.rightHand.quat[1] = controller_right.rotation.x;
        message.rightHand.quat[2] = controller_right.rotation.y;
        message.rightHand.quat[3] = controller_right.rotation.z;

        message.leftHand.pos[0] = controller_left.position.x;
        message.leftHand.pos[1] = controller_left.position.y;
        message.leftHand.pos[2] = controller_left.position.z;

        message.leftHand.quat[0] = controller_left.rotation.w;
        message.leftHand.quat[1] = controller_left.rotation.x;
        message.leftHand.quat[2] = controller_left.rotation.y;
        message.leftHand.quat[3] = controller_left.rotation.z;

        if (pressed_x)
        {
            pressed_x = false;
            message.leftHand.cmd = -2;
            showText.text = "left stop";
        }
        if (pressed_y)
        {
            pressed_y = false;
            message.leftHand.cmd = 2;
            showText.text = "left start";
        }
        if (pressed_a)
        {
            pressed_a = false;
            message.rightHand.cmd = -2;
            showText.text = "right stop";
        }
        if (pressed_b)
        {
            pressed_b = false;
            message.rightHand.cmd = 2;
            showText.text = "right start";
        }
        if (go_home_command)
        {
            go_home_command = false;
            showText.text = "cmd=3";
            message.leftHand.cmd = 3;
        }

        message.leftHand.squeeze = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger);
        message.rightHand.squeeze = OVRInput.Get(OVRInput.Axis1D.SecondaryIndexTrigger);

        message.leftHand.buttonState[0] = OVRInput.Get(OVRInput.RawButton.Y);
        message.leftHand.buttonState[1] = OVRInput.Get(OVRInput.RawButton.X);
        message.leftHand.buttonState[2] = OVRInput.Get(OVRInput.RawButton.LThumbstick);
        message.leftHand.buttonState[3] = OVRInput.Get(OVRInput.RawButton.LIndexTrigger);
        message.leftHand.buttonState[4] = OVRInput.Get(OVRInput.RawButton.LHandTrigger);

        message.rightHand.buttonState[0] = OVRInput.Get(OVRInput.RawButton.B);
        message.rightHand.buttonState[1] = OVRInput.Get(OVRInput.RawButton.A);
        message.rightHand.buttonState[2] = OVRInput.Get(OVRInput.RawButton.RThumbstick);
        message.rightHand.buttonState[3] = OVRInput.Get(OVRInput.RawButton.RIndexTrigger);
        message.rightHand.buttonState[4] = OVRInput.Get(OVRInput.RawButton.RHandTrigger);

        if (LRinverse)
        {
            var temp = message.leftHand;
            message.leftHand = message.rightHand;
            message.rightHand = temp;
        }

        message.valid = true;
    }


    public void RefreshIP(string ip)
    {
        this.ip = ip;
    }
}
