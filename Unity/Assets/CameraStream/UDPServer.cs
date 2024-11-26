using System.Collections.Generic;
using System.Net;
using UnityEngine;
using System.Net.Sockets;
using System.Runtime.Serialization;
using System;
using System.Threading;
using Newtonsoft.Json;
using Newtonsoft.Json.Bson;
using System.IO;
using UnityEngine.UI;

public class UDPServer : MonoBehaviour
{
    public int portRobot = 10001;
    public int portImage = 10004;
    public Transform leftTCP;
    public Transform rightTCP;
    UdpClient serverRobot;
    UdpClient serverImage;

    JsonSerializer serializer = new JsonSerializer();

    public Text logText;

    Transform point;
    Thread threadRobot, threadImage;
    void Start()
    {
        point = new GameObject().transform;
        threadRobot = new Thread(ServerRobot);
        threadRobot.Start();
        threadImage = new Thread(ServerImage);
        threadImage.Start();
    }

    void ServerRobot()
    {
        serverRobot = new UdpClient(portRobot);
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
        while (true)
        {
            try
            {
                byte[] receiveBytes = serverRobot.Receive(ref remoteEndPoint);
                using (MemoryStream ms = new MemoryStream(receiveBytes))
                {
                    using (BsonReader reader = new BsonReader(ms))
                    {
                        BimanualRobotStates pose = serializer.Deserialize<BimanualRobotStates>(reader);
                        UnityMainThreadDispatcher.Instance().Enqueue(() => UpdateRobot(pose));
                    }
                }
                //BimanualRobotStates pose = MessagePackSerializer.Deserialize<BimanualRobotStates>(receiveBytes);
                //UnityMainThreadDispatcher.Instance().Enqueue(() => UpdateRobot(pose));
            }
            catch (Exception e)
            {
                Debug.LogError($"SocketException: {e.Message}");
            }

        }
    }

    void ServerImage()
    {
        serverImage = new UdpClient(portImage);
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
        while (true)
        {
            try
            {
                ReceiveImage(serverImage, remoteEndPoint);
            }
            catch (Exception e)
            {
                //logText.text = e.Message + Guid.NewGuid().ToString();
                Debug.LogError($"SocketException: {e.Message}");
            }
        }
    }
    List<byte> bytes = new List<byte>();
    void ReceiveImage(UdpClient serverImage, IPEndPoint remoteEndPoint)
    {
        byte[] lengthBytes = serverImage.Receive(ref remoteEndPoint);
        if (lengthBytes.Length != 4)
            throw new Exception();
        uint lengthInt = BitConverter.ToUInt32(lengthBytes, 0);
        byte[] chunkBytes = serverImage.Receive(ref remoteEndPoint);
        if (chunkBytes.Length != 4)
            throw new Exception();
        uint lengthChunk = BitConverter.ToUInt32(chunkBytes, 0);
        bytes.Clear();
        int count = Mathf.CeilToInt(lengthInt / (float)lengthChunk);
        for (int index = 0; index < count; index++)
        {
            byte[] buffer = serverImage.Receive(ref remoteEndPoint);
            bytes.AddRange(buffer);
        }
        using (MemoryStream ms = new MemoryStream(bytes.ToArray()))
        {
            using (BsonReader reader = new BsonReader(ms))
            {
                ImageMessage imageData = serializer.Deserialize<ImageMessage>(reader);
                UnityMainThreadDispatcher.Instance().Enqueue(() => UpdateImage(imageData));
            }
        }
    }
    [DataContract]
    public class Image
    {
        [DataMember]
        public string id { get; set; }
        [DataMember]
        public List<float> position { get; set; }
        [DataMember]
        public List<float> rotation { get; set; }
        [DataMember]
        public List<float> scale { get; set; }
        [DataMember]
        public byte[] image { get; set; }
    }
    [DataContract]
    public class ImageMessage
    {
        [DataMember]
        public List<Image> images { get; set; }
    }
    [DataContract]
    public class UIMessage
    {
        [DataMember]
        public string text { get; set; }
    }

    [DataContract]
    public class BimanualRobotStates
    {
        [DataMember]
        public List<float> leftRobotTCP { get; set; }
        [DataMember]
        public List<float> rightRobotTCP { get; set; }
        [DataMember]
        public List<float> leftGripperState { get; set; }
        [DataMember]
        public List<float> rightGripperState { get; set; }
    }

    void UpdateRobot(BimanualRobotStates pose)
    {
        leftTCP.localPosition = new Vector3(pose.leftRobotTCP[0], pose.leftRobotTCP[1], pose.leftRobotTCP[2]);
        rightTCP.localPosition = new Vector3(pose.rightRobotTCP[0], pose.rightRobotTCP[1], pose.rightRobotTCP[2]);
        leftTCP.localRotation = new Quaternion(pose.leftRobotTCP[4], pose.leftRobotTCP[5], pose.leftRobotTCP[6], pose.leftRobotTCP[3]);
        rightTCP.localRotation = new Quaternion(pose.rightRobotTCP[4], pose.rightRobotTCP[5], pose.rightRobotTCP[6], pose.rightRobotTCP[3]);
    }




    Dictionary<string, RawImage> imageUI = new();
    public GameObject oneUI;
    void UpdateImage(ImageMessage imageData)
    {
        foreach (var item in imageData.images)
        {
            if (!imageUI.ContainsKey(item.id))
            {
                RawImage one = GameObject.Instantiate(oneUI).GetComponentInChildren<RawImage>();
                one.canvas.transform.SetParent(WorldAlign.instance.transform);
                one.texture = new Texture2D(1, 1);
                imageUI.Add(item.id, one);
            }
            imageUI[item.id].canvas.transform.localPosition = new Vector3(item.position[0], item.position[1], item.position[2]);
            imageUI[item.id].canvas.transform.localEulerAngles = new Vector3(item.rotation[0], item.rotation[1], item.rotation[2]);
            imageUI[item.id].canvas.transform.localScale = new Vector3(item.scale[0], item.scale[1], item.scale[2]);
            (imageUI[item.id].texture as Texture2D).LoadImage(item.image);
        }
    }
    private void OnDestroy()
    {
        serverRobot?.Close();
        serverImage?.Close();
        threadRobot?.Abort();
        threadImage?.Abort();
    }
}
