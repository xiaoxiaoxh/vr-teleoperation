using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using System.IO;
using System;
using System.Net.Sockets;


public class Mykeyboard : MonoBehaviour
{
    public TMPro.TextMeshProUGUI text;
    public TMPro.TextMeshProUGUI showText;
    public HandDataCollector Client;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
    }

    public void Add(string s)
    {
        text.text = text.text += s;
    }

    public void Remove()
    {
        text.text = text.text.Remove(text.text.Length - 1);
    }

    public void RefreshIP()
    {
        Client.RefreshIP(text.text);

    }

    public void StartTracking()
    {

    }

    public void Test()
    {
        Debug.LogError("pose triggered");
    }

    public void RightThumbUp()
    {
        Client.pressed_x = true;
    }

    public void LeftThumbUp()
    {
        Client.pressed_a = true;
    }

    public void SwitchLRController()
    {
        Client.LRinverse = !Client.LRinverse;
    }
}
