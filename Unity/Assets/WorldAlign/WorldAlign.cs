using System.Threading.Tasks;
using UnityEngine;

public class WorldAlign : MonoBehaviour
{

    public static WorldAlign instance;
    public GameObject coord;
    bool running = false;
    private void Awake()
    {
        instance = this;
        running = false;
        coord.SetActive(false);
        DontDestroyOnLoad(this.gameObject);
    }

    public void SwitchAlign(bool run)
    {
        running = run;
        coord.SetActive(run);

    }

    Vector3 startPositionRight;
    Vector3 startPositionLeft;
    Vector3 startPositionTransform;
    private void Update()
    {
        if (!running) return;
        if (OVRInput.GetDown(OVRInput.RawButton.RIndexTrigger))
        {
            startPositionRight = OVRInput.GetLocalControllerPosition(OVRInput.Controller.RTouch);
        }
        if (OVRInput.Get(OVRInput.RawButton.RIndexTrigger))
        {
            Vector3 offset = OVRInput.GetLocalControllerPosition(OVRInput.Controller.RTouch) - startPositionRight;
            offset.y = 0;
            transform.LookAt(transform.position + offset);
        }

        if (OVRInput.GetDown(OVRInput.RawButton.LIndexTrigger))
        {
            startPositionTransform = transform.position;
            startPositionLeft = OVRInput.GetLocalControllerPosition(OVRInput.Controller.LTouch);
        }
        if (OVRInput.Get(OVRInput.RawButton.LIndexTrigger))
        {
            Vector3 offset = OVRInput.GetLocalControllerPosition(OVRInput.Controller.LTouch) - startPositionLeft;
            transform.position = startPositionTransform + offset;
        }

        if (OVRInput.GetDown(OVRInput.RawButton.A) || OVRInput.GetDown(OVRInput.RawButton.X))
        {
            if (transform.GetChild(0).localScale.magnitude > 0.2f)
                transform.GetChild(0).localScale -= Vector3.one * 0.1F;
        }
        if (OVRInput.GetDown(OVRInput.RawButton.B) || OVRInput.GetDown(OVRInput.RawButton.Y))
        {
            transform.GetChild(0).localScale += Vector3.one * 0.1F;
        }
        if (OVRInput.GetDown(OVRInput.RawButton.LThumbstick) || OVRInput.GetDown(OVRInput.RawButton.RThumbstick))
        {
            transform.position = Vector3.zero;
            transform.GetChild(0).localScale = Vector3.one;
        }
    }



    public Vector3 GetPosition(Vector3 worldPosition)
    {
        return transform.TransformPoint(worldPosition);
    }

    public Vector3 GetEuler(Vector3 worldEuler)
    {
        return Quaternion.Inverse(transform.rotation) * worldEuler;
    }

    public Quaternion GetRotation(Quaternion worldEuler)
    {
        return Quaternion.Inverse(transform.rotation) * worldEuler;
    }
}
