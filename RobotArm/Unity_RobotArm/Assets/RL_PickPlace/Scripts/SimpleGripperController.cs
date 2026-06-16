using UnityEngine;

public class SimpleGripperController : MonoBehaviour
{
    [Header("Finger Objects")]
    public Transform leftFinger;
    public Transform rightFinger;

    [Header("Movement Axis")]
    public Vector3 leftCloseOffset = new Vector3(0.02f, 0f, 0f);
    public Vector3 rightCloseOffset = new Vector3(-0.02f, 0f, 0f);

    [Header("Speed")]
    public float moveSpeed = 0.08f;

    [Header("Debug Keyboard Test")]
    public bool enableKeyboardTest = true;
    public KeyCode closeKey = KeyCode.C;
    public KeyCode openKey = KeyCode.O;

    [Header("Override realvirtual")]
    public bool disableRealvirtualGripperComponents = true;

    private Vector3 leftOpenLocalPosition;
    private Vector3 rightOpenLocalPosition;

    private Vector3 leftClosedLocalPosition;
    private Vector3 rightClosedLocalPosition;

    private bool isClosed = false;
    private bool positionsInitialized = false;

    private void Start()
    {
        InitializeFingerPositions();

        if (disableRealvirtualGripperComponents)
        {
            DisableRealvirtualComponents();
        }
    }

    private void InitializeFingerPositions()
    {
        if (leftFinger != null)
        {
            leftOpenLocalPosition = leftFinger.localPosition;
            leftClosedLocalPosition = leftOpenLocalPosition + leftCloseOffset;
        }

        if (rightFinger != null)
        {
            rightOpenLocalPosition = rightFinger.localPosition;
            rightClosedLocalPosition = rightOpenLocalPosition + rightCloseOffset;
        }

        positionsInitialized = true;
    }

    private void Update()
    {
        if (enableKeyboardTest)
        {
            if (Input.GetKeyDown(closeKey))
            {
                CloseGripper();
            }

            if (Input.GetKeyDown(openKey))
            {
                OpenGripper();
            }
        }
    }

    private void LateUpdate()
    {
        if (!positionsInitialized)
        {
            InitializeFingerPositions();
        }

        if (disableRealvirtualGripperComponents)
        {
            DisableRealvirtualComponents();
        }

        MoveFingers();
    }

    private void MoveFingers()
    {
        if (leftFinger == null || rightFinger == null)
            return;

        Vector3 leftTarget = isClosed ? leftClosedLocalPosition : leftOpenLocalPosition;
        Vector3 rightTarget = isClosed ? rightClosedLocalPosition : rightOpenLocalPosition;

        leftFinger.localPosition = Vector3.MoveTowards(
            leftFinger.localPosition,
            leftTarget,
            moveSpeed * Time.deltaTime
        );

        rightFinger.localPosition = Vector3.MoveTowards(
            rightFinger.localPosition,
            rightTarget,
            moveSpeed * Time.deltaTime
        );
    }

    private void DisableRealvirtualComponents()
    {
        DisableRealvirtualComponentsOn(gameObject);

        if (leftFinger != null)
        {
            DisableRealvirtualComponentsOn(leftFinger.gameObject);
        }

        if (rightFinger != null)
        {
            DisableRealvirtualComponentsOn(rightFinger.gameObject);
        }
    }

    private void DisableRealvirtualComponentsOn(GameObject obj)
    {
        if (obj == null)
            return;

        MonoBehaviour[] components = obj.GetComponents<MonoBehaviour>();

        foreach (MonoBehaviour component in components)
        {
            if (component == null)
                continue;

            if (component == this)
                continue;

            string typeName = component.GetType().Name;

            if (
                typeName.Contains("Drive") ||
                typeName.Contains("Gear") ||
                typeName.Contains("Grip") ||
                typeName.Contains("Sensor")
            )
            {
                component.enabled = false;
            }
        }
    }

    public void CloseGripper()
    {
        isClosed = true;
    }

    public void OpenGripper()
    {
        isClosed = false;
    }

    public bool IsClosed()
    {
        return isClosed;
    }

    public void ToggleGripper()
    {
        isClosed = !isClosed;
    }
}