---
title: "Chapter 04: High-Fidelity Simulation with Unity"
description: "Photorealistic rendering, human-robot interaction, Unity ↔ ROS 2/Gazebo sync"
sidebar_label: "Chapter 04: High-Fidelity Simulation with Unity"
---

# Chapter 04: High-Fidelity Simulation with Unity

Unity provides a powerful platform for creating photorealistic simulation environments that can be used for advanced perception tasks, human-robot interaction studies, and visualization. Unlike physics-focused simulators like Gazebo, Unity excels at creating visually realistic environments that closely match real-world conditions.

## Unity for Robotics: Overview

Unity's robotics simulation capabilities are enhanced by the Unity Robotics Hub, which includes:

- **Unity Robotics Package**: Tools for ROS 2 integration
- **Visual Design Tools**: High-quality rendering and lighting
- **Physics Engine**: PhysX for realistic physics simulation
- **XR Support**: Virtual and augmented reality capabilities

## Setting Up Photorealistic Rendering

### Unity Rendering Pipeline

For high-fidelity rendering, Unity offers multiple rendering pipelines:

#### Universal Render Pipeline (URP)
- Good performance with quality
- Suitable for real-time robotics simulation
- Good lighting and post-processing effects

#### High Definition Render Pipeline (HDRP)
- Maximum visual fidelity
- Advanced lighting and materials
- Higher computational requirements

### Lighting Setup

Create realistic lighting conditions:

```csharp
// Example C# script for dynamic lighting
using UnityEngine;

public class DynamicLighting : MonoBehaviour
{
    public Light mainLight;
    public float dayNightCycleSpeed = 0.1f;

    void Update()
    {
        // Simulate day/night cycle
        float time = Time.time * dayNightCycleSpeed;
        float intensity = Mathf.Lerp(0.2f, 1.0f, (Mathf.Sin(time) + 1) / 2);
        mainLight.intensity = intensity;

        // Rotate light to simulate sun movement
        mainLight.transform.rotation = Quaternion.Euler(
            50 * Mathf.Sin(time),
            180 + 30 * Mathf.Cos(time),
            0
        );
    }
}
```

### Material and Texture Quality

For photorealistic materials:

1. **PBR Materials**: Use Physically Based Rendering materials
2. **High-Resolution Textures**: Use 4K textures where possible
3. **Normal Maps**: Add surface detail without geometry complexity
4. **Reflection Probes**: Capture environmental reflections

## Human-Robot Interaction Scenarios

### Creating Interactive Environments

Unity excels at creating environments where humans and robots interact:

```csharp
// Example: Human-Robot interaction script
using UnityEngine;
using UnityEngine.Events;

public class HumanRobotInteraction : MonoBehaviour
{
    public GameObject robot;
    public Transform interactionPoint;
    public UnityEvent onInteraction;

    private bool inInteractionRange = false;

    void Update()
    {
        float distance = Vector3.Distance(transform.position, interactionPoint.position);

        if (distance < 2.0f && Input.GetKeyDown(KeyCode.E))
        {
            if (inInteractionRange)
            {
                onInteraction.Invoke();
                Debug.Log("Human-Robot interaction triggered!");
            }
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Human"))
        {
            inInteractionRange = true;
            // Show interaction prompt
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Human"))
        {
            inInteractionRange = false;
            // Hide interaction prompt
        }
    }
}
```

### Social Robotics Simulation

Simulate human-like behaviors for interaction studies:

```csharp
// Example: Social behavior simulation
using UnityEngine;

public class SocialBehavior : MonoBehaviour
{
    public float personalSpaceRadius = 1.0f;
    public float comfortZoneRadius = 0.5f;

    public void MaintainPersonalSpace(Transform otherAgent)
    {
        float distance = Vector3.Distance(transform.position, otherAgent.position);

        if (distance < personalSpaceRadius)
        {
            // Move away to maintain personal space
            Vector3 direction = (transform.position - otherAgent.position).normalized;
            transform.position += direction * Time.deltaTime * 2.0f;
        }
        else if (distance > personalSpaceRadius + 0.5f)
        {
            // Move closer if too far
            Vector3 direction = (otherAgent.position - transform.position).normalized;
            transform.position += direction * Time.deltaTime * 1.0f;
        }
    }
}
```

## Unity ↔ ROS 2/Gazebo Synchronization

### Unity ROS TCP Connector

The Unity ROS TCP Connector enables communication between Unity and ROS 2:

```csharp
// Example: Unity subscribing to ROS topics
using UnityEngine;
using RosSharp.RosBridgeClient;

public class UnityROSSubscriber : MonoBehaviour
{
    private RosSocket rosSocket;

    void Start()
    {
        // Connect to ROS bridge
        rosSocket = new RosSocket(new RosSharp.WebSocketNetTransport("ws://localhost:9090"));

        // Subscribe to robot joint states
        rosSocket.Subscribe<sensor_msgs.JointState>(
            "/robot/joint_states",
            ReceiveJointStates
        );
    }

    private void ReceiveJointStates(sensor_msgs.JointState message)
    {
        // Update robot model based on received joint states
        for (int i = 0; i < message.name.Count; i++)
        {
            string jointName = message.name[i];
            float jointPosition = message.position[i];

            // Update corresponding joint in Unity
            Transform joint = transform.Find(jointName);
            if (joint != null)
            {
                joint.localRotation = Quaternion.Euler(0, jointPosition * Mathf.Rad2Deg, 0);
            }
        }
    }

    void OnDestroy()
    {
        rosSocket.Close();
    }
}
```

### Synchronizing Physics Between Unity and Gazebo

To maintain consistency between Unity's visual simulation and Gazebo's physics:

1. **Shared World Coordinates**: Ensure both simulators use the same coordinate system
2. **State Synchronization**: Regularly sync robot states between simulators
3. **Time Synchronization**: Match simulation time steps where possible

### Unity ↔ Gazebo Bridge Architecture

```csharp
// Example: State synchronization between Unity and Gazebo
using UnityEngine;
using System.Collections;

public class UnityGazeboBridge : MonoBehaviour
{
    public float syncInterval = 0.1f; // Sync every 100ms
    private float lastSyncTime = 0f;

    void Update()
    {
        if (Time.time - lastSyncTime > syncInterval)
        {
            SyncRobotStates();
            lastSyncTime = Time.time;
        }
    }

    private void SyncRobotStates()
    {
        // Send current Unity robot state to Gazebo
        SendRobotStateToGazebo();

        // Receive updated state from Gazebo
        ReceiveRobotStateFromGazebo();
    }

    private void SendRobotStateToGazebo()
    {
        // Implementation depends on communication protocol
        // Could use ROS topics, TCP/IP, or shared memory
    }

    private void ReceiveRobotStateFromGazebo()
    {
        // Update Unity robot model with state from Gazebo
    }
}
```

## Advanced Unity Features for Robotics

### Perception Simulation

Unity can simulate various perception modalities:

```csharp
// Example: RGB-D camera simulation
using UnityEngine;

public class RgbdCamera : MonoBehaviour
{
    public Camera rgbCamera;
    public Camera depthCamera;
    public RenderTexture rgbTexture;
    public RenderTexture depthTexture;

    void Start()
    {
        SetupCameras();
    }

    void SetupCameras()
    {
        // Configure RGB camera
        rgbCamera.enabled = true;
        rgbCamera.targetTexture = rgbTexture;

        // Configure depth camera
        depthCamera.enabled = true;
        depthCamera.targetTexture = depthTexture;
        depthCamera.depthTextureMode = DepthTextureMode.Depth;
    }

    public Texture2D GetRgbImage()
    {
        RenderTexture.active = rgbTexture;
        Texture2D rgbImage = new Texture2D(rgbTexture.width, rgbTexture.height);
        rgbImage.ReadPixels(new Rect(0, 0, rgbTexture.width, rgbTexture.height), 0, 0);
        rgbImage.Apply();
        RenderTexture.active = null;
        return rgbImage;
    }

    public float[] GetDepthData()
    {
        RenderTexture.active = depthTexture;
        Texture2D depthImage = new Texture2D(depthTexture.width, depthTexture.height, TextureFormat.RFloat, false);
        depthImage.ReadPixels(new Rect(0, 0, depthTexture.width, depthTexture.height), 0, 0);
        depthImage.Apply();

        Color[] pixels = depthImage.GetPixels();
        float[] depthValues = new float[pixels.Length];
        for (int i = 0; i < pixels.Length; i++)
        {
            depthValues[i] = pixels[i].r; // Depth is stored in red channel
        }

        RenderTexture.active = null;
        return depthValues;
    }
}
```

### Environment Generation

Create diverse and realistic environments:

1. **Procedural Generation**: Automatically generate varied environments
2. **Asset Libraries**: Use high-quality 3D assets for realistic scenes
3. **Dynamic Weather**: Simulate different weather conditions
4. **Crowd Simulation**: Add realistic human crowds for interaction

## Best Practices for Unity Robotics

### Performance Optimization

1. **LOD Systems**: Use Level of Detail for distant objects
2. **Occlusion Culling**: Hide objects not visible to cameras
3. **Texture Streaming**: Load textures on-demand
4. **Baking Lighting**: Precompute static lighting where possible

### Quality Assurance

1. **Validation Against Reality**: Compare simulation output with real sensors
2. **Cross-Validation**: Validate Unity results against Gazebo physics
3. **Repeatability**: Ensure consistent results across runs
4. **Documentation**: Maintain detailed records of simulation parameters

Unity provides an excellent platform for high-fidelity robotics simulation, particularly for perception tasks and human-robot interaction studies. When properly integrated with ROS 2 and synchronized with physics simulators like Gazebo, it creates a powerful environment for developing and testing advanced robotics applications.

## Navigation

- [← Chapter 03: Robot Modeling and Sensor Simulation](./chapter-03-sensors)
- [Chapter 05: Sim-to-Real Readiness](./chapter-05-sim-to-real) →