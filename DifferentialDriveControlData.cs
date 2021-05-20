/**
 * Copyright (c) 2021 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using UnityEngine;
using Simulator.Bridge;

namespace Simulator.Sensors
{
    public struct WheelData
    {
        public float Left;
        public float Right;
    }

    // actual ROS type
    // can use anything from Simulator.Bridge.Ros.Ros namespace
    [Bridge.Ros2.MessageType("lgsvl_msgs/DifferentialDriveOdometry")]
    public struct DifferentialDriveOdometry
    {
        public WheelData AngularVelocity;
        public WheelData LinearVelocity;
        public Bridge.Ros2.Ros.Vector3 Pose;
        public Bridge.Ros2.Ros.Vector3 TwistLinear;
        public Bridge.Ros2.Ros.Vector3 TwistAngular;
    }

    public class DifferentialDriveControlData
    {
        public float LinearVelocity;
        public float AngularVelocity;
    }

    public class DifferentialDriveControlPlugin : ISensorBridgePlugin
    {
        public void Register(IBridgePlugin plugin)
        {
            if (plugin.Factory is Bridge.Ros2.Ros2BridgeFactory)
            {
                plugin.Factory.RegSubscriber<DifferentialDriveControlData, Bridge.Ros2.Ros.Twist>(plugin, ConvertMsg);
                plugin.Factory.RegPublisher(plugin, (DifferentialDriveOdometry data) => data);
            }
        }

        public DifferentialDriveControlData ConvertMsg(Bridge.Ros2.Ros.Twist data)
        {
            return new DifferentialDriveControlData()
            {
                LinearVelocity = (float)-data.linear.x,
                AngularVelocity = (float)-data.angular.z
            };
        }
    }
}