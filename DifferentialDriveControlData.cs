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
                plugin.Factory.RegPublisher(plugin, (Bridge.Ros2.Ros.Odometry data) => data);
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