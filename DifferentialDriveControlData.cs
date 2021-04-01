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
        public Vector3 LinearVelocity;
        public Vector3 AngularVelocity;
    }

    public class DifferentialDriveControlPlugin : ISensorBridgePlugin
    {
        public void Register(IBridgePlugin plugin)
        {
            if (plugin.Factory is Bridge.Ros2.Ros2BridgeFactory)
            {
                plugin.Factory.RegSubscriber<DifferentialDriveControlData, Bridge.Ros2.Ros.Twist>(plugin, ConvertMsg);
            }
        }

        public DifferentialDriveControlData ConvertMsg(Bridge.Ros2.Ros.Twist data)
        {
            return new DifferentialDriveControlData()
            {
                LinearVelocity = new Vector3()
                {
                    x = -(float) data.linear.y,
                    y = (float) data.linear.z,
                    z = (float) data.linear.x,  // only actually use z
                },
                AngularVelocity = new Vector3()
                {
                    x = (float) data.angular.y,
                    y = -(float) data.angular.z,  // only actually use y
                    z = -(float) data.angular.x,
                }
            };
        }
    }
}