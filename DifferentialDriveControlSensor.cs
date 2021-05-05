/**
 * Copyright (c) 2021 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using Simulator.Bridge;
using Simulator.Utilities;
using UnityEngine;
using Simulator.Sensors.UI;
using System.Collections.Generic;

namespace Simulator.Sensors
{
    [SensorType("Differential Drive Control", new System.Type[] { typeof(AutowareVehicleControlData) })]
    public class DifferentialDriveControlSensor : SensorBase
    {
        IAgentController Controller;
        IVehicleDynamics Dynamics;

        [SensorParameter]
        public double WheelRadius;

        [SensorParameter]
        public double WheelSeparation;

        double LastControlUpdate = 0f;
        double LeftWheelRotationVelocity;  // degrees per second
        double RightWheelRotationVelocity;  // degrees per second

        private void Awake()
        {
            LastControlUpdate = SimulatorManager.Instance.CurrentTime;
            Controller = GetComponentInParent<IAgentController>();
            Dynamics = GetComponentInParent<IVehicleDynamics>();
        }

        private void FixedUpdate()
        {
            if (SimulatorManager.Instance.CurrentTime - LastControlUpdate < 0.5f)
            {
                //
            }
        }

        private void CalculateWheelRates(Vector3 linear, Vector3 angular)
        {
            double YawRateVelocity = angular.y * WheelSeparation / 2;
            LeftWheelRotationVelocity = (linear.z / WheelRadius) * Mathf.Rad2Deg + YawRateVelocity;
            RightWheelRotationVelocity = (linear.z / WheelRadius) * Mathf.Rad2Deg - YawRateVelocity;
        }

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            bridge.AddSubscriber<DifferentialDriveControlData>(Topic, data =>
            {
                LastControlUpdate = SimulatorManager.Instance.CurrentTime;
                CalculateWheelRates(data.LinearVelocity, data.AngularVelocity);
            });
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            //
        }

        public override void OnVisualizeToggle(bool state)
        {
            //
        }
    }
}