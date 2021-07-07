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
    [SensorType("Differential Drive Control", new System.Type[] { typeof(DifferentialDriveControlData), typeof(Bridge.Ros2.Ros.Odometry) })]
    public class DifferentialDriveControlSensor : SensorBase, IVehicleInputs
    {
        private IAgentController Controller;
        private IVehicleDynamics Dynamics;

        [SensorParameter]
        public float WheelRadius;
        private float DivideWheelRadius = 0.0f;

        [SensorParameter]
        public float WheelSeparation;

        [SensorParameter]
        public string LeftWheelLinkPath;
        [SensorParameter]
        public string RightWheelLinkPath;

        [SensorParameter]
        public float P_Gain = 1.0f;

        [SensorParameter]
        public float I_Gain = 0.05f;

        [SensorParameter]
        public float D_Gain = 0.0f;

        [SensorParameter]
        public string OdometryTopic;

        [SensorParameter]
        public string OdometryChildFrame;

        double LastControlUpdate = 0f;

        private ArticulationBody RightWheel;
        private ArticulationBody LeftWheel;
        private Motor LeftMotor = null;
        private Motor RightMotor = null;
        private float TargetLinearVelocity;
        private float TargetAngularVelocity;
        private float ADAccelInput = 0f;
        private float ADSteerInput = 0f;

        protected override void Initialize()
        {
            NextSend = Time.time + 1.0f / Frequency;
            ImuInitialRotation = transform.rotation.eulerAngles;
            PreviousImuPosition = transform.position;
            PreviousImuRotation = Vector3.zero;
            _odomVelocity.Set(0, 0);
            _odomPose.Set(0, 0, 0);
            _lastTheta = 0.0f;

            Controller = GetComponentInParent<IAgentController>();
            if (Controller == null)
            {
                Debug.Log("missing agent controller");
                this.enabled = false;
                return;
            }
            Dynamics = GetComponentInParent<IVehicleDynamics>();

            LastControlUpdate = SimulatorManager.Instance.CurrentTime;
            LeftWheel = Controller.AgentGameObject.transform.Find(LeftWheelLinkPath).GetComponent<ArticulationBody>(); ;
            RightWheel = Controller.AgentGameObject.transform.Find(RightWheelLinkPath).GetComponent<ArticulationBody>(); ;

            if (RightWheel == null || LeftWheel == null)
            {
                Debug.LogWarning($"wheels not set in DifferentialDriveSMI: {LeftWheel} {RightWheel}, nap time.");
                this.enabled = false;
                return;
            }

            if (WheelSeparation == 0)
            {
                WheelSeparation = (RightWheel.transform.position - LeftWheel.transform.position).magnitude;
            }

            if (WheelRadius == 0)
            {
                var capsuleCollider = RightWheel.GetComponentInChildren<CapsuleCollider>();
                var collider = RightWheel.GetComponentInChildren<Collider>();
                if (capsuleCollider != null)
                {
                    WheelRadius = capsuleCollider.radius;
                }
                else if (collider != null)
                {
                    // FIXME test this branch
                    WheelRadius = collider.bounds.extents.x;
                }

                DivideWheelRadius = 1.0f / WheelRadius;
            }

            LeftMotor = LeftWheel.gameObject.AddComponent<Motor>();
            LeftMotor.SetTargetJoint(LeftWheel);
            LeftMotor.SetPID(P_Gain, I_Gain, D_Gain);
            RightMotor = LeftWheel.gameObject.AddComponent<Motor>();
            RightMotor.SetTargetJoint(RightWheel);
            RightMotor.SetPID(P_Gain, I_Gain, D_Gain);
        }

        protected override void Deinitialize()
        {
            Destroy(LeftMotor);
            Destroy(RightMotor);
        }

        private void FixedUpdate()
        {
            if (SimulatorManager.Instance.CurrentTime - LastControlUpdate < 0.5f)
            {
                SteerInput = ADSteerInput;
                AccelInput = ADAccelInput;
            }

            TargetAngularVelocity = Controller.SteerInput;
            TargetLinearVelocity = Controller.AccelInput;
            UpdateIMU();
            SetTwistDrive(TargetLinearVelocity, TargetAngularVelocity);

            LeftMotor.Feedback.SetRotatingTargetVelocity(TargetAngularVelocity);
            RightMotor.Feedback.SetRotatingTargetVelocity(TargetAngularVelocity);

            UpdateOdom(Time.fixedDeltaTime);

            LeftMotor.Feedback.SetRotatingVelocity(_odomVelocity.y);
            RightMotor.Feedback.SetRotatingVelocity(_odomVelocity.y);
        }

        public void SetTwistDrive(float linearVelocity, float angularVelocity)
        {
            // m/s, rad/s
            // var linearVelocityLeft = ((2 * linearVelocity) + (angularVelocity * wheelBase)) / (2 * wheelRadius);
            // var linearVelocityRight = ((2 * linearVelocity) + (angularVelocity * wheelBase)) / (2 * wheelRadius);
            var angularCalculation = angularVelocity * WheelSeparation * 0.5f;
            var linearVelocityLeft = linearVelocity - angularCalculation;
            var linearVelocityRight = linearVelocity + angularCalculation;

            SetDifferentialDrive(linearVelocityLeft, linearVelocityRight);
        }
        public void SetDifferentialDrive(float linearVelocityLeft, float linearVelocityRight)
        {
            var angularVelocityLeft = linearVelocityLeft * DivideWheelRadius * Mathf.Rad2Deg;
            var angularVelocityRight = linearVelocityRight * DivideWheelRadius * Mathf.Rad2Deg;
            SetMotorVelocity(angularVelocityLeft, angularVelocityRight);
        }

        private void SetMotorVelocity(float angularVelocityLeft, float angularVelocityRight)
        {
            var robotIsRotating = Mathf.Sign(angularVelocityLeft) != Mathf.Sign(angularVelocityRight);
            LeftMotor.Feedback.SetMotionRotating(robotIsRotating);
            RightMotor.Feedback.SetMotionRotating(robotIsRotating);
            LeftMotor.SetVelocityTarget(angularVelocityLeft);
            RightMotor.SetVelocityTarget(angularVelocityRight);
        }

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            if (!(bridge.Plugin.Factory is Simulator.Bridge.Ros2.Ros2BridgeFactory))
            {
                Debug.LogWarning("DifferentialDriveControlSensor only works with ROS2 bridge.");
                return;
            }

            bridge.AddSubscriber<DifferentialDriveControlData>(Topic, data =>
            {
                LastControlUpdate = SimulatorManager.Instance.CurrentTime;
                ADAccelInput = data.LinearVelocity;
                ADSteerInput = data.AngularVelocity;
            });
            Bridge = bridge;
            Publish = bridge.AddPublisher<Bridge.Ros2.Ros.Odometry>(OdometryTopic);
        }

        [SensorParameter]
        [Range(1f, 100f)]
        public float Frequency = 10.0f;
        private Bridge.Ros2.Ros.Odometry odom;

        [AnalysisMeasurement(MeasurementType.Distance)]
        private float Distance = 0f;
        private Vector3 PrevPos = new Vector3(0f, 0f, 0f);
        private float NextSend;
        private BridgeInstance Bridge;
        private Publisher<Bridge.Ros2.Ros.Odometry> Publish;

        public override System.Type GetDataBridgePlugin()
        {
            return typeof(DifferentialDriveControlPlugin);
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            Debug.Assert(visualizer != null);

            var graphData = new Dictionary<string, object>()
            {
                {"TargetLinearVelocity", TargetAngularVelocity},
                {"TargetAngularVelocity", TargetLinearVelocity},
                {"Speed", Dynamics.Velocity.magnitude},
                {"LastControlUpdate", LastControlUpdate},
            };
            visualizer.UpdateGraphValues(graphData);
        }

        public override void OnVisualizeToggle(bool state) { }

        public void Update()
        {
            if (SimulatorManager.Instance.CurrentTime - LastControlUpdate >= 0.5)
            {
                ADAccelInput = ADSteerInput = SteerInput = AccelInput = 0.0f;
            }
            // distance analysis
            Distance += Vector3.Distance(transform.position, PrevPos) / 1000;
            PrevPos = transform.position;

            if (Time.time < NextSend)
            {
                return;
            }
            NextSend = Time.time + 1.0f / Frequency;

            if (Bridge != null && Bridge.Status == Status.Connected)
            {
                Publish(odom);
            }
            else
            {
                Debug.Log($"{Bridge != null} {Bridge?.Status}");
            }
        }
        /// <summary>Calculate odometry on this robot</summary>
        /// <remarks>rad per second for `theta`</remarks>
        private void CalculateOdometry(float duration, float angularVelocityLeftWheel, float angularVelocityRightWheel, float theta)
        {
            // circumference of wheel [rad] per step time.
            var wheelCircumLeft = angularVelocityLeftWheel * duration;
            var wheelCircumRight = angularVelocityRightWheel * duration;

            var deltaTheta = theta - _lastTheta;

            if (deltaTheta > Mathf.PI)
            {
                deltaTheta -= 2 * Mathf.PI;
            }
            else if (deltaTheta < -Mathf.PI)
            {
                deltaTheta += 2 * Mathf.PI;
            }

            // compute odometric pose
            var poseLinear = WheelRadius * (wheelCircumLeft + wheelCircumRight) * 0.5f;
            var halfDeltaTheta = deltaTheta * 0.5f;
            _odomPose.x += poseLinear * Mathf.Cos(_odomPose.z + halfDeltaTheta);
            _odomPose.y += poseLinear * Mathf.Sin(_odomPose.z + halfDeltaTheta);
            _odomPose.z += deltaTheta;

            if (_odomPose.z > Mathf.PI)
            {
                _odomPose.z -= 2 * Mathf.PI;
            }
            else if (_odomPose.z < -Mathf.PI)
            {
                _odomPose.z += 2 * Mathf.PI;
            }

            // compute odometric instantaneouse velocity
            var v = poseLinear / duration; // v = translational velocity [m/s]
            var w = deltaTheta / duration; // w = rotational velocity [rad/s]

            _odomVelocity.x = v;
            _odomVelocity.y = w;

            _lastTheta = theta;
        }
        void UpdateIMU()
        {
            // Caculate orientation and acceleration
            var imuRotation = transform.rotation.eulerAngles - ImuInitialRotation;
            ImuOrientation = Quaternion.Euler(imuRotation.x, imuRotation.y, imuRotation.z);

            ImuAngularVelocity.x = Mathf.DeltaAngle(imuRotation.x, PreviousImuRotation.x) / Time.fixedDeltaTime;
            ImuAngularVelocity.y = Mathf.DeltaAngle(imuRotation.y, PreviousImuRotation.y) / Time.fixedDeltaTime;
            ImuAngularVelocity.z = Mathf.DeltaAngle(imuRotation.z, PreviousImuRotation.z) / Time.fixedDeltaTime;

            var currentPosition = transform.position;
            var currentLinearVelocity = (currentPosition - PreviousImuPosition) / Time.fixedDeltaTime;
            ImuLinearAcceleration = (currentLinearVelocity - PreviousLinearVelocity) / Time.fixedDeltaTime;
            ImuLinearAcceleration.y += (-Physics.gravity.y);

            PreviousImuRotation = imuRotation;
            PreviousImuPosition = currentPosition;
            PreviousLinearVelocity = currentLinearVelocity;
        }
        private float _lastTheta = 0.0f;
        private Vector3 _odomPose = Vector3.zero;
        private Vector2 _odomVelocity = Vector2.zero;

        public void Reset()
        {
            ImuInitialRotation = transform.rotation.eulerAngles;
            PreviousImuPosition = transform.position;
            PreviousImuRotation = Vector3.zero;
            _odomVelocity.Set(0, 0);
            _odomPose.Set(0, 0, 0);
            _lastTheta = 0.0f;
        }


        static readonly double[] defaultCovariance = new double[]
                    {
                        0.0001, 0, 0, 0, 0, 0,
                        0, 0.0001, 0, 0, 0, 0,
                        0, 0, 0.0001, 0, 0, 0,
                        0, 0, 0, 0.0001, 0, 0,
                        0, 0, 0, 0, 0.0001, 0,
                        0, 0, 0, 0, 0, 0.0001
                    };
        public void UpdateOdom(float duration)
        {
            var angularVelocityLeft = -LeftMotor.GetCurrentVelocity() * Mathf.Deg2Rad;
            var angularVelocityRight = -RightMotor.GetCurrentVelocity() * Mathf.Deg2Rad;
            var yaw = ImuOrientation.eulerAngles.y * Mathf.Deg2Rad;
            CalculateOdometry(duration, angularVelocityLeft, angularVelocityRight, yaw);
            var time = SimulatorManager.Instance.CurrentTime;
            long nanosec = (long)(time * 1e9);

            odom = new Bridge.Ros2.Ros.Odometry
            {
                header = new Bridge.Ros2.Ros.Header()
                {
                    stamp = new Bridge.Ros2.Ros.Time()
                    {
                        secs = (int)(nanosec / 1000000000),
                        nsecs = (uint)(nanosec % 1000000000),
                    },
                    frame_id = Frame,
                },
                child_frame_id = OdometryChildFrame,
                pose = {
                    pose= {
                            position = {
                                x =  _odomPose.x,
                                y = -_odomPose.y,
                                z = -_odomPose.z,
                            },
                            orientation = {
                                x = -ImuOrientation.z,
                                y = ImuOrientation.x,
                                z = -ImuOrientation.y,
                                w = ImuOrientation.w
                            }

                    },
                    covariance = defaultCovariance
                },
                twist = {
                    twist = {
                        angular = {
                            x = 0,
                            y = 0,
                            z = -_odomVelocity.y,
                        },
                        linear = {
                            x = _odomVelocity.x,
                            y = 0,
                            z = 0
                        }
                    },
                    covariance = defaultCovariance

                }
            };
        }

        private Vector3 ImuInitialRotation = Vector3.zero;
        private Quaternion ImuOrientation = Quaternion.identity;
        private Vector3 ImuAngularVelocity = Vector3.zero;
        private Vector3 ImuLinearAcceleration = Vector3.zero;

        private Vector3 PreviousImuPosition = Vector3.zero;
        private Vector3 PreviousImuRotation = Vector3.zero;
        private Vector3 PreviousLinearVelocity = Vector3.zero;

        // IVehicleInputs
        public float SteerInput { get; private set; } = 0.0f;
        public float AccelInput { get; private set; } = 0.0f;
        public float BrakeInput { get; private set; } = 0.0f;
    }
}
