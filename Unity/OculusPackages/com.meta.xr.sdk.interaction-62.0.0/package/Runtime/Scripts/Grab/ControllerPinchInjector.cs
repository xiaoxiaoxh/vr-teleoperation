/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the "License");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

using Oculus.Interaction.Input;
using UnityEngine;

namespace Oculus.Interaction.GrabAPI
{
    /// <summary>
    /// Use this component with a HandGrabAPI so it uses the Controller
    /// trigger (index finger) and grip (middle finger) to drive the FingerAPIs.
    /// </summary>
    public class ControllerPinchInjector : MonoBehaviour
    {
        [SerializeField]
        private HandGrabAPI _handGrabAPI;

        [SerializeField, Interface(typeof(IController))]
        private UnityEngine.Object _controller;
        private IController Controller { get; set; }

        protected bool _started;

        protected virtual void Awake()
        {
            Controller = _controller as IController;
        }

        protected virtual void Start()
        {
            this.BeginStart(ref _started);
            this.AssertField(_handGrabAPI, nameof(_handGrabAPI));
            this.AssertField(Controller, nameof(Controller));

            _handGrabAPI.InjectOptionalFingerPinchAPI(new ControllerPinchAPI(Controller));
            _handGrabAPI.InjectOptionalFingerGrabAPI(new ControllerPinchAPI(Controller));

            this.EndStart(ref _started);
        }

        #region Inject

        public void InjectAll(HandGrabAPI handGrabAPI, IController controller)
        {
            InjectHandGrabAPI(handGrabAPI);
            InjectController(controller);
        }

        public void InjectHandGrabAPI(HandGrabAPI handGrabAPI)
        {
            _handGrabAPI = handGrabAPI;
        }

        public void InjectController(IController controller)
        {
            _controller = controller as UnityEngine.Object;
            Controller = controller;
        }

        #endregion

        private class ControllerPinchAPI : IFingerAPI
        {
            private IController _controller;

            private float _triggerStrength = 0f;
            private float _gripStrength = 0f;
            private bool _triggerDown = false;
            private bool _gripDown = false;

            private bool _prevTriggerDown;
            private bool _prevGripDown;

            private Pose _indexPinchPose = Pose.identity;
            private Pose _middlePinchPose = Pose.identity;

            private Pose _pinchPose = Pose.identity;

            public ControllerPinchAPI(IController controller)
            {
                _controller = controller;
            }

            public float GetFingerGrabScore(HandFinger finger)
            {
                switch (finger)
                {
                    case HandFinger.Thumb: return _triggerStrength;
                    case HandFinger.Index: return _triggerStrength;
                    case HandFinger.Middle:
                    case HandFinger.Ring:
                    case HandFinger.Pinky: return _gripStrength;
                    default: return 0f;
                }
            }

            public bool GetFingerIsGrabbing(HandFinger finger)
            {
                switch (finger)
                {
                    case HandFinger.Thumb: return _triggerDown;
                    case HandFinger.Index: return _triggerDown;
                    case HandFinger.Middle:
                    case HandFinger.Ring:
                    case HandFinger.Pinky: return _gripDown;
                    default: return false;
                }
            }

            public bool GetFingerIsGrabbingChanged(HandFinger finger, bool targetPinchState)
            {
                switch (finger)
                {
                    case HandFinger.Thumb:
                        return (_triggerDown == targetPinchState
                            && _triggerDown != _prevTriggerDown)
                            || (_gripDown == targetPinchState
                            && _gripDown != _prevGripDown);
                    case HandFinger.Index:
                        return _triggerDown == targetPinchState
                            && _triggerDown != _prevTriggerDown;
                    case HandFinger.Middle:
                    case HandFinger.Ring:
                    case HandFinger.Pinky:
                        return _gripDown == targetPinchState
                            && _gripDown != _prevGripDown;
                    default: return false;
                }
            }

            public Vector3 GetWristOffsetLocal()
            {
                return _pinchPose.position;
            }

            public void Update(IHand hand)
            {
                ControllerInput input = _controller.ControllerInput;

                _prevGripDown = _gripDown;
                _prevTriggerDown = _triggerDown;

                _triggerStrength = input.Trigger;
                _triggerDown = input.TriggerButton;
                _gripStrength = input.Grip;
                _gripDown = input.GripButton;

                hand.GetJointPoseFromWrist(HandJointId.HandIndexTip, out _indexPinchPose);
                hand.GetJointPoseFromWrist(HandJointId.HandMiddleTip, out _middlePinchPose);
                hand.GetJointPoseFromWrist(HandJointId.HandThumbTip, out Pose thumbPose);

                PoseUtils.Lerp(ref _indexPinchPose, thumbPose, 0.5f);
                PoseUtils.Lerp(ref _middlePinchPose, thumbPose, 0.5f);


                float combinedStrength = _triggerStrength + _gripStrength;
                float t = combinedStrength > 0f ? _gripStrength / combinedStrength : 0.5f;
                PoseUtils.Lerp(_indexPinchPose, _middlePinchPose, t, ref _pinchPose);

            }
        }
    }
}
