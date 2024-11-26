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

using UnityEngine;

namespace Oculus.Interaction.Input
{
    interface IUsage
    {
        void Apply(ControllerDataAsset controllerDataAsset, OVRInput.Controller controllerMask);
    }

    class UsageTouchMapping : IUsage
    {
        public ControllerButtonUsage Usage { get; }
        public OVRInput.Touch Touch { get; }

        public UsageTouchMapping(ControllerButtonUsage usage, OVRInput.Touch touch)
        {
            Usage = usage;
            Touch = touch;
        }

        public void Apply(ControllerDataAsset controllerDataAsset, OVRInput.Controller controllerMask)
        {
            bool value = OVRInput.Get(Touch, controllerMask);
            controllerDataAsset.Input.SetButton(Usage, value);
        }
    }

    class UsageButtonMapping : IUsage
    {
        public ControllerButtonUsage Usage { get; }
        public OVRInput.Button Button { get; }

        public UsageButtonMapping(ControllerButtonUsage usage, OVRInput.Button button)
        {
            Usage = usage;
            Button = button;
        }

        public void Apply(ControllerDataAsset controllerDataAsset, OVRInput.Controller controllerMask)
        {
            bool value = OVRInput.Get(Button, controllerMask);
            controllerDataAsset.Input.SetButton(Usage, value);
        }
    }

    class UsageAxis1DMapping : IUsage
    {
        public ControllerAxis1DUsage Usage { get; }
        public OVRInput.Axis1D Axis1D { get; }

        public UsageAxis1DMapping(ControllerAxis1DUsage usage, OVRInput.Axis1D axis1D)
        {
            Usage = usage;
            Axis1D = axis1D;
        }

        public void Apply(ControllerDataAsset controllerDataAsset, OVRInput.Controller controllerMask)
        {
            float value = OVRInput.Get(Axis1D, controllerMask);
            controllerDataAsset.Input.SetAxis1D(Usage, value);
        }
    }

    class UsageAxis2DMapping : IUsage
    {
        public ControllerAxis2DUsage Usage { get; }
        public OVRInput.Axis2D Axis2D { get; }

        public UsageAxis2DMapping(ControllerAxis2DUsage usage, OVRInput.Axis2D axis2D)
        {
            Usage = usage;
            Axis2D = axis2D;
        }

        public void Apply(ControllerDataAsset controllerDataAsset, OVRInput.Controller controllerMask)
        {
            Vector2 value = OVRInput.Get(Axis2D, controllerMask);
            controllerDataAsset.Input.SetAxis2D(Usage, value);
        }
    }

    /// <summary>
    /// Returns the Pointer Pose for the active controller model
    /// as found in the official prefabs.
    /// This point is usually located at the front tip of the controller.
    /// </summary>
    struct OVRPointerPoseSelector
    {
        private static readonly Pose[] QUEST1_POINTERS = new Pose[2]
        {
            new Pose(new Vector3(-0.00779999979f,-0.00410000002f,0.0375000015f),
                Quaternion.Euler(359.209534f, 6.45196056f, 6.95544577f)),
            new Pose(new Vector3(0.00779999979f,-0.00410000002f,0.0375000015f),
                Quaternion.Euler(359.209534f, 353.548035f, 353.044556f))
        };

        private static readonly Pose[] QUEST2_POINTERS = new Pose[2]
        {
            new Pose(new Vector3(0.00899999961f, -0.00321028521f, 0.030869998f),
                Quaternion.Euler(359.209534f, 6.45196056f, 6.95544577f)),
            new Pose(new Vector3(-0.00899999961f, -0.00321028521f, 0.030869998f),
                Quaternion.Euler(359.209534f, 353.548035f, 353.044556f))
        };

        public Pose LocalPointerPose { get; private set; }

        public OVRPointerPoseSelector(Handedness handedness)
        {
            OVRPlugin.SystemHeadset headset = OVRPlugin.GetSystemHeadsetType();
            switch (headset)
            {
                case OVRPlugin.SystemHeadset.Oculus_Quest_2:
                case OVRPlugin.SystemHeadset.Oculus_Link_Quest_2:
                    LocalPointerPose = QUEST2_POINTERS[(int)handedness];
                    break;
                default:
                    LocalPointerPose = QUEST1_POINTERS[(int)handedness];
                    break;
            }
        }
    }

    public class FromOVRControllerDataSource : DataSource<ControllerDataAsset>
    {
        [Header("OVR Data Source")]
        [SerializeField, Interface(typeof(IOVRCameraRigRef))]
        private UnityEngine.Object _cameraRigRef;
        public IOVRCameraRigRef CameraRigRef { get; private set; }

        [SerializeField]
        private bool _processLateUpdates = false;

        [Header("Shared Configuration")]
        [SerializeField]
        private Handedness _handedness;

        [SerializeField, Interface(typeof(ITrackingToWorldTransformer))]
        private UnityEngine.Object _trackingToWorldTransformer;
        private ITrackingToWorldTransformer TrackingToWorldTransformer;

        public bool ProcessLateUpdates
        {
            get
            {
                return _processLateUpdates;
            }
            set
            {
                _processLateUpdates = value;
            }
        }

        private readonly ControllerDataAsset _controllerDataAsset = new ControllerDataAsset();
        private OVRInput.Controller _ovrController;
        private ControllerDataSourceConfig _config;

        private OVRPointerPoseSelector _pointerPoseSelector;

        #region OVR Controller Mappings

        // Mappings from Unity XR CommonUsage to Oculus Button/Touch.
        private static readonly IUsage[] ControllerUsageMappings =
        {
            new UsageButtonMapping(ControllerButtonUsage.PrimaryButton, OVRInput.Button.One),
            new UsageTouchMapping(ControllerButtonUsage.PrimaryTouch, OVRInput.Touch.One),
            new UsageButtonMapping(ControllerButtonUsage.SecondaryButton, OVRInput.Button.Two),
            new UsageTouchMapping(ControllerButtonUsage.SecondaryTouch, OVRInput.Touch.Two),
            new UsageButtonMapping(ControllerButtonUsage.GripButton, OVRInput.Button.PrimaryHandTrigger),
            new UsageButtonMapping(ControllerButtonUsage.TriggerButton, OVRInput.Button.PrimaryIndexTrigger),
            new UsageButtonMapping(ControllerButtonUsage.MenuButton, OVRInput.Button.Start),
            new UsageButtonMapping(ControllerButtonUsage.Primary2DAxisClick, OVRInput.Button.PrimaryThumbstick),
            new UsageTouchMapping(ControllerButtonUsage.Primary2DAxisTouch, OVRInput.Touch.PrimaryThumbstick),
            new UsageTouchMapping(ControllerButtonUsage.Thumbrest, OVRInput.Touch.PrimaryThumbRest),

            new UsageAxis1DMapping(ControllerAxis1DUsage.Trigger, OVRInput.Axis1D.PrimaryIndexTrigger),
            new UsageAxis1DMapping(ControllerAxis1DUsage.Grip, OVRInput.Axis1D.PrimaryHandTrigger),

            new UsageAxis2DMapping(ControllerAxis2DUsage.Primary2DAxis, OVRInput.Axis2D.PrimaryThumbstick),
        };

        #endregion

        protected void Awake()
        {
            TrackingToWorldTransformer = _trackingToWorldTransformer as ITrackingToWorldTransformer;
            CameraRigRef = _cameraRigRef as IOVRCameraRigRef;

            UpdateConfig();
        }

        protected override void Start()
        {
            this.BeginStart(ref _started, () => base.Start());
            this.AssertField(CameraRigRef, nameof(CameraRigRef));
            this.AssertField(TrackingToWorldTransformer, nameof(TrackingToWorldTransformer));
            if (_handedness == Handedness.Left)
            {
                _ovrController = OVRInput.Controller.LTouch;
            }
            else
            {
                _ovrController = OVRInput.Controller.RTouch;
            }
            _pointerPoseSelector = new OVRPointerPoseSelector(_handedness);

            UpdateConfig();
            this.EndStart(ref _started);
        }

        protected override void OnEnable()
        {
            base.OnEnable();
            if (_started)
            {
                CameraRigRef.WhenInputDataDirtied += HandleInputDataDirtied;
            }
        }

        protected override void OnDisable()
        {
            if (_started)
            {
                CameraRigRef.WhenInputDataDirtied -= HandleInputDataDirtied;
            }

            base.OnDisable();

            MarkInputDataRequiresUpdate();
        }

        private void HandleInputDataDirtied(bool isLateUpdate)
        {
            if (isLateUpdate && !_processLateUpdates)
            {
                return;
            }
            MarkInputDataRequiresUpdate();
        }

        private ControllerDataSourceConfig Config
        {
            get
            {
                if (_config != null)
                {
                    return _config;
                }

                _config = new ControllerDataSourceConfig()
                {
                    Handedness = _handedness
                };

                return _config;
            }
        }

        private void UpdateConfig()
        {
            Config.Handedness = _handedness;
            Config.TrackingToWorldTransformer = TrackingToWorldTransformer;
        }

        protected override void UpdateData()
        {
            _controllerDataAsset.Config = Config;
            _controllerDataAsset.IsDataValid = true;
            _controllerDataAsset.IsConnected =
                (OVRInput.GetConnectedControllers() & _ovrController) > 0;
            if (!_controllerDataAsset.IsConnected && !this.isActiveAndEnabled)
            {
                // revert state fields to their defaults
                _controllerDataAsset.IsTracked = default;
                _controllerDataAsset.Input = default;
                _controllerDataAsset.RootPoseOrigin = default;
                return;
            }

            _controllerDataAsset.IsTracked = true;

            OVRInput.Handedness dominantHand = OVRInput.GetDominantHand();
            _controllerDataAsset.IsDominantHand =
                (dominantHand == OVRInput.Handedness.LeftHanded && _handedness == Handedness.Left)
                || (dominantHand == OVRInput.Handedness.RightHanded && _handedness == Handedness.Right);

            // Update button usages
            _controllerDataAsset.Input.Clear();

            OVRInput.Controller controllerMask = _ovrController;
            foreach (IUsage mapping in ControllerUsageMappings)
            {
                mapping.Apply(_controllerDataAsset, controllerMask);
            }

            // Update poses

            // Root pose, in tracking space.
            _controllerDataAsset.RootPose = new Pose(
                OVRInput.GetLocalControllerPosition(_ovrController),
                OVRInput.GetLocalControllerRotation(_ovrController));
            _controllerDataAsset.RootPoseOrigin = PoseOrigin.RawTrackedPose;

            // Convert controller pointer pose from local to tracking space.
            Matrix4x4 controllerModelToTracking = Matrix4x4.TRS(
                _controllerDataAsset.RootPose.position, _controllerDataAsset.RootPose.rotation,
                Vector3.one);
            _controllerDataAsset.PointerPose =
                new Pose(controllerModelToTracking.MultiplyPoint3x4(_pointerPoseSelector.LocalPointerPose.position),
                    _controllerDataAsset.RootPose.rotation * _pointerPoseSelector.LocalPointerPose.rotation);

            _controllerDataAsset.PointerPoseOrigin = PoseOrigin.RawTrackedPose;
        }

        protected override ControllerDataAsset DataAsset => _controllerDataAsset;

        #region Inject

        public void InjectAllFromOVRControllerDataSource(UpdateModeFlags updateMode, IDataSource updateAfter,
            Handedness handedness, ITrackingToWorldTransformer trackingToWorldTransformer)
        {
            base.InjectAllDataSource(updateMode, updateAfter);
            InjectHandedness(handedness);
            InjectTrackingToWorldTransformer(trackingToWorldTransformer);
        }

        public void InjectHandedness(Handedness handedness)
        {
            _handedness = handedness;
        }

        public void InjectTrackingToWorldTransformer(ITrackingToWorldTransformer trackingToWorldTransformer)
        {
            _trackingToWorldTransformer = trackingToWorldTransformer as UnityEngine.Object;
            TrackingToWorldTransformer = trackingToWorldTransformer;
        }

        #endregion
    }
}
