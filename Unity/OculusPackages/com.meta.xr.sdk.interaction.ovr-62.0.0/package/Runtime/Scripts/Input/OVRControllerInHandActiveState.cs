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
using static OVRInput;

namespace Oculus.Interaction.OVR
{
    public class OVRControllerInHandActiveState : MonoBehaviour, IActiveState
    {
        [SerializeField]
        private Hand _handType;
        public Hand HandType
        {
            get => _handType;
            set => _handType = value;
        }

        /// <summary>
        /// Determines if the ActiveState should be enabled or disabled when the hand is grabbing a controller
        /// </summary>
        [SerializeField]
        [Tooltip("Determines if the ActiveState should be enabled or disabled when the hand is grabbing a controller")]
        [HelpBox("Ensure you have enabled ConformingHandsToControllers or/and Concurrent Hands/Controller Support " +
            "in the OVRCameraRig.ControllerDrivenHandPosesType and that the OVRHand component ShowState is as permissive as this.",
            HelpBoxAttribute.MessageType.Info, InputDeviceShowState.ControllerNotInHand, ConditionalHideAttribute.DisplayMode.HideIfTrue)]
        private InputDeviceShowState _showState = InputDeviceShowState.ControllerNotInHand;
        public InputDeviceShowState ShowState
        {
            get => _showState;
            set => _showState = value;
        }

        public bool Active
        {
            get
            {
                ControllerInHandState state = GetControllerIsInHandState(_handType);
                switch (_showState)
                {
                    case InputDeviceShowState.Always:
                        return true;

                    case InputDeviceShowState.ControllerInHand:
                        return state == ControllerInHandState.ControllerInHand;

                    case InputDeviceShowState.ControllerInHandOrNoHand:
                        return state == ControllerInHandState.ControllerInHand
                            || state == ControllerInHandState.NoHand;

                    case InputDeviceShowState.ControllerNotInHand:
                        return state == ControllerInHandState.ControllerNotInHand;

                    case InputDeviceShowState.NoHand:
                        return state == ControllerInHandState.NoHand;
                    default:
                        return false;
                }
            }
        }
    }
}
