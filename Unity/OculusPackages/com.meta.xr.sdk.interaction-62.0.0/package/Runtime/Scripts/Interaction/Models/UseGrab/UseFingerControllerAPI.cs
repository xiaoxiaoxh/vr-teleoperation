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

namespace Oculus.Interaction
{
    public class UseFingerControllerAPI : MonoBehaviour, IFingerUseAPI
    {
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
            this.AssertField(Controller, nameof(Controller));
            this.EndStart(ref _started);
        }

        public float GetFingerUseStrength(HandFinger finger)
        {
            switch (finger)
            {
                case HandFinger.Index: return Controller.ControllerInput.Trigger;
                case HandFinger.Middle: return Controller.ControllerInput.Grip;
                case HandFinger.Thumb: return Mathf.Max(Controller.ControllerInput.Trigger, Controller.ControllerInput.Grip);
                default: return 0f;
            }
        }

        #region Inject
        public void InjectAllUseFingerRawPinchAPI(IController controller)
        {
            InjectController(controller);
        }

        public void InjectController(IController controller)
        {
            _controller = controller as UnityEngine.Object;
            Controller = controller;
        }
        #endregion
    }
}
