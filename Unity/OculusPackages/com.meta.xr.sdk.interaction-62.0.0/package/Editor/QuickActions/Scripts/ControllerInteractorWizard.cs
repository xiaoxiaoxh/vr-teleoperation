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

using UnityEditor;
using UnityEngine;
using System;
using System.Linq;
using System.Collections.Generic;
using Oculus.Interaction.Input;

namespace Oculus.Interaction.Editor.QuickActions
{
    using static InteractorUtils;

    internal class ControllerInteractorWizard : QuickActionsWizard
    {
        private const string MENU_NAME = MENU_FOLDER +
            "Add Interactor(s) To Controller";

        [MenuItem(MENU_NAME)]
        private static void OpenWizard()
        {
            ShowWindow<ControllerInteractorWizard>(Selection.gameObjects[0]);
        }

        [MenuItem(MENU_NAME, true)]
        static bool Validate()
        {
            return Selection.gameObjects.Length == 1 && GetBaseComponent<Controller>(
                Selection.gameObjects[0].transform, false, true) != null;
        }

        #region Fields

        [SerializeField]
        [WizardSetting]
        [Tooltip("The selected interactor types will be added to the controller.")]
        private InteractorTypes _interactorTypes = InteractorTypes.All;

        [SerializeField]
        [WizardDependency(ReadOnly = true)]
        [Tooltip("New interactor(s) will be added to this controller.")]
        private Controller _controller;

        [SerializeField]
        [WizardDependency(FindMethod = nameof(FindHmd))]
        [Tooltip("The player HMD instance within the scene.")]
        private Hmd _hmd;

        [SerializeField]
        [WizardDependency, ChangeCheck(nameof(OnTransformChanged))]
        [InspectorName("Parent Transform")]
        [Tooltip("New interactor(s) will be instantiated under this transform.")]
        private Transform _transform;

        [SerializeField]
        [WizardDependency(Category = Category.Optional)]
        [Tooltip("If provided, interactor(s) will be added to this InteractorGroup.")]
        private InteractorGroup _interactorGroup;

        #endregion Fields

        private void OnTransformChanged()
        {
            _interactorGroup = _transform == null ? null :
                _transform.GetComponent<InteractorGroup>();
        }

        private static void RemoveDuplicateInteractors(Transform root,
            ref InteractorTypes interactorType)
        {
            interactorType &= ~GetExistingControllerInteractors(root);
        }

        private void FindHmd()
        {
            _hmd = GetHmd();
        }

        protected override void Create()
        {
            AddInteractorsToController(_interactorTypes,
                _controller,_hmd, _transform, _interactorGroup);
        }

        protected override void InitializeFieldsExtra()
        {
            base.InitializeFieldsExtra();

            // If any selected objects contains a child Controller
            if (_controller = GetBaseComponent<Controller>(Target.transform, false, false))
            {
                _interactorGroup = _controller.GetComponentInChildren<InteractorGroup>();
            }
            else if (_interactorGroup = Target.GetComponentInParent<InteractorGroup>())
            {
                _controller = GetBaseComponent<Controller>(Target.transform, false, true);
            }
            else
            {
                _controller = GetBaseComponent<Controller>(Target.transform, false, true);
                _interactorGroup = _controller?.GetComponentInChildren<InteractorGroup>();
            }

            if (_interactorGroup != null)
            {
                _transform = _interactorGroup.transform;
            }
            else if (_controller != null)
            {
                _transform = FindInteractorsTransform(_controller.transform);
            }

            if (_transform != null)
            {
                _interactorTypes &= ~GetExistingControllerInteractors(_transform);
            }
        }

        protected override IEnumerable<MessageData> GetMessages()
        {
            IEnumerable<MessageData> messages = Enumerable.Empty<MessageData>();
            if (_transform != null)
            {
                foreach (InteractorTypes value in Enum.GetValues(typeof(InteractorTypes)))
                {
                    InteractorTypes existingTypes = GetExistingControllerInteractors(_transform);
                    if (_interactorTypes.HasFlag(value) &&
                        existingTypes.HasFlag(value) &&
                        TryGetTypeForControllerInteractor(value, out Type type))
                    {
                        messages = messages.Append(new MessageData(MessageType.Error,
                        $"There is already a {type.Name} added to this Controller.",
                        new ButtonData("Do Not Add", () => _interactorTypes &= ~value)));
                    }
                }
            }
            if ((_interactorTypes & InteractorTypes.All) == 0)
            {
                messages = messages.Append(new MessageData(MessageType.Error,
                    "No interactor types selected."));
            }
            if (_controller != null && _transform != null &&
                !_transform.IsChildOf(_controller.transform))
            {
                messages = messages.Append(new MessageData(MessageType.Warning,
                $"It recommended to add interactors under their associated Controller."));
            }
            return messages;
        }
    }
}
