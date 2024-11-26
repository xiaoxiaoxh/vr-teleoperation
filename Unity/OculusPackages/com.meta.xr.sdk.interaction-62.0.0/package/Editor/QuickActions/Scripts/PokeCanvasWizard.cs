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
using UnityEngine.UI;
using System.Linq;
using System.Collections.Generic;
using UnityEngine.EventSystems;

namespace Oculus.Interaction.Editor.QuickActions
{
    internal class PokeCanvasWizard : QuickActionsWizard
    {
        private const string MENU_NAME = MENU_FOLDER +
            "Add Poke Interaction to Canvas";

        [MenuItem(MENU_NAME, priority = 201)]
        private static void OpenWizard()
        {
            ShowWindow<PokeCanvasWizard>(Selection.gameObjects[0]);
        }

        [MenuItem(MENU_NAME, true)]
        static bool Validate()
        {
            return Selection.gameObjects.Length == 1;
        }

        #region Fields

        [SerializeField]
        [DeviceType, WizardSetting]
        [InspectorName("Add Required Interactor(s)")]
        [Tooltip("The interactors required for the new interactable will be " +
            "added for the device types selected here, if not already present.")]
        private DeviceTypes _deviceTypes = DeviceTypes.All;

        [SerializeField]
        [Tooltip("The canvas to make Poke interactable.")]
        [WizardDependency(ReadOnly = true,
            FindMethod = nameof(FindCanvas),
            FixMethod = nameof(FixCanvas))]
        private Canvas _canvas;

        #endregion Fields

        private void FindCanvas()
        {
            _canvas = Target.GetComponent<Canvas>();
        }

        private void FixCanvas()
        {
            _canvas = AddComponent<Canvas>(Target);
            _canvas.renderMode = RenderMode.WorldSpace;
        }

        protected override void Create()
        {
            GameObject obj = Templates.CreateFromTemplate(
                Target.transform, Templates.PokeCanvasInteractable);

            // Reset RectTransform
            RectTransform rt = obj.GetComponent<RectTransform>();
            rt.localPosition = Vector3.zero;
            rt.localRotation = Quaternion.identity;
            rt.localScale = Vector3.one;
            rt.anchorMin = Vector2.zero;
            rt.anchorMax = Vector2.one;
            rt.pivot = new Vector2(0.5f, 0.5f);
            rt.sizeDelta = Vector2.zero;

            if (_canvas.GetComponent<GraphicRaycaster>() == null)
            {
                AddComponent<GraphicRaycaster>(_canvas.gameObject);
            }

            obj.GetComponent<PointableCanvas>()
                .InjectCanvas(_canvas);

            InteractorUtils.AddInteractorsToRig(
                InteractorTypes.Poke, _deviceTypes);
        }

        protected override IEnumerable<MessageData> GetMessages()
        {
            var result = Enumerable.Empty<MessageData>();

            result = result.Concat(Messages
                .MissingPointableCanvasModule<PokeInteractor>());

            if (Target.GetComponent<Canvas>() == null)
            {
                result = result.Append(new MessageData(MessageType.Error,
                    "The target object must have a Canvas attached."));
            }
            else if (_canvas != null && _canvas.renderMode != RenderMode.WorldSpace)
            {
                result = result.Append(new MessageData(MessageType.Error,
                    "The provided canvas must be in World space.",
                    new ButtonData("Fix", () => _canvas.renderMode = RenderMode.WorldSpace)));
            }
            return result;
        }
    }
}
