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

using System;
using UnityEngine;
using UnityEditor;
using Oculus.Interaction.HandGrab;
using System.Collections.Generic;

using Object = UnityEngine.Object;

namespace Oculus.Interaction.Editor.QuickActions
{
    internal class Template
    {
        /// <summary>
        /// The instantiated prefab will be given this name
        /// </summary>
        public readonly string DisplayName;

        /// <summary>
        /// The GUID of the prefab asset
        /// </summary>
        public readonly string AssetGUID;

        /// <param name="displayName">The instantiated GameObject will be given this name.
        /// Does not need to correspond to the prefab asset name.</param>
        /// <param name="assetGUID">The GUID of the prefab asset.</param>
        public Template(string displayName, string assetGUID)
        {
            DisplayName = displayName;
            AssetGUID = assetGUID;
        }
    }

    internal static class Templates
    {
        public static event Action<Template, GameObject> WhenObjectCreated = delegate { };

        #region Interactables

        public static readonly Template RayCanvasInteractable =
            new Template(
                "ISDK_RayInteraction",
                "8369d93f7b6b99742bbea0649a41b7b1");

        public static readonly Template PokeCanvasInteractable =
            new Template(
                "ISDK_PokeInteraction",
                "4db41829582c7d24f80ee9603868dd67");

        public static readonly Template HandGrabInteractable =
            new Template(
                "ISDK_HandGrabInteraction",
                "6ee61821e0d5b094a8d732834b365b21");

        public static readonly Template DistanceGrabInteractable_ToHand =
            new Template(
                "ISDK_DistanceHandGrabInteraction",
                "e9f3427813a446742ad647fe9aa1547c"
                );

        public static readonly Template DistanceGrabInteractable_HandTo =
            new Template(
                "ISDK_DistanceHandGrabInteraction",
                "86bc3e593d22eec429ae3910a86ffb03"
                );

        public static readonly Template DistanceGrabInteractable_AnchorAtHand =
            new Template(
                "ISDK_DistanceHandGrabInteraction",
                "4c91b23cd5294f84eb9ab4b61a183f63"
                );

        public static readonly Template DistanceGrabInteractable_SnapZone =
            new Template(
                "ISDK_DistanceGrabSnapZone",
                "b63e95c77d701bd44a848c316f6e9fa9"
                );

        #endregion Interactables

        #region Interactors

        public static readonly Template HandGrabInteractor =
            new Template(
                "HandGrabInteractor",
                "f0a90b2d303e7744fa8c9d3c6e2418a4");

        public static readonly Template HandPokeInteractor =
            new Template(
                "PokeInteractor",
                "abe5a2b766edc96438786a6785a2f74b");

        public static readonly Template HandRayInteractor =
            new Template(
                "RayInteractor",
                "a6df867c95b07224498cb3ea2d410ce5");

        public static readonly Template DistanceHandGrabInteractor =
            new Template(
                "DistanceHandGrabInteractor",
                "7ea5ce61c81c5ba40a697e2642e80c83");

        public static readonly Template ControllerPokeInteractor =
            new Template(
                "PokeInteractor",
                "ef9bd966f1a997b4cb9eef15b0620b24");

        public static readonly Template ControllerRayInteractor =
            new Template(
                "RayInteractor",
                "074f70ff54d0c6d489aaeba17f4bc66d");

        public static readonly Template ControllerGrabInteractor =
            new Template(
                "GrabInteractor",
                "069b845e75891f04bb2e512a8ebf3b78");

        public static readonly Template ControllerDistanceGrabInteractor =
            new Template(
                "DistanceGrabInteractor",
                "d9ef0d4c78b4bfd409cb884dfe1524d6");

        private static Dictionary<Type, Template> _handInteractorTemplates = new()
        {
            [typeof(HandGrabInteractor)] = HandGrabInteractor,
            [typeof(PokeInteractor)] = HandPokeInteractor,
            [typeof(RayInteractor)] = HandRayInteractor,
            [typeof(DistanceHandGrabInteractor)] = DistanceHandGrabInteractor,

        };

        private static Dictionary<Type, Template> _controllerInteractorTemplates = new()
        {
            [typeof(GrabInteractor)] = ControllerGrabInteractor,
            [typeof(PokeInteractor)] = ControllerPokeInteractor,
            [typeof(RayInteractor)] = ControllerRayInteractor,
            [typeof(DistanceGrabInteractor)] = ControllerDistanceGrabInteractor,
        };

        /// <summary>
        /// Gets the <see cref="Template"/> for a Hand interactor type
        /// </summary>
        public static bool TryGetHandInteractorTemplate(Type type, out Template template)
        {
            return _handInteractorTemplates.TryGetValue(type, out template);
        }

        /// <summary>
        /// Gets the <see cref="Template"/> for a Controller interactor type
        /// </summary>
        public static bool TryGetControllerInteractorTemplate(Type type, out Template template)
        {
            return _controllerInteractorTemplates.TryGetValue(type, out template);
        }

        #endregion Interactors

        /// <summary>
        /// Add an interactable prefab to a GameObject and register it in the Undo stack.
        /// Also registers with the cleanup list, to be optionally removed
        /// when the user cancels out of the wizard.
        /// </summary>
        /// <param name="parent">The Transform the prefab will be instantiated under</param>
        /// <param name="template">The <see cref="Template"/>to be instantiated</param>
        /// <returns>The GameObject at the root of the prefab.</returns>
        public static GameObject CreateFromTemplate(Transform parent, Template template)
        {
            GameObject result = Object.Instantiate(AssetDatabase.LoadMainAssetAtPath(
                AssetDatabase.GUIDToAssetPath(template.AssetGUID))) as GameObject;
            result.name = template.DisplayName;
            result.transform.SetParent(parent?.transform, false);
            Undo.RegisterCreatedObjectUndo(result, "Add " + template.DisplayName);
            WhenObjectCreated.Invoke(template, result);
            return result;
        }
    }
}
