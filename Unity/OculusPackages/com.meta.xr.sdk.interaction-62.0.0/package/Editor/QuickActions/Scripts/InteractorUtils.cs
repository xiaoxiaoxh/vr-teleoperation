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
using System.Linq;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using Oculus.Interaction.Input;
using Oculus.Interaction.HandGrab;

using Object = UnityEngine.Object;
using UnityEditor;

namespace Oculus.Interaction.Editor.QuickActions
{
    [Flags]
    internal enum InteractorTypes
    {
        None = 0,
        Poke = 1 << 0,
        Grab = 1 << 1,
        Ray = 1 << 2,
        DistanceGrab = 1 << 3,
        All = (1 << 4) - 1,
    }

    [Flags]
    internal enum DeviceTypes
    {
        None = 0,
        Hands = 1 << 0,
        Controllers = 1 << 1,
        All = (1 << 2) - 1,
    }

    internal static class InteractorUtils
    {
        public const string HAND_INTERACTOR_PARENT_NAME = "HandInteractors";
        public const string CONTROLLER_INTERACTOR_PARENT_NAME = "ControllerInteractors";
        public const string CONTROLLERHAND_INTERACTOR_PARENT_NAME = "ControllerHandInteractors";

        private static readonly Dictionary<InteractorTypes, Type> _handTypeLookup =
            new Dictionary<InteractorTypes, Type>()
            {
                [InteractorTypes.Poke] = typeof(PokeInteractor),
                [InteractorTypes.Grab] = typeof(HandGrabInteractor),
                [InteractorTypes.Ray] = typeof(RayInteractor),
                [InteractorTypes.DistanceGrab] = typeof(DistanceHandGrabInteractor),
            };

        private static readonly Dictionary<InteractorTypes, Type> _controllerTypeLookup =
            new Dictionary<InteractorTypes, Type>()
            {
                [InteractorTypes.Poke] = typeof(PokeInteractor),
                [InteractorTypes.Grab] = typeof(GrabInteractor),
                [InteractorTypes.Ray] = typeof(RayInteractor),
                [InteractorTypes.DistanceGrab] = typeof(DistanceGrabInteractor),
            };

        /// <summary>
        /// Get the Type of a Hand interactor associated with an
        /// <see cref="InteractorTypes"/> value
        /// </summary>
        public static bool TryGetTypeForHandInteractor(
            InteractorTypes interactorType, out Type type)
        {
            return _handTypeLookup.TryGetValue(interactorType, out type);
        }

        /// <summary>
        /// Get the Type of a Controller interactor associated with an
        /// <see cref="InteractorTypes"/> value
        /// </summary>
        public static bool TryGetTypeForControllerInteractor(
            InteractorTypes interactorType, out Type type)
        {
            return _controllerTypeLookup.TryGetValue(interactorType, out type);
        }

        /// <summary>
        /// Find the default transform that serves as a parent to
        /// the ISDK interactors
        /// </summary>
        /// <param name="root"></param>
        /// <returns></returns>
        public static Transform FindInteractorsTransform(Transform root)
        {
            if (root == null)
            {
                return null;
            }

            foreach (Transform child in root.transform)
            {
                if (child.gameObject.name.StartsWith(HAND_INTERACTOR_PARENT_NAME) ||
                    child.gameObject.name.StartsWith(CONTROLLER_INTERACTOR_PARENT_NAME) ||
                    child.gameObject.name.StartsWith(CONTROLLERHAND_INTERACTOR_PARENT_NAME))
                {
                    return child;
                }
            }
            return null;
        }

        /// <summary>
        /// Get all the interactors in <see cref="InteractorGroup.Interactors"/>,
        /// using reflection to get the backing list due to the public property
        /// only being populated at runtime.
        /// </summary>
        /// <param name="group"></param>
        /// <returns>The <see cref="IInteractor"/>s within the group</returns>
        public static IEnumerable<IInteractor> GetInteractorsFromGroup(InteractorGroup group)
        {
            FieldInfo field = group.GetType().GetField("_interactors",
                BindingFlags.NonPublic | BindingFlags.Instance);
            var curValue = field.GetValue(group) as List<Object>;
            return curValue == null ? Enumerable.Empty<IInteractor>() : curValue
                .Where(o => o != null)
                .ToList().ConvertAll(o => o as IInteractor);
        }

        /// <summary>
        /// Get any Hand <see cref="InteractorTypes"/> that may
        /// already exist under a Transform.
        /// </summary>
        public static InteractorTypes GetExistingHandInteractors(Transform root)
        {
            InteractorTypes result = 0;
            foreach (InteractorTypes type in Enum.GetValues(typeof(InteractorTypes)))
            {
                if (TryGetTypeForHandInteractor(type, out Type iType) &&
                    root.GetComponentInChildren(iType, true) != null)
                {
                    result |= type;
                }
            }
            return result;
        }

        /// <summary>
        /// Get any Controller <see cref="InteractorTypes"/> that may
        /// already exist under a Transform.
        /// </summary>
        public static InteractorTypes GetExistingControllerInteractors(Transform root)
        {
            InteractorTypes result = 0;
            foreach (InteractorTypes type in Enum.GetValues(typeof(InteractorTypes)))
            {
                if (TryGetTypeForControllerInteractor(type, out Type iType) &&
                    root.GetComponentInChildren(iType, true) != null)
                {
                    result |= type;
                }
            }
            return result;
        }

        /// <summary>
        /// Verify that rig contains correct object structure for auto-adding
        /// interactors to Controllers
        /// </summary>
        public static bool CanAddControllerInteractorsToRig()
        {
            return GetHmd() != null && GetControllers()
                .Where(c => FindInteractorsTransform(c.transform)).Any();
        }

        /// <summary>
        /// Verify that rig contains correct object structure for auto-adding
        /// interactors to Hands
        /// </summary>
        public static bool CanAddHandInteractorsToRig()
        {
            return GetHmd() != null && GetHands()
                .Where(h => FindInteractorsTransform(h.transform)).Any();
        }

        /// <summary>
        /// Add a set of interactors to a rig within the provided devices.
        /// </summary>
        /// <param name="interactors">The interactors to add</param>
        /// <param name="devices">The devices to add the interactors to</param>
        /// <returns>A collection of any added objects</returns>
        public static IEnumerable<GameObject> AddInteractorsToRig(
            InteractorTypes interactors, DeviceTypes devices)
        {
            bool TryGetInteractorParent(Transform root, out Transform parent)
            {
                parent = FindInteractorsTransform(root);
                return parent != null;
            }

            List<GameObject> newInteractors = new List<GameObject>();

            if (devices.HasFlag(DeviceTypes.Hands) && CanAddHandInteractorsToRig())
            {
                foreach (var hand in GetHands())
                {
                    if (TryGetInteractorParent(hand.transform, out Transform parent))
                    {
                        var group = parent.GetComponent<InteractorGroup>();
                        var result = AddInteractorsToHand(
                            interactors, hand, GetHmd(), parent, group);
                        newInteractors.AddRange(result);
                    }
                }
            }

            if (devices.HasFlag(DeviceTypes.Controllers) && CanAddControllerInteractorsToRig())
            {
                foreach (var controller in GetControllers())
                {
                    if (TryGetInteractorParent(controller.transform, out Transform parent))
                    {
                        var group = parent.GetComponent<InteractorGroup>();
                        var result = AddInteractorsToController(
                            interactors, controller, GetHmd(), parent, group);
                        newInteractors.AddRange(result);
                    }
                }
            }

            return newInteractors;
        }

        /// <summary>
        /// Adds interactor(s) to a Hand
        /// </summary>
        /// <param name="types">The interactor types to add</param>
        /// <returns>A collection of the added interactors</returns>
        public static IEnumerable<GameObject> AddInteractorsToHand(InteractorTypes types,
            Hand hand, Hmd hmd, Transform parentTransform, InteractorGroup group = null)
        {
            List<GameObject> newInteractors = new List<GameObject>();
            foreach (InteractorTypes interactor in Enum.GetValues(typeof(InteractorTypes)))
            {
                if (types.HasFlag(interactor) &&
                    !GetExistingHandInteractors(parentTransform).HasFlag(interactor) &&
                    TryGetTypeForHandInteractor(interactor, out Type type) &&
                    Templates.TryGetHandInteractorTemplate(type, out var template))
                {
                    GameObject newInteractor = AddInteractor(template, hmd, parentTransform, group);
                    newInteractor.GetComponent<HandRef>().InjectHand(hand);
                    newInteractors.Add(newInteractor);
                }
            }
            return newInteractors;
        }

        /// <summary>
        /// Adds interactor(s) to a Controller
        /// </summary>
        /// <param name="types">The interactor types to add</param>
        /// <returns>A collection of the added interactors</returns>
        public static IEnumerable<GameObject> AddInteractorsToController(InteractorTypes types,
            Controller controller, Hmd hmd, Transform parentTransform, InteractorGroup group = null)
        {
            List<GameObject> newInteractors = new List<GameObject>();
            foreach (InteractorTypes interactor in Enum.GetValues(typeof(InteractorTypes)))
            {
                if (types.HasFlag(interactor) &&
                    !GetExistingControllerInteractors(parentTransform).HasFlag(interactor) &&
                    TryGetTypeForControllerInteractor(interactor, out Type type) &&
                    Templates.TryGetControllerInteractorTemplate(type, out var template))
                {
                    GameObject newInteractor = AddInteractor(template, hmd, parentTransform, group);
                    newInteractor.GetComponent<ControllerRef>().InjectController(controller);
                    newInteractors.Add(newInteractor);
                }
            }
            return newInteractors;
        }

        internal static GameObject AddInteractor(Template template, Hmd hmd,
            Transform parentTransform, InteractorGroup group = null)
        {
            var newInteractorGo = Templates.CreateFromTemplate(parentTransform, template);
            newInteractorGo.GetComponent<HmdRef>()?.InjectHmd(hmd);
            var newInteractor = newInteractorGo.GetComponent<IInteractor>();
            if (group != null)
            {
                var currentInteractors = GetInteractorsFromGroup(group);
                group.InjectInteractors(currentInteractors.Append(newInteractor).ToList());
                EditorUtility.SetDirty(group); // List will not persist if not set dirty
            }
            return newInteractorGo;
        }


        /// <summary>
        /// Find Hand components in the scene, ignoring derived types.
        /// </summary>
        public static IEnumerable<Hand> GetHands()
        {
            return Object.FindObjectsOfType<Hand>()
                .Where(h => h.GetType() == typeof(Hand));
        }

        /// <summary>
        /// Find Controller components in the scene, ignoring derived types.
        /// </summary>
        public static IEnumerable<Controller> GetControllers()
        {
            return Object.FindObjectsOfType<Controller>()
                .Where(h => h.GetType() == typeof(Controller));
        }

        /// <summary>
        /// Find the HMD component in the scene, ignoring derived types.
        /// </summary>
        public static Hmd GetHmd()
        {
            return Object.FindObjectsOfType<Hmd>()
                .Where(h => h.GetType() == typeof(Hmd))
                .FirstOrDefault();
        }

        /// <summary>
        /// GetComponent but only matches the base (non-derived) type.
        /// </summary>
        /// <typeparam name="TComponent"></typeparam>
        /// <param name="root">The root transform to search from</param>
        /// <param name="includeChildren">Search down the hierarchy</param>
        /// <param name="includeParents">Search up the hierarchy</param>
        /// <returns></returns>
        public static TComponent GetBaseComponent<TComponent>(Transform root,
            bool includeChildren = false, bool includeParents = false)
            where TComponent : Component
        {
            IEnumerable<TComponent> components =
                root.GetComponents<TComponent>();
            if (includeChildren)
            {
                components = components.Union(
                    root.GetComponentsInChildren<TComponent>());
            }
            if (includeParents)
            {
                components = components.Union(
                    root.GetComponentsInParent<TComponent>());
            }
            return components
                .Where(h => h.GetType() == typeof(TComponent))
                .FirstOrDefault();
        }
    }
}
