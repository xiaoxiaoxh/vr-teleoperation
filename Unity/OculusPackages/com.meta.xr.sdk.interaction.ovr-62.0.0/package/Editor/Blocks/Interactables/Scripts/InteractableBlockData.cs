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
using System.Collections.Generic;
using System.Linq;
using Oculus.Interaction;
using Oculus.Interaction.Editor.QuickActions;
using Oculus.Interaction.HandGrab;
using UnityEditor;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{
    public class InteractableBlockData : BlockData
    {
        internal abstract class InteractableCreationDescriptionBase
        {
            public Type InteractableType;
            public Type InteractorType;
            
            public abstract IEnumerable<GameObject> Create(GameObject target);
        }
        
        internal class InteractableCreationDescription<TWizard> : InteractableCreationDescriptionBase
            where TWizard : QuickActionsWizard
        {
            public Func<GameObject, bool, Action<TWizard>, IEnumerable<GameObject>> CreationDelegate;
            public Action<TWizard> Injections;
            
            public override IEnumerable<GameObject> Create(GameObject target)
            {
                return CreationDelegate?.Invoke(target, false, Injections);
            }
        }
        
        private enum InteractableTypes
        {
            HandGrab,
            DistanceHandGrab,
            TouchHandGrab,
            DistanceHandGrabAnchorAtHand,
            DistanceHandGrabHandToInteractable
        }
        
        protected override bool UsesPrefab => false;

        [SerializeField]
        private InteractableTypes _type;
        
        internal InteractableCreationDescriptionBase CreationDescription
        {
            get
            {
                if(!InteractableCreationDescriptions.TryGetValue(_type, out var creationDescription))
                {
                    throw new InvalidOperationException(
                        $"Undefined behavior for type {_type}");
                }

                return creationDescription;
            }
        }

        private static readonly Dictionary<InteractableTypes, InteractableCreationDescriptionBase>
            InteractableCreationDescriptions = new Dictionary<InteractableTypes, InteractableCreationDescriptionBase>
            {
                { 
                    InteractableTypes.HandGrab, 
                    new InteractableCreationDescription<GrabWizard>()
                    {
                        InteractableType =  typeof(HandGrabInteractable),
                        InteractorType = typeof(HandGrabInteractor),
                        CreationDelegate = QuickActionsWizard.CreateWithDefaults
                    } 
                },
                { 
                    InteractableTypes.DistanceHandGrab, 
                    new InteractableCreationDescription<DistanceGrabWizard>()
                    {
                        InteractableType =  typeof(DistanceHandGrabInteractable),
                        InteractorType = typeof(DistanceHandGrabInteractor),
                        CreationDelegate = QuickActionsWizard.CreateWithDefaults,
                        Injections = wizard => { wizard.InjectMode(DistanceGrabWizard.Mode.InteractableToHand); }
                    } 
                },
                { 
                    InteractableTypes.DistanceHandGrabAnchorAtHand, 
                    new InteractableCreationDescription<DistanceGrabWizard>()
                    {
                        InteractableType =  typeof(DistanceHandGrabInteractable),
                        InteractorType = typeof(DistanceHandGrabInteractor),
                        CreationDelegate = QuickActionsWizard.CreateWithDefaults,
                        Injections = wizard => { wizard.InjectMode(DistanceGrabWizard.Mode.AnchorAtHand); }
                    } 
                },
                { 
                    InteractableTypes.DistanceHandGrabHandToInteractable, 
                    new InteractableCreationDescription<DistanceGrabWizard>()
                    {
                        InteractableType =  typeof(DistanceHandGrabInteractable),
                        InteractorType = typeof(DistanceHandGrabInteractor),
                        CreationDelegate = QuickActionsWizard.CreateWithDefaults,
                        Injections = wizard => { wizard.InjectMode(DistanceGrabWizard.Mode.HandToInteractable); }
                    } 
                },
                { 
                    InteractableTypes.TouchHandGrab, 
                    new InteractableCreationDescription<TouchHandGrabBlockWizard>()
                    {
                        InteractableType =  typeof(TouchHandGrabInteractable),
                        InteractorType = typeof(TouchHandGrabInteractor),
                        CreationDelegate = QuickActionsWizard.CreateWithDefaults
                    } 
                }
            };

        protected override List<GameObject> InstallRoutine() => InstallRoutine(null);

        protected override List<GameObject> InstallRoutine(GameObject selectedObject)
        {
            if (selectedObject == null)
            {
                // Install Dummy Cube
                var cubeBlockData = Utils.GetBlockData(BlockDataIds.Cube);
                var cubeBlockObjects = cubeBlockData.Install();

                // Install on Dummy Cube
                selectedObject = cubeBlockObjects.First();
            }

            if (selectedObject == null)
            {
                throw new ArgumentNullException(nameof(selectedObject));
            }

            // Get corresponding wizard
            var creationDescription = CreationDescription;

            // Invoke Wizard with Default values
            var createdObjects = creationDescription.Create(selectedObject);
            var block = createdObjects.FirstOrDefault(obj => obj.GetComponent(creationDescription.InteractableType) != null);
            if (block != null)
            {
                block.name = $"{Utils.BlockPublicTag} {name}";
            }
            Undo.RegisterFullObjectHierarchyUndo(selectedObject, $"Installing {creationDescription.InteractableType} on {selectedObject.name}");
            return new List<GameObject>() { block };
        }
    }
}
