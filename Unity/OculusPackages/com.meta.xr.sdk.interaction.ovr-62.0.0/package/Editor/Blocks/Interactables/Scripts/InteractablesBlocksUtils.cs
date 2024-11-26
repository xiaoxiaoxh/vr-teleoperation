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
using Oculus.Interaction;
using Oculus.Interaction.Editor.QuickActions;
using Oculus.Interaction.Input;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{
    internal class TouchHandGrabBlockWizard : QuickActionsWizard
    {
        public static readonly Template TouchHandGrabInteractable =
            new Template(
                "ISDKBlock_TouchHandGrabInteraction",
                "99784e1a3225c9841bac1e563e0f92a2");
        
        public static readonly Template TouchHandGrabInteractor =
            new Template(
                "TouchHandGrabInteractor",
                "37318c26f22752d4c88f584585c490e5");
        
        [SerializeField]
        [WizardDependency(FindMethod = nameof(FindTransform), FixMethod = nameof(FixTransform))]
        private Transform _targetTransform;

        [SerializeField]
        [WizardDependency(FindMethod = nameof(FindRigidbody), FixMethod = nameof(FixRigidbody))]
        private Rigidbody _rigidbody;

        [SerializeField]
        [WizardDependency(FindMethod = nameof(FindCollider), FixMethod = nameof(FixCollider))]
        private MeshCollider _collider;
        
        [SerializeField]
        [WizardDependency(FindMethod = nameof(FindBoundCollider), FixMethod = nameof(FixBoundCollider))]
        private BoxCollider _boundsCollider;

        private void FindTransform()
        {
            _targetTransform = Target.GetComponent<Transform>();
        }

        private void FixTransform()
        {
            FindTransform();
        }

        private void FindRigidbody()
        {
            _rigidbody = Target.GetComponent<Rigidbody>();
        }

        private void FixRigidbody()
        {
            _rigidbody = AddComponent<Rigidbody>(Target);
            _rigidbody.useGravity = true;
            _rigidbody.isKinematic = false;
        }

        private void FindCollider()
        {
            _collider = Target.GetComponentInChildren<MeshCollider>();
        }

        private void FixCollider()
        {
            _collider = AddComponent<MeshCollider>(Target);
            _collider.convex = true;
            _collider.isTrigger = false;
        }
        
        private void FindBoundCollider()
        {
            _boundsCollider = Target.GetComponentInChildren<BoxCollider>();
        }

        private void FixBoundCollider()
        {
            _boundsCollider = AddComponent<BoxCollider>(Target);
            _boundsCollider.isTrigger = true;
            var meshFilter = Target.GetComponentInChildren<MeshFilter>();
            var mesh = meshFilter.sharedMesh;
            _boundsCollider.center = mesh.bounds.center;
            _boundsCollider.size = mesh.bounds.size;
        }

        protected override void Create()
        {
            GameObject obj = Templates.CreateFromTemplate(
                Target.transform, TouchHandGrabInteractable);

            Transform transform = obj.transform;
            transform.localPosition = Vector3.zero;
            transform.localScale = Vector3.one;
            transform.localRotation = Quaternion.identity;

            obj.GetComponent<TouchHandGrabInteractable>()
                .InjectAllTouchHandGrabInteractable(_boundsCollider, new List<Collider>() { _collider });
                
            obj.GetComponent<Grabbable>()
                .InjectOptionalTargetTransform(_targetTransform);

            obj.GetComponent<PhysicsGrabbable>()
                .InjectRigidbody(_rigidbody);

            AddInteractorsToRig(typeof(TouchHandGrabInteractor), TouchHandGrabInteractor);
        }
        
        private static void AddInteractorsToRig(Type expectedInteractorType, Template template)
        {
            bool TryGetInteractorParent(Transform root, out Transform parent)
            {
                parent = InteractorUtils.FindInteractorsTransform(root);
                return parent != null;
            }

            if (InteractorUtils.CanAddHandInteractorsToRig())
            {
                foreach (var hand in InteractorUtils.GetHands())
                {
                    if (TryGetInteractorParent(hand.transform, out Transform parent)
                        && parent.GetComponentInChildren(expectedInteractorType, true) == null)
                    {
                        var group = parent.GetComponent<InteractorGroup>();
                        AddInteractorsToHand(template, hand, InteractorUtils.GetHmd(), parent, group);
                    }
                }
            }
        }
        
        private static void AddInteractorsToHand(Template template,
            Hand hand, Hmd hmd, Transform parentTransform, InteractorGroup group = null)
        {
            var newInteractor = InteractorUtils.AddInteractor(template, hmd, parentTransform, group);
            newInteractor.GetComponent<HandRef>().InjectHand(hand);

            var syntheticHandBlockData = Meta.XR.BuildingBlocks.Editor.Utils.GetBlockData(BlockDataIds.SyntheticHands);
            var syntheticHandBlocks = syntheticHandBlockData.GetBlocks();
            foreach (var syntheticHandBlock in syntheticHandBlocks)
            {
                var syntheticHand = syntheticHandBlock.GetComponent<SyntheticHand>();
                if (syntheticHand.Handedness == hand.Handedness)
                {
                    newInteractor.GetComponent<TouchHandGrabInteractorVisual>().InjectSyntheticHand(syntheticHand);
                }
            }
        }
    }
}
