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
using System.Collections.Generic;
using Meta.XR.BuildingBlocks.Editor;
using UnityEditor;
using UnityEngine;

namespace Oculus.Interaction.Editor.BuildingBlocks
{
    public class OVRSyntheticHandsCollidersBlockData : BlockData
    {
        protected override bool UsesPrefab => false;

        protected override List<GameObject> InstallRoutine()
        {
            var syntheticHandBlockData = Meta.XR.BuildingBlocks.Editor.Utils.GetBlockData(BlockDataIds.SyntheticHandBlockData);
            var blocks = syntheticHandBlockData.GetBlocks();
            var colliders = new List<GameObject>();
            foreach (var block in blocks)
            {
                var collider = InstantiateHand(block.gameObject);
                colliders.Add(collider);
            }

            return colliders;
        }

        private GameObject InstantiateHand(GameObject hand)
        {
            var syntheticHand = hand.GetComponent<SyntheticHand>();
            var handedness = syntheticHand.Handedness;
            var visual = syntheticHand.GetComponentInChildren<HandVisual>();
            var visualGameObject = visual.gameObject;
            var capsule = visualGameObject.GetComponent<HandPhysicsCapsules>() ??
                          visualGameObject.AddComponent<HandPhysicsCapsules>();
            capsule.InjectAllOVRHandPhysicsCapsules(syntheticHand, false, 0);
            capsule.InjectMask(HandFingerJointFlags.Index1
                               | HandFingerJointFlags.Index2
                               | HandFingerJointFlags.Index3
                               | HandFingerJointFlags.IndexTip
                               | HandFingerJointFlags.Thumb0
                               | HandFingerJointFlags.Thumb1
                               | HandFingerJointFlags.Thumb2
                               | HandFingerJointFlags.Thumb3
                               | HandFingerJointFlags.ThumbTip
                               | HandFingerJointFlags.Middle1
                               | HandFingerJointFlags.Middle2
                               | HandFingerJointFlags.Middle3
                               | HandFingerJointFlags.MiddleTip
                               | HandFingerJointFlags.HandMaxSkinnable
                               );
            var jointRadiusFeature = hand.transform.parent.gameObject.GetComponentInChildren<JointsRadiusFeature>();
            capsule.InjectJointsRadiusFeature(jointRadiusFeature);
            BlocksUtils.UpdateForAutoWiring(hand);
            BlocksUtils.UpdateForAutoWiring(visualGameObject);
            return visualGameObject;
        }
    }
}
