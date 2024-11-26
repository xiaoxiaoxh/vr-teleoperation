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
using UnityEditor;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{
    public class ControllerTrackingBlockData : BlockData
    {
        protected override List<GameObject> InstallRoutine()
        {
            var cameraRigBB = Utils.GetBlocksWithType<OVRCameraRig>().First();

            var leftController = Instantiate(Prefab, Vector3.zero, Quaternion.identity);
            leftController.SetActive(true);
            leftController.name = $"{Utils.BlockPublicTag} {BlockName} left";
            Undo.RegisterCreatedObjectUndo(leftController, "Create left controller.");
            Undo.SetTransformParent(leftController.transform, cameraRigBB.leftControllerAnchor, true, "Parent to camera rig.");
            leftController.GetComponent<OVRControllerHelper>().m_controller = OVRInput.Controller.LTouch;

            var rightController = Instantiate(Prefab, Vector3.zero, Quaternion.identity);
            rightController.SetActive(true);
            rightController.name = $"{Utils.BlockPublicTag}  {BlockName} right";
            Undo.RegisterCreatedObjectUndo(rightController, "Create right controller.");
            Undo.SetTransformParent(rightController.transform, cameraRigBB.rightControllerAnchor, true, "Parent to camera rig.");
            rightController.GetComponent<OVRControllerHelper>().m_controller = OVRInput.Controller.RTouch;

            return new List<GameObject> { leftController, rightController };
        }
    }
}
