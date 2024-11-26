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
using UnityEditor;
using System.Reflection;
using Oculus.Interaction.Body.Input;
using System.Collections.Generic;
using UnityEngine.Assertions;

namespace Oculus.Interaction.Body.PoseDetection.Editor
{
    using static Oculus.Interaction.Body.PoseDetection.BodyPoseData;

    [CustomEditor(typeof(BodyPoseData))]
    public class BodyPoseDataEditor : UnityEditor.Editor
    {
        private BodyPoseData BodyPoseData => (BodyPoseData)target;

        public override void OnInspectorGUI()
        {
            var versionField = typeof(BodyPoseData).GetField("_serializedVersion",
                BindingFlags.Instance | BindingFlags.NonPublic);

            int currentVersion = (int)versionField.GetValue(BodyPoseData);
            bool needsDataUpgrade = currentVersion < DATA_VERSION;

            if (needsDataUpgrade)
            {
                using (new EditorGUILayout.HorizontalScope())
                {
                    EditorGUILayout.HelpBox("Serialized data version is outdated, " +
                        "and upgrading is highly recommended.", MessageType.Warning);

                    if (GUILayout.Button("Upgrade",
                        GUILayout.ExpandHeight(true)))
                    {
                        try
                        {
                            ApplyDataUpgrade(currentVersion);
                            versionField.SetValue(BodyPoseData, DATA_VERSION);
                            SaveAsset();
                        }
                        catch
                        {
                            EditorUtility.DisplayDialog("Upgrade Failed",
                                "Data version upgrade failed.", "Ok");
                        }
                    }
                }
                EditorGUILayout.Space();
            }

            base.OnInspectorGUI();
        }

        /// <summary>
        /// Applies the data version upgrade and updates the serialized version.
        /// Asset must subsequently be saved to AssetDatabase or changes will be lost.
        /// </summary>
        private void ApplyDataUpgrade(int serializedVersion)
        {
            if (serializedVersion < 1)
            {
                // Version < 1 had wrist and palm joints swapped vs OpenXR
                Assert.IsTrue(SwapWristAndPalmIds());
            }
        }

        private bool SwapWristAndPalmIds()
        {
            var jointDataField = typeof(BodyPoseData).GetField("_jointData",
                BindingFlags.Instance | BindingFlags.NonPublic);

            if (jointDataField == null)
            {
                return false;
            }

            var currentData = jointDataField.GetValue(BodyPoseData) as List<JointData>;
            List<JointData> upgradedData = new List<JointData>(currentData);

            IReadOnlyDictionary<BodyJointId, BodyJointId> wristPalmSwap =
                new Dictionary<BodyJointId, BodyJointId>()
                {
                    [BodyJointId.Body_LeftHandWrist] = BodyJointId.Body_LeftHandPalm,
                    [BodyJointId.Body_LeftHandPalm] = BodyJointId.Body_LeftHandWrist,
                    [BodyJointId.Body_RightHandWrist] = BodyJointId.Body_RightHandPalm,
                    [BodyJointId.Body_RightHandPalm] = BodyJointId.Body_RightHandWrist,
                };

            for (int i = 0; i < upgradedData.Count; ++i)
            {
                JointData jointData = upgradedData[i];
                if (wristPalmSwap.ContainsKey(jointData.JointId))
                {
                    jointData.JointId = wristPalmSwap[jointData.JointId];
                }
                if (wristPalmSwap.ContainsKey(jointData.ParentId))
                {
                    jointData.ParentId = wristPalmSwap[jointData.ParentId];
                }
                upgradedData[i] = jointData;
            }

            jointDataField.SetValue(BodyPoseData, new List<JointData>(upgradedData));
            SaveAsset();
            return true;
        }

        private void SaveAsset()
        {
            EditorUtility.SetDirty(BodyPoseData);
            AssetDatabase.SaveAssetIfDirty(BodyPoseData);
            AssetDatabase.Refresh();
        }
    }
}
