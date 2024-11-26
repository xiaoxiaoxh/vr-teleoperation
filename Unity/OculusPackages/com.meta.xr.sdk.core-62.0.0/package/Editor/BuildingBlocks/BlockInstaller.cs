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
using UnityEditor;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{
    [InitializeOnLoad]
    internal static class BlockInstaller
    {
        private const string SessionQueueKey = "BlockInstaller.SessionQueueKey";

        private static float _lastCheckTime;
        private static readonly float StartTime;
        private const float CheckIntervalInS = 1f;
        private const float MaxTimeInS = 180f;

        static BlockInstaller()
        {
            if (!HasInstallationRequest())
            {
                return;
            }

            StartTime = Time.realtimeSinceStartup;

            EditorApplication.update -= CheckPackageInstallation;
            EditorApplication.update += CheckPackageInstallation;
        }

        [Serializable]
        private struct InstallationRequest
        {
            public int gameObjectId;
            public string scriptableObjectPath;
            public string blockDataId;

            public bool HasGameObjectId => gameObjectId != default;
        }

        public static void RequestInstallation(BlockData blockData, GameObject selectedGameObject = null)
        {
            SessionQueue.Enqueue(new InstallationRequest
            {
                gameObjectId = selectedGameObject != null ? selectedGameObject.GetInstanceID() : default,
                scriptableObjectPath = AssetDatabase.GetAssetPath(blockData),
                blockDataId = blockData.Id
            }, SessionQueueKey);

            AssetDatabase.Refresh();
            EditorUtility.RequestScriptReload();

            EditorApplication.update -= CheckPackageInstallation;
            EditorApplication.update += CheckPackageInstallation;
        }

        private static bool HasInstallationRequest()
        {
            return SessionQueue.Count<InstallationRequest>(SessionQueueKey) > 0;
        }

        private static void CheckPackageInstallation()
        {
            if (!HasInstallationRequest())
            {
                StopChecking();
                return;
            }

            var shouldCheck = Time.realtimeSinceStartup - _lastCheckTime > CheckIntervalInS;
            if (!shouldCheck)
            {
                return;
            }

            _lastCheckTime = Time.realtimeSinceStartup;

            var request = SessionQueue.Dequeue<InstallationRequest>(SessionQueueKey);

            if (request == null)
            {
                return;
            }

            var timeout = Time.realtimeSinceStartup - StartTime > MaxTimeInS;
            if (timeout)
            {
                OVRTelemetry.Start(OVRTelemetryConstants.BB.MarkerId.InstallBlockData)
                    .SetResult(OVRPlugin.Qpl.ResultType.Fail)
                    .AddAnnotation(OVRTelemetryConstants.BB.AnnotationType.BlockId, request.Value.blockDataId)
                    .AddAnnotationIfNotNullOrEmpty(OVRTelemetryConstants.BB.AnnotationType.Error, "timeout")
                    .Send();

                StopChecking();
                return;
            }

            var selectedGameObject = request.Value.HasGameObjectId
                ? EditorUtility.InstanceIDToObject(request.Value.gameObjectId) as GameObject
                : null;
            var blockData = AssetDatabase.LoadAssetAtPath<BlockData>(request.Value.scriptableObjectPath);

            Utils.RefreshList(true);

            if (blockData.HasMissingDependencies)
            {
                return;
            }

            InstallBlock(blockData, selectedGameObject);
        }

        private static void StopChecking()
        {
            SessionQueue.Clear(SessionQueueKey);
            EditorApplication.update -= CheckPackageInstallation;
        }

        private static void InstallBlock(BlockData blockData, GameObject selectedGameObject)
        {
            blockData.InstallWithDependenciesAndCommit(selectedGameObject);
            var windows = Resources.FindObjectsOfTypeAll<BuildingBlocksWindow>();
            if (windows.Length > 0)
            {
                windows[0].RefreshBlockList();
            }
        }
    }
}
