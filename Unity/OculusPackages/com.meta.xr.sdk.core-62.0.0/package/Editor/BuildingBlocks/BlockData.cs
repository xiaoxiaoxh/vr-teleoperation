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
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.SceneManagement;


namespace Meta.XR.BuildingBlocks.Editor
{
    public class BlockData : BlockBaseData
    {
        [SerializeField] internal GameObject prefab;
        public GameObject Prefab => prefab;
        protected virtual bool UsesPrefab => true;

        [SerializeField] internal List<string> externalBlockDependencies;
        [SerializeField] internal List<string> dependencies;

        public IReadOnlyList<BlockData> Dependencies
        {
            get
            {
                var dependencyIds =
                    (dependencies ?? Enumerable.Empty<string>()).Concat(externalBlockDependencies ??
                                                                        Enumerable.Empty<string>());

                return dependencyIds.Select(Utils.GetBlockData).ToList();
            }
        }

        [SerializeField] internal List<string> packageDependencies;

        public IReadOnlyList<string> PackageDependencies => packageDependencies ?? EmptyPackageList;

        private static readonly IReadOnlyList<string> EmptyPackageList = new List<string>();

        [Tooltip("Indicates whether only one instance of this block can be installed per scene.")]
        [SerializeField]
        internal bool isSingleton;

        [Tooltip("(Optional) Briefly write how this block should be used.")]
        [TextArea(5, 40)]
        [SerializeField]
        internal string usageInstructions;

        public string UsageInstructions => usageInstructions;
        public bool IsSingleton => isSingleton;


        internal override void AddToProject(GameObject selectedGameObject = null, Action onInstall = null)
        {
            using (new OVREditorUtils.UndoScope($"Install {Utils.BlockPublicTag} {BlockName}"))
            {
                InstallPackagesAndBlockData(selectedGameObject, onInstall);
            }
        }

        [ContextMenu("Install")]
        private void ContextMenuInstall()
        {
            AddToProject(null, null);
        }

        private void InstallPackagesAndBlockData(GameObject selectedGameObject = null, Action onInstall = null)
        {
            try
            {
                var (success, nInstalled) = InstallPackageDependencies();

                if (success && nInstalled > 0)
                {
                    BlockInstaller.RequestInstallation(this, selectedGameObject);
                }
                else
                {
                    InstallWithDependenciesAndCommit(selectedGameObject);
                    onInstall?.Invoke();
                }
            }
            catch (Exception e)
            {
                OVRTelemetry.Start(OVRTelemetryConstants.BB.MarkerId.InstallBlockData)
                    .SetResult(OVRPlugin.Qpl.ResultType.Fail)
                    .AddAnnotation(OVRTelemetryConstants.BB.AnnotationType.BlockId, Id)
                    .AddAnnotationIfNotNullOrEmpty(OVRTelemetryConstants.BB.AnnotationType.Error, e.Message)
                    .Send();
                throw;
            }
        }

        internal void InstallWithDependenciesAndCommit(GameObject selectedGameObject = null)
        {
            if (HasNonBuildingBlockCameraRig())
            {
                if (!EditorUtility.DisplayDialog("Confirmation",
                        $"You already have a scene setup with OVRCameraRig that may not be compatible with {Utils.BlocksPublicName}. Do you want to proceed?", "Yes", "No"))
                {
                    return;
                }
            }

            Exception installException = null;
            try
            {
                var installedObjects = InstallWithDependencies(selectedGameObject);

                SaveScene();
                FixSetupRules();

                EditorApplication.delayCall += () => { Utils.SelectBlocksInScene(installedObjects); };
            }
            catch (Exception e)
            {
                installException = e;
                throw;
            }
            finally
            {
                OVRTelemetry.Start(OVRTelemetryConstants.BB.MarkerId.InstallBlockData)
                    .SetResult(installException == null ? OVRPlugin.Qpl.ResultType.Success : OVRPlugin.Qpl.ResultType.Fail)
                    .AddAnnotation(OVRTelemetryConstants.BB.AnnotationType.BlockId, Id)
                    .AddAnnotationIfNotNullOrEmpty(OVRTelemetryConstants.BB.AnnotationType.Error, installException?.Message)
                    .Send();
            }
        }

        internal static bool HasNonBuildingBlockCameraRig()
        {
            var cameraRig = FindAnyObjectByType<OVRCameraRig>();
            return cameraRig != null && cameraRig.GetComponent<BuildingBlock>() == null;
        }

        internal static void FixSetupRules()
        {
            var buildTargetGroup = BuildPipeline.GetBuildTargetGroup(EditorUserBuildSettings.activeBuildTarget);
            UpdateTasks(buildTargetGroup, FixTasks);
        }

        private static void FixTasks(OVRConfigurationTaskProcessor processor)
        {
            OVRProjectSetup.FixTasks(processor.BuildTargetGroup, tasks => tasks
                .Where(task =>
                    !task.IsDone(processor.BuildTargetGroup)
                    && !task.IsIgnored(processor.BuildTargetGroup)
                    && task.Level.GetValue(processor.BuildTargetGroup) == OVRProjectSetup.TaskLevel.Required)
                .ToList(), blocking: false, onCompleted: AfterFixApply);
        }

        private static void UpdateTasks(BuildTargetGroup buildTargetGroup,
            Action<OVRConfigurationTaskProcessor> onCompleted = null)
        {
            OVRProjectSetup.UpdateTasks(buildTargetGroup, logMessages: OVRProjectSetup.LogMessages.Disabled,
                blocking: false, onCompleted: onCompleted);
        }

        private static void AfterFixApply(OVRConfigurationTaskProcessor processor)
        {
            AssetDatabase.SaveAssets();
            UpdateTasks(processor.BuildTargetGroup);
        }

        internal override bool CanBeAdded => !HasMissingDependencies && !IsSingletonAndAlreadyPresent;
        internal bool HasMissingDependencies
        {
            get
            {
                if (dependencies.Any(dependencyId => Utils.GetBlockData(dependencyId) == null))
                {
                    return true;
                }

                return PackageDependencies.All(OVRProjectSetupUtils.IsPackageInstalled)
                       && externalBlockDependencies.Any(dependencyId => Utils.GetBlockData(dependencyId) == null);
            }
        }
        private bool IsSingletonAndAlreadyPresent => IsSingleton && IsBlockPresentInScene(Id);

        private (bool, int) InstallPackageDependencies()
        {
            var nInstalled = PackageDependencies.Count(packageId => InstallPackage(packageId) == InstallPackageStatus.Installed);
            return (true, nInstalled);
        }

        private enum InstallPackageStatus
        {
            AlreadyInstalled,
            Installed
        }

        private InstallPackageStatus InstallPackage(string packageId)
        {
            if (OVRProjectSetupUtils.IsPackageInstalled(packageId))
            {
                return InstallPackageStatus.AlreadyInstalled;
            }

            var installed = OVRProjectSetupUtils.InstallPackage(packageId);

            if (!installed)
            {
                throw new InvalidOperationException(
                    $"Installation of package dependency {packageId} failed for block {BlockName}.");
            }

            return InstallPackageStatus.Installed;
        }

        internal List<GameObject> InstallWithDependencies(GameObject selectedGameObject = null)
        {
            if (IsSingletonAndAlreadyPresent)
            {
                throw new InvalidOperationException(
                    $"Block {BlockName} is a singleton and already present in the scene so it cannot be installed.");
            }

            if (HasMissingDependencies)
            {
                throw new InvalidOperationException($"A dependency of block {BlockName} is not present in the project.");
            }

            using (new OVREditorUtils.UndoScope($"Install {Utils.BlockPublicTag} {BlockName}"))
            {
                InstallDependencies(Dependencies, selectedGameObject);
                return Install(selectedGameObject);
            }
        }

        internal virtual List<GameObject> Install(GameObject selectedGameObject = null)
        {
            return InstallBlock<BuildingBlock>(selectedGameObject);
        }

        internal List<GameObject> InstallBlock<T>(GameObject selectedGameObject) where T : BuildingBlock
        {
            var spawnedObjects = InstallRoutine(selectedGameObject);

            foreach (var spawnedObject in spawnedObjects)
            {
                var block = Undo.AddComponent<T>(spawnedObject);
                block.blockId = Id;
                block.version = Version;
                while (UnityEditorInternal.ComponentUtility.MoveComponentUp(block))
                {
                }

                OVRTelemetry.Start(OVRTelemetryConstants.BB.MarkerId.AddBlock)
                    .AddBlockInfo(block)
                    .AddSceneInfo(spawnedObject.scene)
                    .Send();
            }

            return spawnedObjects;
        }

        protected virtual List<GameObject> InstallRoutine()
        {
            var instance = Instantiate(Prefab, Vector3.zero, Quaternion.identity);
            instance.SetActive(true);
            instance.name = $"{Utils.BlockPublicTag} {BlockName}";
            Undo.RegisterCreatedObjectUndo(instance, "Create " + instance.name);
            return new List<GameObject> { instance };
        }

        protected virtual List<GameObject> InstallRoutine(GameObject selectedGameObject)
        {
            return InstallRoutine();
        }

        private static void InstallDependencies(IEnumerable<BlockData> dependencies, GameObject selectedGameObject = null)
        {
            foreach (var dependency in dependencies)
            {
                if (IsBlockPresentInScene(dependency.Id))
                {
                    continue;
                }

                dependency.InstallWithDependencies(selectedGameObject);
            }
        }

        internal static bool IsBlockPresentInScene(string blockId)
        {
            return FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Any(x => x.BlockId == blockId);
        }

        public bool IsBlockPresentInScene()
        {
            return IsBlockPresentInScene(Id);
        }

        internal bool IsUpdateAvailableForBlock(BuildingBlock block)
        {
            return Version > block.Version;
        }

        internal void UpdateBlockToLatestVersion(BuildingBlock block)
        {
            if (!IsUpdateAvailableForBlock(block))
            {
                throw new InvalidOperationException(
                    $"Block {BlockName} is already in the latest version.");
            }

            if (IsSingleton)
            {
                foreach (var instance in this.GetBlocks())
                {
                    DestroyImmediate(instance.gameObject);
                }
            }
            else
            {
                DestroyImmediate(block.gameObject);
            }

            InstallWithDependenciesAndCommit();

            OVRTelemetry.Start(OVRTelemetryConstants.BB.MarkerId.UpdateBlock)
                .AddAnnotation(OVRTelemetryConstants.BB.AnnotationType.BlockId, Id)
                .AddAnnotation(OVRTelemetryConstants.BB.AnnotationType.Version, Version.ToString())
                .Send();
        }

        private static void SaveScene()
        {
            if (!IsCurrentSceneSaved())
            {
                return;
            }

            var activeScene = SceneManager.GetActiveScene();
            EditorSceneManager.MarkSceneDirty(activeScene);
            EditorSceneManager.SaveScene(activeScene);
        }

        private static bool IsCurrentSceneSaved()
        {
            var scenePath = SceneManager.GetActiveScene().path;
            return !string.IsNullOrEmpty(scenePath);
        }


        internal override bool OverridesInstallRoutine
        {
            get
            {
                var derivedMethodInfo = GetType().GetMethod(nameof(InstallRoutine),
                    BindingFlags.Instance | BindingFlags.NonPublic, null, new[] { typeof(GameObject) }, null);
                return derivedMethodInfo != null &&
                       derivedMethodInfo != derivedMethodInfo.GetBaseDefinition();
            }
        }
    }
}
