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
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{
    internal class UPMBlockDownloaderData : BlockDownloaderData
    {
        [SerializeField] private string packageId;

        internal override void AddToProject(GameObject selectedGameObject = null, Action onInstall = null)
        {
            Install();
            onInstall?.Invoke();
        }

        protected override bool IsInstalled() => OVRProjectSetupUtils.IsPackageInstalled(packageId);


        [ContextMenu("Install")]
        protected override void Install()
        {
            var success = OVRProjectSetupUtils.InstallPackage(packageId);

            if (!success)
            {
                throw new InvalidOperationException(
                    $"Installation of package {packageId} failed for block {BlockName}.");
            }
        }

        [ContextMenu("Remove")]
        protected override void Remove()
        {
            if (!IsInstalled())
            {
                return;
            }

            var success = OVRProjectSetupUtils.UninstallPackage(packageId);

            if (!success)
            {
                throw new InvalidOperationException(
                    $"Removal of package {packageId} failed for block {BlockName}.");
            }
        }


    }
}
