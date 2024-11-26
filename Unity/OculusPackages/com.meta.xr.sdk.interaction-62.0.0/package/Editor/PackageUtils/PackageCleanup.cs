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

namespace Oculus.Interaction.Editor
{
    [Obsolete("Will be removed in future release.", false)]
    public static class PackageCleanup
    {

        public const string PACKAGE_VERSION = "0.62.0";
        public const string DEPRECATED_TAG = "oculus_interaction_deprecated";
        public const string MOVED_TAG = "oculus_interaction_moved_";

        /// <summary>
        /// Obsolete - Check if there are any assets in the project that require
        /// cleanup operations.
        /// </summary>
        /// <returns>True if package needs cleanup</returns>
        public static bool CheckPackageNeedsCleanup()
        {
            return false;
        }

        /// <summary>
        /// Obsolete - Start the removal flow for removing deprecated assets.
        /// </summary>
        /// <param name="userTriggered">If true, the window will
        /// be non-modal, and a dialog will be shown if no assets found</param>
        public static void StartRemovalUserFlow(bool userTriggered){}
    }
}
