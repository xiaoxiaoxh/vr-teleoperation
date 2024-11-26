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

namespace Oculus.Interaction
{
    /// <summary>
    /// This component, to be placed at the root of a prefab, indicates that said
    /// prefab has been deprecated and should be replaced or unpacked before upgrading.
    /// </summary>
    public class DeprecatedPrefab : MonoBehaviour
    {
        public static readonly string label = "This prefab has been depecated. " +
            "Consider using using the replacement provided or unpack " +
            "the prefab before upgrading to a new version " +
            "in order to avoid losing information.";

        [SerializeField, HideInInspector]
        private UnityEngine.Object _replacement;

        [SerializeField, HideInInspector]
        private bool _supressWarning;

        protected virtual void Start()
        {
            if (_supressWarning)
            {
                return;
            }

            string higlightColor = AssertUtils.HiglightColor;
            string gameObjectName = this.gameObject.name;
            Debug.LogWarning($"At GameObject <color={higlightColor}><b>{gameObjectName}</b></color>. " +
                label, this);
        }
    }
}
