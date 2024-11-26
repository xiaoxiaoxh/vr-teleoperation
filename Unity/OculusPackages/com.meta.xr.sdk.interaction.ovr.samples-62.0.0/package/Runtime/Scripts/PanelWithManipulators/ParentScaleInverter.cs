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

namespace Oculus.Interaction.Samples
{
    /// <summary>
    /// It is not expected that typical users of the SlateWithManipulators prefab
    /// should need to either interact with or understand this script.
    ///
    /// This script simply inverts the scaling of the parent transform. This is useful
    /// for elements which should, for logic or placement reasons, exist within a scaled
    /// space, but which should retain their original world-space scaling independent
    /// of that space. All of the SlateWithManipulators affordances have this behavior
    /// except for those on the slate itself, which scales in a more specialized way.
    /// </summary>
    public class ParentScaleInverter : MonoBehaviour
    {
        private Vector3 _initialLocalScale;
        private Vector3 _initialParentScale;

        private void Start()
        {
            _initialLocalScale = this.transform.localScale;
            _initialParentScale = this.transform.parent.localScale;
        }

        private void LateUpdate()
        {
            this.transform.localScale = new Vector3(
                _initialParentScale.x * _initialLocalScale.x / this.transform.parent.localScale.x,
                _initialParentScale.y * _initialLocalScale.y / this.transform.parent.localScale.y,
                _initialParentScale.z * _initialLocalScale.z / this.transform.parent.localScale.z);
        }
    }
}
