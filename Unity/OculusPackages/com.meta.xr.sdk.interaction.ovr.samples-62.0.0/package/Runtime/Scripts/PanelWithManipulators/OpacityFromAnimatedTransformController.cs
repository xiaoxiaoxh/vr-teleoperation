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
    /// This script monitors a transform controlled by an animation in order to set
    /// the opacity of a material. This allows opacity to be controlled by an
    /// animation so that visual behaviors like transparency and hydration curves
    /// can be set using art workflows rather than code.
    /// </summary>
    public class OpacityFromAnimatedTransformController : MonoBehaviour
    {
        [SerializeField, Tooltip("The renderer to which the opacity should be applied")]
        private Renderer _renderer;

        [SerializeField, Tooltip("The animation-controlled transform whose X magnitude will be applied to the renderer as `_Opacity`")]
        private Transform _opacityTransform;

        private MaterialPropertyBlock _materialProperties;

        private bool _isSkinnedMeshRenderer;

        private void Start()
        {
            _isSkinnedMeshRenderer = _renderer is SkinnedMeshRenderer;
            if (!_isSkinnedMeshRenderer)
            {
                _materialProperties = new MaterialPropertyBlock();
            }
        }

        private void Update()
        {
            float opacity = Mathf.Abs(_opacityTransform.localPosition.x);

            if (_isSkinnedMeshRenderer)
            {
                _renderer.material.SetFloat("_Opacity", opacity);
            }
            else
            {
                _materialProperties.SetFloat("_Opacity", opacity);
                _renderer.SetPropertyBlock(_materialProperties);
            }
        }
    }
}
