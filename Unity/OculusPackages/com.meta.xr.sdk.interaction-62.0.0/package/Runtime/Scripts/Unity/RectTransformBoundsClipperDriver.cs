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
using Oculus.Interaction.Surfaces;

namespace Oculus.Interaction
{
    [RequireComponent(typeof(RectTransform))]
    [ExecuteInEditMode]
    public class RectTransformBoundsClipperDriver : MonoBehaviour
    {
        [SerializeField]
        private BoundsClipper _boundsClipper;

#if UNITY_EDITOR
        private void Reset()
        {
            if (_boundsClipper == null)
            {
                _boundsClipper = GetComponent<BoundsClipper>();
            }
        }

        private void OnValidate()
        {
            Resize();
        }
#endif

        protected virtual void Awake()
        {
            Resize();
        }

        protected virtual void Start()
        {
            this.AssertField(_boundsClipper, nameof(_boundsClipper));
        }

        private void OnRectTransformDimensionsChange()
        {
            Resize();
        }

        private void Resize()
        {
            if (_boundsClipper == null)
            {
                return;
            }

            RectTransform rt = transform as RectTransform;
            _boundsClipper.Size = new Vector3(rt.rect.width, rt.rect.height, 0.01f);
        }
    }
}
