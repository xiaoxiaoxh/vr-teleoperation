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

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Oculus.Interaction.Samples
{
    public class CanvasConstantWidthScaler : MonoBehaviour
    {
        [SerializeField]
        private RectTransform _rect;

        private float _initialLocalScaleY;
        private float _initialWidth;
        private float _initialHeight;

        private void Start()
        {
            _initialLocalScaleY = transform.localScale.y;
            _initialWidth = _rect.sizeDelta.x;
            _initialHeight = _rect.sizeDelta.y;
        }

        private void Update()
        {
            transform.localScale = new Vector3(
                transform.localScale.x,
                _initialLocalScaleY * transform.parent.lossyScale.x / transform.parent.lossyScale.y,
                transform.localScale.z);
            _rect.sizeDelta = new Vector2(_initialWidth, _initialHeight * transform.localScale.x / transform.localScale.y);
        }
    }
}
