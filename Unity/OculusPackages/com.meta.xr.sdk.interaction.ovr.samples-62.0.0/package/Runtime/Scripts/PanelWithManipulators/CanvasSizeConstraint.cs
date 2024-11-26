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
using System.Drawing;
using UnityEngine;

public class CanvasSizeConstraint : MonoBehaviour
{
    public Transform horizontalAnchorA;
    public Transform horizontalAnchorB;
    public Transform verticalAnchorA;
    public Transform verticalAnchorB;

    public float horizontalSizeOffset;
    public float verticalSizeOffset;

    private Vector2 _initialSize;
    private Vector2 _initialRectSize;
    private RectTransform _rectTransform;

    private Vector3 _initialLocalScale;

    private void Start()
    {
        _rectTransform = GetComponent<RectTransform>();
        _initialRectSize = _rectTransform.sizeDelta;

        _initialSize = new Vector2(
            Vector3.Distance(horizontalAnchorA.position, horizontalAnchorB.position) - horizontalSizeOffset,
            Vector3.Distance(verticalAnchorA.position, verticalAnchorB.position) - verticalSizeOffset);
        _initialLocalScale = _rectTransform.localScale;
    }

    private void Update()
    {
        var currentSize = new Vector2(
            Vector3.Distance(horizontalAnchorA.position, horizontalAnchorB.position) - horizontalSizeOffset,
            Vector3.Distance(verticalAnchorA.position, verticalAnchorB.position) - verticalSizeOffset);

        var sizeRatio = new Vector2(currentSize.x/ _initialSize.x, currentSize.y/ _initialSize.y);
        _rectTransform.localScale = new Vector3(_initialLocalScale.x / sizeRatio.x, _initialLocalScale.y / sizeRatio.y, _initialLocalScale.z);
        _rectTransform.sizeDelta = new Vector2(_initialRectSize.x * sizeRatio.x, _initialRectSize.y * sizeRatio.y);
    }
}
