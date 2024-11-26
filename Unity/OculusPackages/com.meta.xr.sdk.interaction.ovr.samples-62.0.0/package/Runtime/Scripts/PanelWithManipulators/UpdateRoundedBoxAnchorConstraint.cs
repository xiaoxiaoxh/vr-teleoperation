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
using UnityEngine.Animations;

public class UpdateRoundedBoxAnchorConstraint : MonoBehaviour
{
    [SerializeField]
    private PositionConstraint _topLeft;
    [SerializeField]
    private PositionConstraint _topRight;
    [SerializeField]
    private PositionConstraint _bottomLeft;
    [SerializeField]
    private PositionConstraint _bottomRight;
    [SerializeField]
    private float _interactableLength;
    [SerializeField]
    private Vector2 _offset;

    private static void UpdateOffset(PositionConstraint constraint, Vector2 direction, Vector2 offset, float interactableLength)
    {
        constraint.translationOffset = direction * offset + direction * interactableLength * 0.5f;
    }

    public static void UpdateAnchors(PositionConstraint topLeft, PositionConstraint topRight, PositionConstraint bottomLeft, PositionConstraint bottomRight, Vector2 offset, float interactableLength)
    {
        UpdateOffset(topLeft, new Vector2(1.0f, -1.0f), offset, interactableLength);
        UpdateOffset(topRight, new Vector2(-1.0f, -1.0f), offset, interactableLength);
        UpdateOffset(bottomLeft, new Vector2(1.0f, 1.0f), offset, interactableLength);
        UpdateOffset(bottomRight, new Vector2(-1.0f, 1.0f), offset, interactableLength);
    }

    [ContextMenu("Update Anchors")]
    public void UpdateAnchorsMenu()
    {
        UpdateAnchors(_topLeft, _topRight, _bottomLeft, _bottomRight, _offset, _interactableLength);
    }
}
