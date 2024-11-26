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

using Oculus.Interaction.Surfaces;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Animations;

public class PanelSetup : MonoBehaviour
{

    public float InteractableLength;
    public float InteractableDepth;

    public bool AddVerticalRotation;
    public bool AddHorizontalRotation;

    public RectTransform panelTransform;
    public UnionClippedPlaneSurface panelClippedPlaneSurface;
    public BoundsClipper boundsClipper;
    public Transform topLeftCornerAnchor;

    [Header("Anchors")]
    public Transform AnchorTopLeft;
    public Transform AnchorTopRight;
    public Transform AnchorBottomLeft;
    public Transform AnchorBottomRight;

    [Header("SideCollider")]
    public GameObject PanelInteractable;

    [Header("Scaler")]
    public GameObject ScalerTopLeft;
    public GameObject ScalerTopRight;
    public GameObject ScalerBottomLeft;
    public GameObject ScalerBottomRight;

    [Header("Rotator")]
    public GameObject RotatorVerticalTop;
    public GameObject RotatorVerticalBottom;
    public GameObject RotatorHorizontalLeft;
    public GameObject RotatorHorizontalRight;

    [ContextMenu("Update Panel")]
    public void UpdatePanelProperties()
    {
        Debug.Log("Update Function");
        var rectSize = panelTransform.sizeDelta * panelTransform.lossyScale;
        var corners = GetRectCorners(Vector3.zero, rectSize);
        var halfLength = InteractableLength * 0.5f;

        AnchorTopLeft.localPosition = corners[0] + halfLength * (Vector3)Vec2Sign(corners[0]);
        AnchorTopRight.localPosition = corners[1] + halfLength * (Vector3)Vec2Sign(corners[1]);
        AnchorBottomLeft.localPosition = corners[2] + halfLength * (Vector3)Vec2Sign(corners[2]);
        AnchorBottomRight.localPosition = corners[3] + halfLength * (Vector3)Vec2Sign(corners[3]);

        var InteractableSize = new Vector3(InteractableLength, InteractableLength, InteractableDepth);
        SetColliderSize(ScalerTopLeft, InteractableSize);
        SetColliderSize(ScalerTopRight, InteractableSize);
        SetColliderSize(ScalerBottomLeft, InteractableSize);
        SetColliderSize(ScalerBottomRight, InteractableSize);

        if (AddVerticalRotation)
        {
            SetColliderSize(RotatorVerticalTop, InteractableSize);
            SetColliderSize(RotatorVerticalBottom, InteractableSize);
            RotatorVerticalTop.gameObject.SetActive(true);
            RotatorVerticalBottom.gameObject.SetActive(true);
        }
        else
        {
            RotatorVerticalTop.gameObject.SetActive(false);
            RotatorVerticalBottom.gameObject.SetActive(false);
        }

        if (AddHorizontalRotation)
        {
            SetColliderSize(RotatorHorizontalLeft, InteractableSize);
            SetColliderSize(RotatorHorizontalRight, InteractableSize);
            RotatorHorizontalLeft.gameObject.SetActive(true);
            RotatorHorizontalRight.gameObject.SetActive(true);
        }
        else
        {
            RotatorHorizontalLeft.gameObject.SetActive(false);
            RotatorHorizontalRight.gameObject.SetActive(false);
        }

        var sides = GetRectSides(Vector3.zero, rectSize);

        if (AddVerticalRotation)
        {
            CreateCollider("ColliderUpLeft", rectSize, sides[0], Vector3.up, Vector3.left, false, 0, 1, RotatorVerticalTop.transform, ScalerTopLeft.transform);
            CreateCollider("ColliderUpRight", rectSize, sides[0], Vector3.up, Vector3.right, false, 0, 1, RotatorVerticalTop.transform, ScalerTopRight.transform);

            CreateCollider("ColliderDownLeft", rectSize, sides[1], Vector3.down, Vector3.left, false, 0, 1, RotatorVerticalBottom.transform, ScalerBottomLeft.transform);
            CreateCollider("ColliderDownRight", rectSize, sides[1], Vector3.down, Vector3.right, false, 0, 1, RotatorVerticalBottom.transform, ScalerBottomRight.transform);
        }
        else
        {
            CreateCollider("ColliderUp", rectSize, sides[0], Vector3.up, Vector3.zero, true, 0, 1, ScalerTopLeft.transform, ScalerTopRight.transform);
            CreateCollider("ColliderDown", rectSize, sides[1], Vector3.down, Vector3.zero, true, 0, 1, ScalerBottomLeft.transform, ScalerBottomRight.transform);
        }

        if (AddHorizontalRotation)
        {
            CreateCollider("ColliderLeftUp", rectSize, sides[2], Vector3.left, Vector3.up, false, 1, 0, RotatorHorizontalLeft.transform, ScalerTopLeft.transform);
            CreateCollider("ColliderLeftDown", rectSize, sides[2], Vector3.left, Vector3.down, false, 1, 0, RotatorHorizontalLeft.transform, ScalerBottomLeft.transform);

            CreateCollider("ColliderRightUp", rectSize, sides[3], Vector3.right, Vector3.up, false, 1, 0, RotatorHorizontalRight.transform, ScalerTopRight.transform);
            CreateCollider("ColliderRightDown", rectSize, sides[3], Vector3.right, Vector3.down, false, 1, 0, RotatorHorizontalRight.transform, ScalerBottomRight.transform);
        }
        else
        {
            CreateCollider("ColliderLeft", rectSize, sides[2], Vector3.left, Vector3.zero, true, 1, 0, ScalerBottomLeft.transform, ScalerTopLeft.transform);
            CreateCollider("ColliderRight", rectSize, sides[3], Vector3.right, Vector3.zero, true, 1, 0, ScalerBottomRight.transform, ScalerTopRight.transform);
        }

        boundsClipper.Size = new Vector3(rectSize.x, rectSize.y, InteractableDepth);

        topLeftCornerAnchor.localPosition = new Vector3(-rectSize.x * 0.5f, rectSize.y * 0.5f, 0.0f);
    }

    private void CreateCollider(string name, Vector2 rectSize, Vector3 sidePosition, Vector3 sideDirection,
        Vector3 offsetDirection, bool fullSize, int wideAxis, int normalAxis, Transform anchorA, Transform anchorB)
    {
        var halfLength = InteractableLength * 0.5f;
        var bound = new GameObject(name);
        bound.transform.SetParent(PanelInteractable.transform, false);
        var collider = bound.AddComponent<BoxCollider>();
        collider.isTrigger = true;
        bound.AddComponent<BoundsClipper>();

        var positionConstraint = bound.AddComponent<PositionConstraint>();
        positionConstraint.AddSource(new ConstraintSource() { sourceTransform = anchorA, weight = 1.0f });
        positionConstraint.AddSource(new ConstraintSource() { sourceTransform = anchorB, weight = 1.0f });
        positionConstraint.constraintActive = true;

        bound.transform.localPosition = sidePosition + sideDirection * halfLength;
        var boundSize = fullSize ? rectSize[wideAxis] : (Mathf.Abs(rectSize[wideAxis] - InteractableLength) / 2.0f);
        bound.transform.localPosition += offsetDirection * (halfLength + boundSize * 0.5f);
        var size = new Vector3(0.0f, 0.0f, InteractableDepth);
        size[wideAxis] = boundSize;
        size[normalAxis] = InteractableLength;

        var sizeConstraint = bound.AddComponent<ColliderSizeConstraint>();
        sizeConstraint.pointA = anchorA;
        sizeConstraint.pointB = anchorB;
        sizeConstraint.size = size;
        sizeConstraint.wideSideOffset = InteractableLength;
        sizeConstraint.expandingAxis = wideAxis;

        bound.transform.localScale = size;
    }

    private void SetColliderSize(GameObject colliderGO, Vector3 size)
    {
        if (colliderGO == null) return;
        var scale = colliderGO.transform.lossyScale;
        var scaledSize = new Vector3(size.x / scale.x, size.y / scale.y, size.z / scale.z);
        var collider = colliderGO.GetComponent<BoxCollider>();
        if (collider != null)
        {
            collider.size = scaledSize;
        }
        var boundsClipper = colliderGO.GetComponent<BoundsClipper>();
        if (boundsClipper != null)
        {
            boundsClipper.Size = scaledSize;
        }
    }

    private Vector2 Vec2Sign(Vector2 value)
    {
        return new Vector2(Mathf.Sign(value.x), Mathf.Sign(value.y));
    }

    private Vector3[] GetRectCorners(Vector3 position, Vector2 size)
    {
        var corners = new Vector3[4];
        corners[0] = position + new Vector3(-size.x * 0.5f, size.y * 0.5f, 0.0f);
        corners[1] = position + new Vector3(size.x * 0.5f, size.y * 0.5f, 0.0f);
        corners[2] = position + new Vector3(-size.x * 0.5f, -size.y * 0.5f, 0.0f);
        corners[3] = position + new Vector3(size.x * 0.5f, -size.y * 0.5f, 0.0f);
        return corners;
    }
    private Vector3[] GetRectSides(Vector3 position, Vector2 size)
    {
        var sides = new Vector3[4];

        sides[0] = position + new Vector3(0.0f, size.y * 0.5f, 0.0f);
        sides[1] = position + new Vector3(0.0f, -size.y * 0.5f, 0.0f);

        sides[2] = position + new Vector3(-size.x * 0.5f, 0.0f, 0.0f);
        sides[3] = position + new Vector3(size.x * 0.5f, 0.0f, 0.0f);

        return sides;
    }
}
