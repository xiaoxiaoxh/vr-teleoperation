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

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

[ExecuteAlways]
public class VirtualLayout : UIBehaviour
{
    public float animationSpeed;
    [SerializeField] private RectTransform _layoutParent;

    private List<RectTransform> _rectChildren;
    private List<RectTransform> _virtualLayoutChildren;

    protected override void OnEnable()
    {
        if (_layoutParent == null) return;
        var layoutChildren = _layoutParent.gameObject.GetComponentsInChildren<RectTransform>();
        for (int i = 1; i < layoutChildren.Length; i++)
        {
            var child = layoutChildren[i];
            if (Application.isPlaying)
            {
                Destroy(child.gameObject);
            }
            else
            {
                DestroyImmediate(child.gameObject);
            }
        }

        var children = gameObject.GetComponentsInChildren<RectTransform>();
        _rectChildren = new List<RectTransform>();
        _virtualLayoutChildren = new List<RectTransform>();
        for (int i = 1; i < children.Length; i++)
        {
            var child = children[i];
            if (child.parent != (RectTransform)transform) continue;
            _rectChildren.Add(child);
            ResetChildTransform(child);


            var virtualTransform = new GameObject();
            virtualTransform.hideFlags = HideFlags.HideAndDontSave;
            virtualTransform.name = child.name;
            virtualTransform.AddComponent<RectTransform>();
            var virtualChild = (RectTransform)virtualTransform.transform;
            virtualChild.SetParent(_layoutParent, false);
            ResetChildTransform(virtualChild);

            _virtualLayoutChildren.Add(virtualChild);
        }

        _layoutParent.ForceUpdateRectTransforms();
    }

    private void ResetChildTransform(RectTransform child)
    {
        child.localPosition = Vector3.zero;
        child.anchoredPosition = Vector2.zero;
        child.localScale = Vector2.one;
        child.localRotation = Quaternion.identity;
        child.anchorMin = Vector2.zero;
        child.anchorMax = Vector2.zero;
        child.pivot = new Vector2(0.5f, 0.5f);
    }

    protected override void OnDisable()
    {
        foreach (var child in _virtualLayoutChildren)
        {
            if (Application.isPlaying)
            {
                Destroy(child.gameObject);
            }
            else
            {
                DestroyImmediate(child.gameObject);
            }
        }
    }

    private void LateUpdate()
    {
        if (_layoutParent == null) return;
        var layoutTransform = (RectTransform)transform;
        layoutTransform.anchoredPosition = _layoutParent.anchoredPosition;
        for (int i = 0; i < _virtualLayoutChildren.Count; i++)
        {
            var rectChild = _rectChildren[i];
            var virtualChild = _virtualLayoutChildren[i];
            if (Application.isPlaying)
            {
                rectChild.anchoredPosition = Vector2.Lerp(rectChild.anchoredPosition, virtualChild.anchoredPosition, animationSpeed * Time.deltaTime);
                rectChild.sizeDelta = Vector2.Lerp(rectChild.sizeDelta, virtualChild.sizeDelta, animationSpeed * Time.deltaTime);
            }
            else
            {
                rectChild.anchoredPosition = virtualChild.anchoredPosition + _layoutParent.anchoredPosition;
                rectChild.sizeDelta = virtualChild.sizeDelta;
            }
        }
    }

    #region Inject
    public void InjectAllVirtualLayoutElement(RectTransform layoutParent)
    {
        _layoutParent = layoutParent;
    }
    #endregion
}
