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
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

/// <summary>
/// This component smoothly moves the contentContainer horizontally
/// until the selected page is completely visible, the anchor of the page is at the (0, any)
/// position relative to the parent of the contentContainer when the page anchor is set to "top-left"
/// </summary>
public class PageScroll : UIBehaviour
{
    [SerializeField] private UnityEngine.UI.ToggleGroup _toggleGroup;
    [SerializeField] private RectTransform _contentContainer;

    [Serializable]
    public struct Page{
        public UnityEngine.UI.Toggle toggle;
        public RectTransform container;
        public CanvasGroup canvasGroup;
    }
    [SerializeField] private List<Page> _pages;
    [SerializeField] private int _pageIndex;

    public float animationSpeed;
    public AnimationCurve alphaTransitionCurve;
    private float _pageAnim;

    public void SetPageIndex(int pageIndex)
    {
        var index = pageIndex < 0 ? 0 : pageIndex > _pages.Count - 1 ? _pages.Count - 1 : pageIndex;
        if (this._pageIndex != index)
        {
            this._pageIndex = index;
            _pages[this._pageIndex].toggle.isOn = true;
        }
    }

    public void ScrollPage(int direction)
    {
        var newPage = _pageIndex + direction;
        SetPageIndex(newPage);
    }

/*#if UNITY_EDITOR
    protected override void OnValidate()
    {
        if (_pageIndex < 0)
        {
            _pageIndex = 0;
        }
        else if (_pageIndex > _pages.Count - 1)
        {
            _pageIndex = _pages.Count - 1;
        }
        _pages[_pageIndex].toggle.isOn = true;
    }
#endif*/

    protected override void OnEnable()
    {
        foreach (var page in _pages)
        {
            page.toggle.onValueChanged.AddListener(delegate { ActiveToggleChanged(page.toggle); });
        }
    }

    protected override void OnDisable()
    {
        foreach (var page in _pages)
        {
            page.toggle.onValueChanged.RemoveAllListeners();
        }
    }

    private void ActiveToggleChanged(UnityEngine.UI.Toggle toggle)
    {
        if (toggle == null) return;
        if (!toggle.isOn) return;
        var toggleIndex = _pages.FindIndex(page => page.toggle == toggle);
        var containsToggle = toggleIndex >= 0;
        if (!containsToggle) return;
        _pageIndex = toggleIndex;
    }

    protected override void Start()
    {
        StartCoroutine(LateStart());
    }

    private IEnumerator LateStart()
    {
        yield return null;
        if (_pages == null) yield break;
        _pages[0].toggle.isOn = true;
    }

    protected virtual void Update()
    {
        _pageAnim = Mathf.Lerp(_pageAnim, (float)_pageIndex, animationSpeed * Time.deltaTime);
        _pageAnim = Mathf.Clamp(_pageAnim, 0.0f, _pages.Count - 1);
        UpdateVisial();
    }

    private void UpdateVisial()
    {
        if (Mathf.Abs(_pageAnim - (float)_pageIndex) < 0.005f)
        {
            var position = _pages[_pageIndex].container.anchoredPosition;
            _pages[_pageIndex].canvasGroup.alpha = 1.0f;
            SetOtherPagesTransparent(_pageIndex, -1);
            _contentContainer.anchoredPosition = position * new Vector2(-1.0f, 1.0f);
        }
        else
        {
            var anim = Mathf.Clamp(_pageAnim, 0.0f, _pages.Count - 1);

            var currentPage = Mathf.Floor(anim);
            var nextPage = Mathf.Ceil(anim);

            var currentPageIndex = (int)currentPage;
            var nextPageIndex = (int)nextPage;
            var animParam = anim - currentPage;

            var startPosition = _pages[currentPageIndex].container.anchoredPosition;
            var endPosition = _pages[nextPageIndex].container.anchoredPosition;

            SetOtherPagesTransparent(currentPageIndex, nextPageIndex);
            _pages[currentPageIndex].canvasGroup.alpha = alphaTransitionCurve.Evaluate(1.0f - animParam);
            _pages[nextPageIndex].canvasGroup.alpha = alphaTransitionCurve.Evaluate(animParam);

            _contentContainer.anchoredPosition = Vector2.Lerp(startPosition, endPosition, animParam) * new Vector2(-1.0f, 1.0f);
        }
    }

    private void SetOtherPagesTransparent(int index0, int index1)
    {
        for (int i = 0; i < _pages.Count; i++)
        {
            if (i == index0 || i == index1) continue;
            if (_pages[i].canvasGroup == null) continue;
            _pages[i].canvasGroup.alpha = 0;
        }
    }

    #region Inject

    public void InjectAllPageScroll(UnityEngine.UI.ToggleGroup toggleGroup, RectTransform contentContainer, List<Page> pages, int pageIndex)
    {
        InjectToggleGroup(toggleGroup);
        InjectContentContainer(contentContainer);
        InjectPages(pages);
        InjectPageIndex(pageIndex);
    }

    public void InjectToggleGroup(UnityEngine.UI.ToggleGroup toggleGroup)
    {
        this._toggleGroup = toggleGroup;
    }

    public void InjectContentContainer(RectTransform contentContainer)
    {
        this._contentContainer = contentContainer;
    }

    public void InjectPages(List<Page> pages)
    {
        this._pages = pages;
    }

    public void InjectPageIndex(int pageIndex)
    {
        this._pageIndex = pageIndex;
    }
    #endregion
}
