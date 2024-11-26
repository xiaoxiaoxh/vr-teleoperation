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

#if UNITY_2021_2_OR_NEWER
#define OVR_BB_DRAGANDDROP
#endif

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Meta.XR.Editor.Tags;
using UnityEditor;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{
    public class BuildingBlocksWindow : EditorWindow
    {
        private const string MenuPath = "Oculus/Tools/Building Blocks";
        private const int MenuPriority = 2;
        private const string WindowName = Utils.BlocksPublicName;
        private const string AddButtonLabel = "Add";

#if OVR_BB_DRAGANDDROP
        private const string DragAndDropLabel = "Dragging Block";
        private const string DragAndDropBlockDataLabel = "block";
        private const string DragAndDropBlockThumbnailLabel = "blockThumbnail";
#endif // OVR_BB_DRAGANDDROP

        private static readonly GUIContent Title = new GUIContent(Utils.BlocksPublicName);

        private static readonly GUIContent Description =
            new GUIContent("Drag and drop blocks onto your scene to add XR features to your project.");

        private const string DocumentationUrl = "https://developer.oculus.com/documentation/unity/unity-buildingblocks-overview";


        private Vector2 _scrollPosition;

        private OVRAnimatedContent _outline = null;
        private OVRAnimatedContent _tutorial = null;
        private static readonly OVRProjectSetupSettingBool _tutorialCompleted =
            new OVRProjectSetupUserSettingBool("BuildingBlocksTutorialCompleted", false);
        private static bool _shouldShowTutorial = false;
        private bool isHoveringHotControl = false;

        private HashSet<Tag> _tagSearch = new HashSet<Tag>();
        private string _filterSearch = "";

        private Repainter _repainter = new Repainter();
        private Dimensions _dimensions = new Dimensions();

        private class Repainter
        {
            public bool NeedsRepaint { get; private set; }
            public Vector2 MousePosition { get; private set; }

            public void Assess(EditorWindow window)
            {
                if (Event.current.type == EventType.Layout)
                {
                    var fullRect = new Rect(0, 0, window.position.width, window.position.height);
                    var isMoving = Event.current.mousePosition != MousePosition;
                    MousePosition = Event.current.mousePosition;
                    var isMovingOver = fullRect.Contains(Event.current.mousePosition);
                    if (isMoving && isMovingOver)
                    {
                        NeedsRepaint = true;
                    }

                    if (NeedsRepaint)
                    {
                        window.Repaint();
                        NeedsRepaint = false;
                    }
                }
            }

            public void RequestRepaint()
            {
                NeedsRepaint = true;
            }
        }

        private class Dimensions
        {
            public int WindowWidth { get; private set; }
            public int ExpectedThumbnailWidth { get; private set; }
            public int ExpectedThumbnailHeight { get; private set; }
            public int NumberOfColumns { get; private set; }

            private int _previousThumbnailWidth;
            private int _previousThumbnailHeight;

            public void Refresh(EditorWindow window)
            {
                var windowWidth = (int)window.position.width - Styles.BlockMargin;
                if (Math.Abs(WindowWidth - windowWidth) <= Mathf.Epsilon)
                {
                    return;
                }

                WindowWidth = windowWidth;

                var blockWidth = Styles.IdealThumbnailWidth;
                windowWidth = Mathf.Max(Styles.IdealThumbnailWidth + Styles.Padding * 3, windowWidth);
                var scrollableAreaWidth = windowWidth - 18;
                NumberOfColumns = Mathf.FloorToInt(scrollableAreaWidth / blockWidth);
                if (NumberOfColumns < 1) NumberOfColumns = 1;
                var marginToRemove = NumberOfColumns * Styles.BlockMargin;

                ExpectedThumbnailWidth = (int)Mathf.FloorToInt((scrollableAreaWidth - marginToRemove) / NumberOfColumns);
                ExpectedThumbnailHeight = (int)Mathf.FloorToInt(ExpectedThumbnailWidth / Styles.ThumbnailRatio);
                if (ExpectedThumbnailWidth != _previousThumbnailWidth || ExpectedThumbnailHeight != _previousThumbnailHeight)
                {
                    _previousThumbnailWidth = ExpectedThumbnailWidth;
                    _previousThumbnailHeight = ExpectedThumbnailHeight;
                    OVREditorUtils.TweenHelper.Reset();
                }
            }
        }

        [MenuItem(MenuPath, false, MenuPriority)]
        private static void ShowWindow()
        {
            ShowWindow("MenuItem");
        }

        public static void ShowWindow(string source)
        {
            var window = GetWindow<BuildingBlocksWindow>(WindowName);
            window.minSize = new Vector2(800, 400);

            OVRTelemetry.Start(OVRTelemetryConstants.BB.MarkerId.OpenWindow)
                .AddAnnotation(OVRTelemetryConstants.BB.AnnotationType.ActionTrigger, source)
                .Send();
        }

        private void OnGUI()
        {
            if (Event.current.type == EventType.MouseMove)
            {
                _repainter.RequestRepaint();
                return;
            }

            OnHeaderGUI();

            isHoveringHotControl = false;

            _dimensions.Refresh(this);

            ShowList(_dimensions);

#if OVR_BB_DRAGANDDROP
            RefreshDragAndDrop(_dimensions);
#endif // OVR_BB_DRAGANDDROP

            _repainter.Assess(this);
        }

        private void OnHeaderGUI()
        {
            EditorGUILayout.BeginHorizontal(Styles.Header);
            {
                using (new OVREditorUtils.OVRGUIColorScope(OVREditorUtils.OVRGUIColorScope.Scope.Content, Styles.Colors.AccentColor))
                {
                    EditorGUILayout.LabelField(Styles.HeaderIcon, Styles.HeaderIconStyle, GUILayout.Width(32.0f),
                        GUILayout.ExpandWidth(false));
                }
                EditorGUILayout.LabelField(Title, Styles.BoldLabel);

                EditorGUILayout.Space(0, true);

                using (new OVREditorUtils.OVRGUIColorScope(OVREditorUtils.OVRGUIColorScope.Scope.Content,
                           Styles.Colors.LightGray))
                {
                    if (GUILayout.Button(Styles.ConfigIcon, Styles.MiniButton))
                    {
                        ShowSettingsMenu();
                    }

                    if (GUILayout.Button(Styles.DocumentationIcon, Styles.MiniButton))
                    {
                        Application.OpenURL(DocumentationUrl);
                    }
                }

            }
            EditorGUILayout.EndHorizontal();

            if (!OVREditorUtils.IsUnityVersionCompatible())
            {
                using (new OVREditorUtils.OVRGUIColorScope(OVREditorUtils.OVRGUIColorScope.Scope.Background, Styles.Colors.ErrorColorSemiTransparent))
                {
                    EditorGUILayout.LabelField(
                        $"<b>Warning:</b> Your version of Unity is not supported. Consider upgrading to {OVREditorUtils.VersionCompatible} or higher.",
                        Styles.ErrorHelpBox);
                }
            }

            EditorGUILayout.BeginHorizontal(Styles.SubtitleHelpText);
            GUILayout.Label(Description, Styles.LabelStyle);
            EditorGUILayout.EndHorizontal();
        }


        private void ShowSettingsMenu()
        {
            var menu = new GenericMenu();
            foreach (var tag in Tag.Registry)
            {
                if (tag.Behavior.ToggleableVisibility)
                {
                    tag.Behavior.VisibilitySetting.AppendToMenu(menu, ClearTagSearch);
                }
            }
            menu.ShowAsContext();
        }

        private void ClearTagSearch()
        {
            _tagSearch.Clear();
        }

        private void RefreshShowTutorial()
        {
            _shouldShowTutorial = ShouldShowTutorial();
        }

        private void OnEnable()
        {
            RefreshBlockList();
#if OVR_BB_DRAGANDDROP
            DragAndDrop.AddDropHandler(SceneDropHandler);
            DragAndDrop.AddDropHandler(HierarchyDropHandler);
#endif // OVR_BB_DRAGANDDROP
            wantsMouseMove = true;
            RefreshShowTutorial();
        }

        internal void RefreshBlockList()
        {
            _blockList = BlocksContentManager.FilterBlockWindowContent(GetList());
        }

        private void OnDisable()
        {
#if OVR_BB_DRAGANDDROP
            DragAndDrop.RemoveDropHandler(SceneDropHandler);
            DragAndDrop.RemoveDropHandler(HierarchyDropHandler);
#endif // OVR_BB_DRAGANDDROP
        }

        private List<BlockBaseData> _blockList;

        private static List<BlockBaseData> GetList()
        {
            var blockGuids = AssetDatabase.FindAssets($"t:{nameof(BlockBaseData)}");

            return blockGuids.Select(id =>
                    AssetDatabase.LoadAssetAtPath<BlockBaseData>(AssetDatabase.GUIDToAssetPath(id)))
                .Where(obj => !string.IsNullOrEmpty(obj.name))
                .OrderBy(block => block.Order)
                .ThenBy(block => block.BlockName)
                .ToList();
        }

        private void ShowList(Dimensions dimensions)
        {
            GUILayout.BeginHorizontal(Styles.FilterByLine);
            EditorGUILayout.LabelField("Filter by", EditorStyles.miniBoldLabel, GUILayout.Width(44));
            ShowTagList("window", Tag.Registry.SortedTags, _tagSearch, Tag.TagListType.Filters);
            EditorGUILayout.Space(0, true);
            _filterSearch = EditorGUILayout.TextField(_filterSearch, GUI.skin.FindStyle("SearchTextField"), GUILayout.Width(256));
            GUILayout.EndHorizontal();

            ShowList(_blockList, Filter, dimensions);
        }

        private IEnumerable<BlockBaseData> Filter(IEnumerable<BlockBaseData> blocks) => blocks.Where(Match);

        private bool Match(BlockBaseData block)
        {
            if (block.Hidden && Utils.HiddenTag.Behavior.Visibility == false) return false;

            if (block.Tags.Any(tag => tag.Behavior.Visibility == false)) return false;

            if (_tagSearch.Any(tag => !block.Tags.Contains(tag))) return false;

            var containsSearch = string.IsNullOrEmpty(_filterSearch)
                           || block.blockName.Contains(_filterSearch)
                           || block.Description.Contains(_filterSearch)
                           || block.Tags.Any(tag => tag.Name.Contains(_filterSearch));
            return containsSearch;
        }

        private bool HasAnyBlock(Tag tag)
        {
            return _blockList.Any((data =>
                data.Tags.Contains(tag) && data.Tags.All(otherTag => otherTag.Behavior.Visibility != false)));
        }

        private void ShowList(List<BlockBaseData> blocks, Func<IEnumerable<BlockBaseData>,
                IEnumerable<BlockBaseData>> filter, Dimensions dimensions)
        {
            _scrollPosition = EditorGUILayout.BeginScrollView(_scrollPosition, false, true, GUIStyle.none, GUI.skin.verticalScrollbar, Styles.NoMargin, GUILayout.Width(dimensions.WindowWidth));

            var blockWidth = dimensions.ExpectedThumbnailWidth;
            var blockHeight = dimensions.ExpectedThumbnailHeight + Styles.DescriptionAreaStyle.fixedHeight + 3;

            var columnIndex = 0;
            var lineIndex = 0;
            var showTutorial = _shouldShowTutorial;
            GUILayout.BeginHorizontal(Styles.NoMargin);
            var filteredBlocks = filter(blocks);
            foreach (var block in filteredBlocks)
            {
                var blockRect = new Rect(columnIndex * (blockWidth + Styles.BlockMargin) + Styles.BlockMargin, lineIndex * (blockHeight + Styles.BlockMargin), blockWidth, blockHeight);
                Show(block, blockRect, dimensions.ExpectedThumbnailWidth, dimensions.ExpectedThumbnailHeight);

                if (showTutorial && block.CanBeAdded)
                {
                    ShowTutorial(blockRect);
                    showTutorial = false;
                }

                columnIndex++;
                if (columnIndex >= dimensions.NumberOfColumns)
                {
                    lineIndex++;
                    columnIndex = 0;
                    GUILayout.EndHorizontal();
                    GUILayout.BeginHorizontal(Styles.NoMargin);
                }
            }

            GUILayout.EndHorizontal();

            EditorGUILayout.EndScrollView();
        }

        private void ShowThumbnail(BlockBaseData block, float targetHeight, int expectedThumbnailHeight)
        {
            var thumbnailAreaStyle = new GUIStyle(Styles.ThumbnailAreaStyle);
            thumbnailAreaStyle.fixedHeight = targetHeight;
            var thumbnailArea = EditorGUILayout.BeginVertical(thumbnailAreaStyle, GUILayout.Height(thumbnailAreaStyle.fixedHeight));
            {
                thumbnailArea.height = expectedThumbnailHeight;
                GUI.DrawTexture(thumbnailArea, block.Thumbnail, ScaleMode.ScaleAndCrop);

                var hasAttributes = ShowTagList(block.Id + "overlay", block.Tags, _tagSearch, Tag.TagListType.Overlays);
                if (!hasAttributes)
                {
                    // This space fills the area, otherwise the area will have a height of null
                    // despite the fixedHeight set
                    EditorGUILayout.Space();
                }
            }
            EditorGUILayout.EndVertical();

            EditorGUILayout.BeginVertical(Styles.SeparatorAreaStyle);
            {
                // This space fills the area, otherwise the area will have a height of null
                // despite the fixedHeight set
                EditorGUILayout.Space();
            }
            EditorGUILayout.EndVertical();
        }

        private void ShowButtons(BlockBaseData block, Rect blockRect, bool canBeAdded, bool canBeSelected)
        {
            GUILayout.BeginArea(blockRect, Styles.LargeButtonArea);
            GUILayout.FlexibleSpace();

            EditorGUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
            var blockData = block as BlockData;
            if (canBeAdded)
            {
                var addIcon = Styles.AddIcon;
                if (ShowLargeButton(block.Id, addIcon))
                {
                    block.AddToProject(null, block.RequireListRefreshAfterInstall ? RefreshBlockList : (Action)null);
                }
            }

            if (canBeSelected)
            {
                if (ShowLargeButton(block.Id, Styles.SelectIcon))
                {
                    blockData.SelectBlocksInScene();
                }
            }

            EditorGUILayout.EndHorizontal();
            GUILayout.EndArea();
        }

        private void ShowDescription(BlockBaseData block, Rect blockRect, float targetHeight, int expectedThumbnailWidth, int expectedThumbnailHeight, bool canBeAdded, bool canBeSelected)
        {
            var blockData = block as BlockData;

            var hoverDescription = OVREditorUtils.HoverHelper.IsHover(block.Id + "Description");
            var descriptionStyle = new GUIStyle(hoverDescription ? Styles.DescriptionAreaHoverStyle : Styles.DescriptionAreaStyle);
            descriptionStyle.fixedHeight += expectedThumbnailHeight - targetHeight;
            var descriptionArea = EditorGUILayout.BeginVertical(descriptionStyle);
            hoverDescription = OVREditorUtils.HoverHelper.IsHover(block.Id + "Description", Event.current, descriptionArea);
            EditorGUILayout.BeginHorizontal();
            var descriptionRect = blockRect;
            descriptionRect.y += targetHeight + 2;
            descriptionRect.height -= targetHeight + Styles.Padding + 2;
            GUILayout.BeginArea(descriptionRect);
            var numberOfIcons = 0;
            if (canBeAdded) numberOfIcons++;
            if (canBeSelected) numberOfIcons++;
            var iconWidth = Styles.LargeButton.fixedWidth + Styles.LargeButton.margin.horizontal;
            var padding = descriptionStyle.padding.horizontal;
            var style = new GUIStyle(Styles.EmptyAreaStyle);
            style.fixedWidth = expectedThumbnailWidth - padding - numberOfIcons * iconWidth;
            style.fixedHeight = descriptionStyle.fixedHeight;
            style.padding = new RectOffset(Styles.BlockMargin, Styles.BlockMargin, Styles.BlockMargin, Styles.BlockMargin);
            EditorGUILayout.BeginVertical(style);
            EditorGUILayout.BeginHorizontal();
            var labelStyle = hoverDescription ? Styles.LabelHoverStyle : Styles.LabelStyle;
            EditorGUILayout.LabelField(block.BlockName, labelStyle);
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.LabelField(block.Description, Styles.InfoStyle);
            EditorGUILayout.BeginHorizontal();
            ShowTagList(block.Id, block.Tags, _tagSearch, Tag.TagListType.Filters);
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();
            GUILayout.EndArea();
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();
        }

#if OVR_BB_DRAGANDDROP
        private void ShowDragAndDrop(BlockBaseData block, Rect blockRect, bool canBeAdded)
        {
            var hoverGrid = OVREditorUtils.HoverHelper.IsHover(block.Id + "Grid", Event.current, blockRect);
            if (canBeAdded)
            {
                if (hoverGrid)
                {
                    if (Event.current.type == EventType.Repaint && !isHoveringHotControl)
                    {
                        EditorGUIUtility.AddCursorRect(blockRect, MouseCursor.Pan);
                    }

                    if (Event.current.type == EventType.MouseDown)
                    {
                        SetDragAndDrop(block);
                    }
                }
            }
        }

#endif // OVR_BB_DRAGANDDROP

        private void Show(BlockBaseData block, Rect blockRect, int expectedThumbnailWidth, int expectedThumbnailHeight)
        {
            var blockData = block as BlockData;
            var canBeAdded = block.CanBeAdded;
            var numberInScene = blockData != null ? blockData.ComputeNumberOfBlocksInScene() : 0;
            var canBeSelected = numberInScene > 0;

            var expectedColor = canBeAdded ? Color.white : Styles.DisabledColor;
            using var color = new OVREditorUtils.OVRGUIColorScope(OVREditorUtils.OVRGUIColorScope.Scope.All, expectedColor);
            var gridStyle = new GUIStyle(Styles.GridItemStyleWithHover);
            var gridDisabledStyle = new GUIStyle(Styles.GridItemDisabledStyle);
            var gridItemStyle = canBeAdded ? gridStyle : gridDisabledStyle;
            gridItemStyle.fixedWidth = expectedThumbnailWidth;

            var isHover = OVREditorUtils.HoverHelper.IsHover(block.Id + "Description");
            var targetHeight = isHover ? expectedThumbnailHeight - Styles.DescriptionAreaStyle.fixedHeight : expectedThumbnailHeight;
            targetHeight = (int)OVREditorUtils.TweenHelper.GUISmooth(block.Id, targetHeight, ifNotCompletedDelegate: _repainter.RequestRepaint);

            if (isHover)
            {
                block.MarkAsSeen();
            }

            EditorGUILayout.BeginVertical(gridItemStyle);
            {
                ShowThumbnail(block, targetHeight, expectedThumbnailHeight);
                ShowDescription(block, blockRect, targetHeight, expectedThumbnailWidth, expectedThumbnailHeight, canBeAdded, canBeSelected);
                ShowButtons(block, blockRect, canBeAdded, canBeSelected);

#if OVR_BB_DRAGANDDROP
                ShowDragAndDrop(block, blockRect, canBeAdded);
#endif // OVR_BB_DRAGANDDROP
            }
            EditorGUILayout.EndVertical();
        }

        private bool ShowTagList(string controlId, IEnumerable<Tag> tagArray, HashSet<Tag> search, Tag.TagListType listType)
        {
            var any = false;
            foreach (var tag in tagArray)
            {
                any |= ShowTag(controlId + "list", tag, search, listType);
            }

            return any;
        }

        private bool ShowTag(string controlId, Tag tag, HashSet<Tag> search, Tag.TagListType listType)
        {
            var tagBehavior = tag.Behavior;
            if (!tagBehavior.Show)
            {
                return false;
            }

            if (!tagBehavior.Visibility)
            {
                return false;
            }

            if (!HasAnyBlock(tag))
            {
                return false;
            }

            switch (listType)
            {
                case Tag.TagListType.Filters when !tagBehavior.CanFilterBy:
                case Tag.TagListType.Overlays when !tagBehavior.ShowOverlay:
                    return false;
            }

            var style = tagBehavior.Icon != null ? Styles.TagStyleWithIcon : Styles.TagStyle;
            var backgroundColors = listType == Tag.TagListType.Overlays ? Styles.TagOverlayBackgroundColors : Styles.TagBackgroundColors;

            var tagContent = new GUIContent(tag.Name);
            var tagWidth = style.CalcSize(tagContent).x + 1;
            var rect = GUILayoutUtility.GetRect(tagContent, style, GUILayout.Width(tagWidth));
            Vector2 mousePosition = Event.current.mousePosition;
            var id = controlId + tag;
            var color = backgroundColors.GetColor(search.Contains(tag), OVREditorUtils.HoverHelper.IsHover(id));
            using (new OVREditorUtils.OVRGUIColorScope(OVREditorUtils.OVRGUIColorScope.Scope.Background, color))
            {
                using (new OVREditorUtils.OVRGUIColorScope(OVREditorUtils.OVRGUIColorScope.Scope.Content, tagBehavior.Color))

                {
                    if (OVREditorUtils.HoverHelper.Button(id, rect, tagContent, style, out var hover))
                    {
                        if (_tagSearch.Contains(tag))
                        {
                            _tagSearch.Remove(tag);
                        }
                        else
                        {
                            _tagSearch.Clear();
                            _tagSearch.Add(tag);
                        }
                    }

                    if (tagBehavior.Icon != null)
                    {
                        GUI.Label(rect, tagBehavior.Icon, Styles.TagIcon);
                    }

                    isHoveringHotControl |= hover;
                }
            }
            EditorGUIUtility.AddCursorRect(rect, MouseCursor.Link);

            return true;
        }

        private bool ShouldShowTutorial()
        {
            _shouldShowTutorial = !_tutorialCompleted.Value;
            if (_shouldShowTutorial)
            {
                // Make sure the scene doesn't have a non block version of the OVRCameraRig
                _shouldShowTutorial = !BlockData.HasNonBuildingBlockCameraRig();
            }

            return _shouldShowTutorial;
        }

        private void ShowTutorial(Rect dragArea)
        {
            if (_outline == null && OVRGUIContent.BuildPath("bb_outline.asset", OVRGUIContent.Source.BuildingBlocksAnimations, out var outlinePath))

            {
                _outline = AssetDatabase.LoadAssetAtPath<OVRAnimatedContent>(outlinePath);
            }

            if (_outline != null)
            {
                _outline.Update();
                GUI.DrawTexture(dragArea, _outline.CurrentFrame);
            }

            if (_tutorial == null && OVRGUIContent.BuildPath("bb_tutorial.asset", OVRGUIContent.Source.BuildingBlocksAnimations, out var tutorialPath))

            {
                _tutorial = AssetDatabase.LoadAssetAtPath<OVRAnimatedContent>(tutorialPath);
            }

            if (_tutorial != null)
            {
                _tutorial.Update();
                GUI.DrawTexture(dragArea, _tutorial.CurrentFrame);
            }

            _repainter.RequestRepaint();
        }

        private bool ShowLargeButton(string controlId, OVRGUIContent icon)
        {
            var previousColor = GUI.color;
            GUI.color = Color.white;
            var id = controlId + icon.Name;
            var hit = OVREditorUtils.HoverHelper.Button(id, icon, Styles.LargeButton, out var hover);
            isHoveringHotControl |= hover;
            GUI.color = previousColor;
            EditorGUIUtility.AddCursorRect(GUILayoutUtility.GetLastRect(), MouseCursor.Link);
            return hit;
        }

#if OVR_BB_DRAGANDDROP
        private void RefreshDragAndDrop(Dimensions dimensions)
        {
            var blockThumbnail = DragAndDrop.GetGenericData(DragAndDropBlockThumbnailLabel) as Texture2D;
            if (blockThumbnail)
            {
                var cursorOffset = new Vector2(dimensions.ExpectedThumbnailWidth / 2.0f, dimensions.ExpectedThumbnailHeight / 2.0f);
                var cursorRect = new Rect(Event.current.mousePosition - cursorOffset, new Vector2(dimensions.ExpectedThumbnailWidth, dimensions.ExpectedThumbnailHeight));
                GUI.color = new Color(1, 1, 1, Styles.DragOpacity);
                GUI.DrawTexture(cursorRect, blockThumbnail, ScaleMode.ScaleAndCrop);
                GUI.color = Color.white;

                // Enforce a repaint next frame, as we need to move this thumbnail everyframe
                _repainter.RequestRepaint();
            }

            if (Event.current.type == EventType.DragExited)
            {
                ResetDragThumbnail();
            }

            if (Event.current.type == EventType.DragUpdated)
            {
                DragAndDrop.visualMode = DragAndDropVisualMode.Move;
            }
        }

        private static DragAndDropVisualMode HierarchyDropHandler(
            int dropTargetInstanceID,
            HierarchyDropFlags dropMode,
            Transform parentForDraggedObjects,
            bool perform)
        {
            var hoveredObject = EditorUtility.InstanceIDToObject(dropTargetInstanceID) as GameObject;
            return DropHandler(perform, hoveredObject);
        }

        private static DragAndDropVisualMode SceneDropHandler(
            UnityEngine.Object dropUpon,
            Vector3 worldPosition,
            Vector2 viewportPosition,
            Transform parentForDraggedObjects,
            bool perform)
        {
            return DropHandler(perform, dropUpon as GameObject);
        }

        private static DragAndDropVisualMode DropHandler(bool perform, GameObject dropUpon)
        {
            var block = DragAndDrop.GetGenericData(DragAndDropBlockDataLabel) as BlockBaseData;

            if (block == null)
            {
                return DragAndDropVisualMode.None;
            }

            if (!perform)
            {
                return DragAndDropVisualMode.Generic;
            }

            if (block.OverridesInstallRoutine && Selection.objects.Contains(dropUpon))
            {
                foreach (var obj in Selection.objects)
                {
                    if (obj is GameObject gameObject)
                    {
                        block.AddToProject(gameObject);
                    }
                }
            }
            else
            {
                block.AddToProject(dropUpon);
            }

            ResetDragAndDrop();
            _tutorialCompleted.Value = true;
            _shouldShowTutorial = false;

            return DragAndDropVisualMode.Generic;
        }

        private static void SetDragAndDrop(BlockBaseData block)
        {
            DragAndDrop.PrepareStartDrag();
            DragAndDrop.SetGenericData(DragAndDropBlockDataLabel, block);
            DragAndDrop.SetGenericData(DragAndDropBlockThumbnailLabel, block.Thumbnail);
            DragAndDrop.StartDrag(DragAndDropLabel);
        }

        private static void ResetDragThumbnail()
        {
            DragAndDrop.SetGenericData(DragAndDropBlockThumbnailLabel, null);
        }

        private static void ResetDragAndDrop()
        {
            DragAndDrop.SetGenericData(DragAndDropBlockDataLabel, null);
        }
#endif // OVR_BB_DRAGANDDROP
    }
}
