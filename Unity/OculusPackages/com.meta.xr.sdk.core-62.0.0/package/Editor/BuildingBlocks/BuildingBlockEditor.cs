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
using Meta.XR.Editor.Tags;
using UnityEditor;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{
    [CustomEditor(typeof(BuildingBlock))]
    public class BuildingBlockEditor : UnityEditor.Editor
    {
        internal BuildingBlock _block;
        internal BlockData _blockData;

        private bool _foldoutInstruction = true;

        public override void OnInspectorGUI()
        {
            _block = target as BuildingBlock;
            _blockData = _block.GetBlockData();

            if (_blockData == null)
            {
                return;
            }

            ShowThumbnail();
            ShowBlock(_blockData, _block, false, false, true);
            ShowTagList(_blockData.Tags, Tag.TagListType.Filters);
            ShowBlockDataList("Dependencies", _blockData.GetAllDependencyDatas());
            ShowBlockList("Used by", _blockData.GetUsingBlocksInScene());
            ShowInstructions();

        }

        internal void ShowVersionInfo()
        {
            EditorGUILayout.LabelField("Version", EditorStyles.boldLabel);

            var blockVersion = _block ? _block.Version : 0;
            var currentVersionStr = $"Current version: {blockVersion}.";
            if (_blockData.IsUpdateAvailableForBlock(_block))
            {
                EditorGUILayout.LabelField($"{currentVersionStr} Newest version: {_blockData.Version}.",
                    Styles.InfoStyle);

                if (GUILayout.Button($"Update to latest version ({_blockData.Version})"))
                {
                    if (EditorUtility.DisplayDialog("Confirmation",
                            "Any changes done to this block will be lost. Do you want to proceed?", "Yes", "No"))
                    {
                        _blockData.UpdateBlockToLatestVersion(_block);
                    }
                }
            }
            else
            {
                EditorGUILayout.LabelField($"{currentVersionStr} Block is up to date", Styles.InfoStyle);
            }
        }

        internal void ShowInstructions()
        {
            if (!string.IsNullOrEmpty(_blockData.UsageInstructions))
            {
                EditorGUILayout.Space();
                _foldoutInstruction =
                    EditorGUILayout.Foldout(_foldoutInstruction, "Block instructions", Styles.FoldoutBoldLabel);
                if (_foldoutInstruction)
                {
                    EditorGUILayout.LabelField(_blockData.UsageInstructions, EditorStyles.helpBox);
                }
            }
        }

        internal void ShowThumbnail()
        {
            var currentWidth = EditorGUIUtility.currentViewWidth;
            var expectedHeight = currentWidth / Styles.ThumbnailRatio;
            expectedHeight *= 0.5f;

            // Thumbnail
            var rect = GUILayoutUtility.GetRect(currentWidth, expectedHeight);
            rect.x -= 20;
            rect.width += 40;
            rect.y -= 4;
            GUI.DrawTexture(rect, _blockData.Thumbnail, ScaleMode.ScaleAndCrop);

            GUILayout.BeginArea(new Rect(Styles.TagStyle.margin.left,
                Styles.TagStyle.margin.top, currentWidth, expectedHeight));
            ShowTagList(_blockData.Tags, Tag.TagListType.Overlays);
            GUILayout.EndArea();

            // Separator
            rect = GUILayoutUtility.GetRect(currentWidth, 1);
            rect.x -= 20;
            rect.width += 40;
            rect.y -= 4;
            GUI.DrawTexture(rect, OVREditorUtils.MakeTexture(1, 1, Styles.Colors.AccentColor),
                ScaleMode.ScaleAndCrop);
        }

        internal void ShowTagList(IEnumerable<Tag> tagArray, Tag.TagListType listType)
        {
            EditorGUILayout.BeginHorizontal();
            foreach (var tag in tagArray)
            {
                ShowTag(tag, listType);
            }
            EditorGUILayout.EndHorizontal();
        }

        private void ShowTag(Tag tag, Tag.TagListType listType)
        {
            var tagBehavior = tag.Behavior;
            if (!tagBehavior.Show)
            {
                return;
            }

            switch (listType)
            {
                case Tag.TagListType.Filters when !tagBehavior.CanFilterBy:
                case Tag.TagListType.Overlays when !tagBehavior.ShowOverlay:
                    return;
            }

            var style = tagBehavior.Icon != null ? Styles.TagStyleWithIcon : Styles.TagStyle;
            var backgroundColors = listType == Tag.TagListType.Overlays ? Styles.TagOverlayBackgroundColors : Styles.TagBackgroundColors;

            var tagContent = new GUIContent(tag.Name);
            var tagSize = style.CalcSize(tagContent);
            var rect = GUILayoutUtility.GetRect(tagContent, style, GUILayout.MinWidth(tagSize.x + 1));
            Vector2 mousePosition = Event.current.mousePosition;
            var color = backgroundColors.GetColor(false, false);
            using (new OVREditorUtils.OVRGUIColorScope(OVREditorUtils.OVRGUIColorScope.Scope.Background, color))
            {
                using (new OVREditorUtils.OVRGUIColorScope(OVREditorUtils.OVRGUIColorScope.Scope.Content, tagBehavior.Color))

                {
                    if (GUI.Button(rect, tagContent, style))
                    {

                    }

                    if (tagBehavior.Icon != null)
                    {
                        GUI.Label(rect, tagBehavior.Icon, Styles.TagIcon);
                    }
                }
            }
        }

        private bool ShowLargeButton(GUIContent icon)
        {
            var previousColor = GUI.color;
            GUI.color = Color.white;
            var hit = GUILayout.Button(icon, Styles.LargeButton);
            GUI.color = previousColor;
            EditorGUIUtility.AddCursorRect(GUILayoutUtility.GetLastRect(), MouseCursor.Link);
            return hit;
        }

        internal void ShowBlockDataList(string name, List<BlockData> list)
        {
            EditorGUILayout.LabelField(name, EditorStyles.boldLabel);

            if (list.Count == 0)
            {
                EditorGUILayout.LabelField("No dependency blocks are required.", EditorStyles.helpBox);
            }
            else
            {
                foreach (var dependency in list)
                {
                    ShowBlock(dependency, null, true, true, false);
                }
            }
        }

        internal void ShowBlockList(string name, List<BuildingBlock> list)
        {
            EditorGUILayout.LabelField(name, EditorStyles.boldLabel);

            if (list.Count == 0)
            {
                EditorGUILayout.LabelField("No dependency blocks are required.", EditorStyles.helpBox);
            }
            else
            {
                foreach (var dependency in list)
                {
                    ShowBlock(null, dependency, true, true, false);
                }
            }
        }

        internal void ShowBlock(BlockData data, BuildingBlock block, bool asGridItem,
            bool showAction, bool showBuildingBlock)
        {
            var previousIndent = EditorGUI.indentLevel;
            EditorGUI.indentLevel = 0;

            data = data ? data : block.GetBlockData();
            block = block ? block : data.GetBlock();

            // Thumbnail
            if (asGridItem)
            {
                var gridStyle = new GUIStyle(Styles.GridItemStyle);
                gridStyle.margin = new RectOffset(0, 0, 0, 0);
                EditorGUILayout.BeginHorizontal(gridStyle);
                EditorGUILayout.BeginHorizontal(Styles.DescriptionAreaStyle);

                var expectedSize = Styles.ItemHeight;
                var rect = GUILayoutUtility.GetRect(0, expectedSize);
                rect.y -= Styles.Padding;
                rect.x -= Styles.Padding;
                rect.width = Styles.ItemHeight;
                GUI.DrawTexture(rect, data.Thumbnail, ScaleMode.ScaleAndCrop);

                EditorGUILayout.Space(Styles.ItemHeight - Styles.Padding - Styles.SmallIconSize * 0.5f - 2);

                EditorGUILayout.LabelField(block != null ? Styles.SuccessIcon : Styles.ErrorIcon, Styles.IconStyle,
                    GUILayout.Width(Styles.SmallIconSize), GUILayout.Height(Styles.ItemHeight - Styles.Padding * 2));
            }
            else
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.BeginHorizontal();
            }

            // Label
            EditorGUILayout.BeginVertical();
            EditorGUILayout.BeginHorizontal();
            var labelStyle = Styles.LabelStyle;
            EditorGUILayout.LabelField(data.BlockName, labelStyle);
            labelStyle = Styles.SubtitleStyle;
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.LabelField(block ? block.name : "Not Installed", Styles.InfoStyle);
            EditorGUILayout.EndVertical();

            GUILayout.FlexibleSpace();

            if (showAction)
            {
                if (block != null)
                {
                    if (ShowLargeButton(Utils.GotoIcon))
                    {
                        data.SelectBlocksInScene();
                    }
                }
                else
                {
                    if (ShowLargeButton(Utils.AddIcon))
                    {
                        data.AddToProject();
                    }
                }
            }

            if (showBuildingBlock && ShowLargeButton(Utils.StatusIcon))
            {
                BuildingBlocksWindow.ShowWindow("BuildingBlockEditor");
            }

            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndHorizontal();

            // Only for dependency block(s)
            if (!showBuildingBlock)
            {
                AddBlockHighlightListeners(block);
            }

            EditorGUI.indentLevel = previousIndent;
        }

        private static void AddBlockHighlightListeners(BuildingBlock buildingBlock)
        {
            Rect rect = GUILayoutUtility.GetLastRect();
            EditorGUIUtility.AddCursorRect(rect, MouseCursor.Link);

            Event currentEvent = Event.current;
            if (currentEvent.type == EventType.MouseDown && currentEvent.button == 0 &&
                rect.Contains(currentEvent.mousePosition))
            {
                buildingBlock.HighlightBlockInScene();
                if (currentEvent.clickCount == 2)
                    buildingBlock.SelectBlockInScene();

                currentEvent.Use();
            }
        }
    }
}
