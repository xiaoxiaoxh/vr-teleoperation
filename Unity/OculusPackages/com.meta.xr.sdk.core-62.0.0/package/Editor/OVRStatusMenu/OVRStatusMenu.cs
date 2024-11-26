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
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

internal class OVRStatusMenu : EditorWindow
{
    public struct Item
    {
        public delegate (string, Color?) TextDelegate();
        public delegate (OVRGUIContent, Color?) PillIconDelegate();
        public void OnClick() => OnClickDelegate?.Invoke();

        public string Name;
        public Color Color;
        public int Order;
        public OVRGUIContent Icon;
        public PillIconDelegate PillIcon;
        public TextDelegate InfoTextDelegate;
        public Action OnClickDelegate;
    }

    internal class Styles
    {
        internal const float Width = 360;
        internal const int LeftMargin = 4;
        internal const int Border = 1;
        internal const int Padding = 4;
        internal const float ItemHeight = 48.0f;

        public static readonly Color LightGray = OVREditorUtils.HexToColor("#aaaaaa");

        internal readonly GUIStyle BackgroundAreaStyle = new GUIStyle()
        {
            stretchHeight = true,
            padding = new RectOffset(Border, Border, Border, Border),
            normal =
            {
                background = OVREditorUtils.MakeTexture(1, 1, OVREditorUtils.HexToColor("#1d1d1d"))
            }
        };

        internal readonly GUIStyle DescriptionAreaStyle = new GUIStyle()
        {
            stretchHeight = true,
            fixedHeight = Styles.ItemHeight,
            padding = new RectOffset(LeftMargin + Padding, Padding, Padding, Padding),
            margin = new RectOffset(0, 0, 0, Border),

            normal =
            {
                background = OVREditorUtils.MakeTexture(1, 1, OVREditorUtils.HexToColor("#3e3e3e"))
            },
            hover =
            {
                background = OVREditorUtils.MakeTexture(1, 1, OVREditorUtils.HexToColor("#4e4e4e"))
            }
        };

        internal readonly GUIStyle LabelStyle = new GUIStyle(EditorStyles.boldLabel);

        internal readonly GUIStyle LabelHoverStyle = new GUIStyle(EditorStyles.boldLabel)
        {
            normal = { textColor = Color.white }
        };

        internal readonly GUIStyle SubtitleStyle = new GUIStyle(EditorStyles.label)
        {
            fontStyle = FontStyle.Italic
        };

        internal readonly GUIStyle IconStyle = new GUIStyle(EditorStyles.label)
        {
            fixedWidth = 48 - Padding * 2,
            fixedHeight = 48 - Padding * 2,
            stretchHeight = true,
            padding = new RectOffset(8, 8, 8, 8),
        };

        internal readonly GUIStyle PillIconStyle = new GUIStyle(EditorStyles.label)
        {
            fixedWidth = 22,
            fixedHeight = 22,
            stretchHeight = true,
            padding = new RectOffset(0, 0, 0, 0),
        };

        internal readonly GUIStyle StatusIconStyle = new GUIStyle("StatusBarIcon");

        internal readonly GUIStyle StatusPillIconStyle = new GUIStyle(EditorStyles.label)
        {
            fixedWidth = 10,
            fixedHeight = 10,
            stretchHeight = true,
            padding = new RectOffset(0, 0, 0, 0),
        };

        internal readonly OVRGUIContent StatusIcon =
            OVREditorUtils.CreateContent("ovr_icon_meta.png", OVRGUIContent.Source.GenericIcons, null);

        internal readonly OVRGUIContent StatusPillIcon =
            OVREditorUtils.CreateContent("ovr_icon_pill.png", OVRGUIContent.Source.GenericIcons, null);
    }

    private static Styles _styles;
    internal static Styles styles => _styles ??= new Styles();
    private static readonly List<Item> Items = new List<Item>();
    private static OVRStatusMenu _instance;

    public static List<Item> RegisteredItems => Items;

    public static void RegisterItem(Item item)
    {
        Items.Add(item);
        Items.Sort((x, y) => x.Order.CompareTo(y.Order));
    }

    public static Item GetHighestItem()
    {
        foreach (var item in Items)
        {
            var (_, color) = item.PillIcon?.Invoke() ?? default;
            if (color.HasValue)
            {
                return item;
            }
        }

        return default;
    }

    public static void ShowDropdown(Vector2 position)
    {
        if (_instance != null)
        {
            _instance.Close();
        }

        if (Items.Count == 0)
        {
            return;
        }

        _instance = CreateInstance<OVRStatusMenu>();
        _instance.ShowAsDropDown(new Rect(position, Vector2.zero), new Vector2(Styles.Width, _instance.ComputeHeight()));
        _instance.wantsMouseMove = true;
        _instance.Focus();
    }

    private float ComputeHeight()
    {
        return Styles.ItemHeight * Items.Count + 2;
    }

    private void OnGUI()
    {
        EditorGUILayout.BeginVertical(styles.BackgroundAreaStyle);
        {
            foreach (var item in Items)
            {
                ShowItem(item);
            }
        }
        EditorGUILayout.EndVertical();

        if (Event.current.type == EventType.MouseMove)
        {
            Repaint();
        }
    }

    private void ShowItem(Item item)
    {
        var buttonRect = EditorGUILayout.BeginVertical(styles.DescriptionAreaStyle);
        var hover = buttonRect.Contains(Event.current.mousePosition);
        {
            var rect = EditorGUILayout.BeginHorizontal();
            {
                ShowIcon(item, rect);
                ShowLabel(item, hover);
            }
            EditorGUILayout.EndHorizontal();
        }
        EditorGUILayout.EndVertical();

        var leftMarginRect = buttonRect;
        leftMarginRect.width = Styles.LeftMargin;
        EditorGUI.DrawRect(leftMarginRect, item.Color);
        EditorGUIUtility.AddCursorRect(buttonRect, MouseCursor.Link);
        if (hover && Event.current.type == EventType.MouseUp)
        {
            item.OnClick();
            Close();
        }
    }

    private void ShowLabel(Item item, bool hover)
    {
        EditorGUILayout.BeginVertical();
        {
            EditorGUILayout.LabelField(item.Name, hover ? styles.LabelHoverStyle : styles.LabelStyle);
            ShowInfoText(item);
        }
        EditorGUILayout.EndVertical();
    }

    private void ShowInfoText(Item item)
    {
        if (item.InfoTextDelegate == null) return;

        var (content, color) = item.InfoTextDelegate();
        var style = new GUIStyle(styles.SubtitleStyle);
        style.normal.textColor = color ?? Styles.LightGray;
        EditorGUILayout.LabelField(content, style);
    }

    private void ShowIcon(Item item, Rect rect)
    {
        EditorGUILayout.LabelField(item.Icon, styles.IconStyle, GUILayout.Width(Styles.ItemHeight));
        ShowPill(item, rect);
    }

    private void ShowPill(Item item, Rect rect)
    {
        if (item.PillIcon == null) return;

        var (content, color) = item.PillIcon();

        if (content == null) return;

        rect.x += 16;
        rect.y += 2;
        rect.width = styles.PillIconStyle.fixedWidth;
        rect.height = styles.PillIconStyle.fixedHeight;
        using (new OVREditorUtils.OVRGUIColorScope(OVREditorUtils.OVRGUIColorScope.Scope.Content, color ?? Color.white))
        {
            GUI.Label(rect, content, styles.PillIconStyle);
        }
    }
}
