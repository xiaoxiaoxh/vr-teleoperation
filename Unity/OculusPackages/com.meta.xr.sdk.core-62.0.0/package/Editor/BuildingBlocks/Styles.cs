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

using UnityEditor;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{
    internal static class Styles
    {
        public static class Colors
        {
            public static readonly Color AccentColor = OVREditorUtils.HexToColor("#a29de5");
            public static readonly Color ExperimentalColor = OVREditorUtils.HexToColor("#eba333");
            public static readonly Color InternalColor = OVREditorUtils.HexToColor("#66aaff");
            public static readonly Color NewColor = OVREditorUtils.HexToColor("#ffc75d");
            public static readonly Color LightGray = OVREditorUtils.HexToColor("#aaaaaa");
            public static readonly Color DarkGray = OVREditorUtils.HexToColor("#3e3e3e");
            public static readonly Color DarkGraySemiTransparent = OVREditorUtils.HexToColor("#3e3e3eaa");
            public static readonly Color DarkGrayHover = OVREditorUtils.HexToColor("#4e4e4e");
            public static readonly Color DarkGrayActive = OVREditorUtils.HexToColor("#5d5d5d");
            public static readonly Color DarkBlue = OVREditorUtils.HexToColor("#48484d");
            public static readonly Color CharcoalGray = OVREditorUtils.HexToColor("#1d1d1d");
            public static readonly Color CharcoalGraySemiTransparent = OVREditorUtils.HexToColor("#1d1d1d80");
            public static readonly Color OffWhite = OVREditorUtils.HexToColor("#dddddd");
            public static readonly Color ErrorColor = OVREditorUtils.HexToColor("ed5757");
            public static readonly Color ErrorColorSemiTransparent = OVREditorUtils.HexToColor("ed575780");
        }

        public const float ThumbnailRatio = 1.8f;
        public const int Border = 1;
        public const float SmallIconSize = 16.0f;
        public const float ItemHeight = 48.0f;
        public const int Padding = 4;
        public const int TightPadding = 2;
        public const int BlockMargin = 8;
        public const int IdealThumbnailWidth = 280;
        public const int DescriptionHeight = 48;

#if OVR_BB_DRAGANDDROP
        public const float DragOpacity = 0.5f;
        public static readonly Color DragColor = new Color(1.0f, 1.0f, 1.0f, DragOpacity);
#endif // OVR_BB_DRAGANDDROP

        public const float DisabledTint = 0.6f;
        public static readonly Color DisabledColor = new Color(DisabledTint, DisabledTint, DisabledTint, 1.0f);

        private static Texture2D _infoExperimentalBgTexture;


        public static readonly GUIStyle NoMargin = new GUIStyle()
        {
            margin = new RectOffset(0, 0, 0, 0),
            padding = new RectOffset(0, 0, 0, 0)

        };

        public static readonly GUIStyle ErrorHelpBox = new GUIStyle(EditorStyles.helpBox)
        {
            richText = true,
            fontSize = 12,
            margin = new RectOffset(Styles.BlockMargin, Styles.BlockMargin, Styles.BlockMargin,
                Styles.BlockMargin),
            padding = new RectOffset(Styles.BlockMargin, Styles.BlockMargin, Styles.BlockMargin,
                Styles.BlockMargin),
            normal = { textColor = Styles.Colors.ErrorColor }
        };


        public static readonly GUIStyle GridItemStyle = new GUIStyle()
        {
            margin = new RectOffset(BlockMargin, BlockMargin, BlockMargin, BlockMargin),
            padding = new RectOffset(Border, Border, Border, Border),
            stretchWidth = false,
            stretchHeight = false,
            normal =
            {
                background = OVREditorUtils.MakeTexture(1, 1, Colors.CharcoalGray)
            }
        };

        public static readonly GUIStyle GridItemStyleWithHover = new GUIStyle()
        {
            margin = new RectOffset(BlockMargin, BlockMargin, 0, BlockMargin),
            padding = new RectOffset(Border, Border, Border, Border),
            stretchWidth = false,
            stretchHeight = false,
            normal =
            {
                background = OVREditorUtils.MakeTexture(1, 1, Colors.CharcoalGray)
            },
            hover =
            {
                background = OVREditorUtils.MakeTexture(1, 1, Styles.Colors.AccentColor)
            }
        };

        public static readonly GUIStyle BlocksWindowGridItemStyle = new GUIStyle()
        {
            margin = new RectOffset(BlockMargin, BlockMargin, BlockMargin, BlockMargin),
            padding = new RectOffset(Border, Border, Border, Border),
            stretchWidth = false,
            stretchHeight = false,
            normal =
            {
                background = OVREditorUtils.MakeTexture(1, 1, Colors.CharcoalGray)
            },
            hover =
            {
                background = OVREditorUtils.MakeTexture(1, 1, Colors.AccentColor)
            }
        };

        public static readonly GUIStyle GridItemDisabledStyle = new GUIStyle()
        {
            margin = new RectOffset(BlockMargin, BlockMargin, 0, BlockMargin),
            padding = new RectOffset(Border, Border, Border, Border),
            stretchWidth = false,
            stretchHeight = false,
            normal =
            {
                background = OVREditorUtils.MakeTexture(1, 1, Colors.CharcoalGray)
            }
        };

        public static readonly GUIStyle ThumbnailAreaStyle = new GUIStyle()
        {
            stretchHeight = false
        };

        public static readonly GUIStyle SeparatorAreaStyle = new GUIStyle()
        {
            fixedHeight = Border,
            stretchHeight = false,
            normal =
            {
                background = OVREditorUtils.MakeTexture(1, 1, Colors.DarkGray)
            }
        };

        public static readonly GUIStyle DescriptionAreaStyle = new GUIStyle()
        {
            stretchHeight = false,
            padding = new RectOffset(Padding, Padding, Padding, Padding),
            margin = new RectOffset(0, 0, 0, Border),
            fixedHeight = ItemHeight,
            normal =
            {
                background = OVREditorUtils.MakeTexture(1, 1, Colors.DarkGray)
            }
        };

        public static readonly GUIStyle DescriptionAreaHoverStyle = new GUIStyle()
        {
            stretchHeight = false,
            fixedHeight = Styles.DescriptionHeight,
            padding = new RectOffset(Padding, Padding, Padding, Padding),
            margin = new RectOffset(0, 0, 0, Border),

            normal =
            {
                background = OVREditorUtils.MakeTexture(1, 1, Colors.DarkGrayHover)
            }
        };

        public static readonly GUIStyle EmptyAreaStyle = new GUIStyle()
        {
            stretchHeight = true,
            fixedWidth = 0,
            fixedHeight = Styles.DescriptionHeight,
            padding = new RectOffset(0, 0, 0, 0),
            margin = new RectOffset(0, 0, 0, 0),
        };

        public static readonly GUIStyle IconStyle = new GUIStyle(EditorStyles.label)
        {
            margin = new RectOffset(0, 0, 0, 0),
            padding = new RectOffset(0, 0, 0, 0),
            fixedWidth = SmallIconSize,
            stretchWidth = false
        };

        public static readonly GUIStyle MiniButton = new GUIStyle(EditorStyles.miniButton)
        {
            clipping = TextClipping.Overflow,
            fixedHeight = 18.0f,
            fixedWidth = 18.0f,
            margin = new RectOffset(2, 2, 2, 2),
            padding = new RectOffset(2, 2, 2, 2)
        };

        public static readonly GUIStyle LargeButton = new GUIStyle(EditorStyles.miniButton)
        {
            clipping = TextClipping.Overflow,
            fixedHeight = ItemHeight - Padding * 2,
            fixedWidth = ItemHeight - Padding * 2,
            margin = new RectOffset(2, 2, 2, 2),
            padding = new RectOffset(Padding, Padding, Padding, Padding)
        };

        public static readonly GUIStyle LargeButtonArea = new GUIStyle(Styles.EmptyAreaStyle)
        {
            padding = new RectOffset(Styles.Padding, Styles.Padding + 1, Styles.Padding, Styles.Padding + 1)
        };

        public static readonly GUIStyle IssuesTitleLabel = new GUIStyle(EditorStyles.label)
        {
            fontSize = 14,
            wordWrap = false,
            stretchWidth = false,
            fontStyle = FontStyle.Bold,
            padding = new RectOffset(10, 10, 0, 0)
        };

        public static readonly GUIStyle LabelStyle = new GUIStyle(EditorStyles.boldLabel);

        public static readonly GUIStyle LabelStyleWrapped = new GUIStyle(EditorStyles.boldLabel)
        {
            wordWrap = true,
            richText = true
        };

        public static readonly GUIStyle LabelHoverStyle = new GUIStyle(EditorStyles.boldLabel)
        {
            normal = { textColor = Color.white }
        };

        public static readonly GUIStyle Header = new GUIStyle(EditorStyles.miniLabel)
        {
            fontSize = 12,
            fixedHeight = 32 + BlockMargin * 2,
            padding = new RectOffset(BlockMargin, BlockMargin, BlockMargin, BlockMargin),
            margin = new RectOffset(0, 0, 0, 0),

            wordWrap = true,
            normal =
            {
                background = OVREditorUtils.MakeTexture(1, 1, Colors.DarkGray)
            }
        };

        public static readonly GUIStyle HeaderIconStyle = new GUIStyle()
        {
            fixedHeight = 32.0f,
            fixedWidth = 32.0f,
            stretchWidth = false,
            alignment = TextAnchor.MiddleCenter
        };

        public static readonly GUIStyle SubtitleStyle = new GUIStyle(EditorStyles.label)
        {
            fontStyle = FontStyle.Italic,
            normal =
            {
                textColor = Color.gray
            }
        };

        public static readonly GUIStyle SubtitleHelpText = new GUIStyle(EditorStyles.miniLabel)
        {
            fontSize = 12,
            margin = new RectOffset(BlockMargin, BlockMargin, BlockMargin, BlockMargin),
            wordWrap = true
        };

        public static readonly GUIStyle InternalHelpBox = new GUIStyle(EditorStyles.helpBox)
        {
            margin = new RectOffset(BlockMargin, BlockMargin, BlockMargin, BlockMargin)
        };

        public static readonly GUIStyle InternalHelpText = new GUIStyle(EditorStyles.miniLabel)
        {
            margin = new RectOffset(10, 0, 0, 0),
            wordWrap = true,
            fontStyle = FontStyle.Italic,
            normal =
            {
                textColor = new Color(0.58f, 0.72f, 0.95f)
            }
        };

        public static readonly GUIStyle BoldLabel = new GUIStyle(EditorStyles.boldLabel)
        {
            stretchHeight = true,
            fixedHeight = 32,
            fontSize = 16,
            normal =
            {
                textColor = Color.white
            },
            hover =
            {
                textColor = Color.white
            },
            alignment = TextAnchor.MiddleLeft
        };

        public static readonly GUIStyle InfoStyle = new GUIStyle(EditorStyles.label)
        {
            fontSize = 10,
            wordWrap = true,
            normal =
            {
                textColor = Color.gray
            }
        };

        public static readonly GUIStyle InfoStyleLightGray = new GUIStyle(EditorStyles.label)
        {
            fontSize = 10,
            wordWrap = true,
            normal =
            {
                textColor = Colors.LightGray
            }
        };

        public static Texture2D InfoExperimentalBgTexture
        {

            get
            {
                if (_infoExperimentalBgTexture != null)
                {
                    return _infoExperimentalBgTexture;
                }

                _infoExperimentalBgTexture = new Texture2D(1, 1);
                _infoExperimentalBgTexture.SetPixel(0, 0, new Color(0.22f, 0.22f, 0.22f, 0.25f));
                _infoExperimentalBgTexture.Apply();

                return _infoExperimentalBgTexture;
            }
        }

        public static readonly GUIStyle FoldoutBoldLabel = new GUIStyle(EditorStyles.foldout)
        {
            fontStyle = FontStyle.Bold
        };

        public static readonly OVRGUIContent AddIcon =
            OVREditorUtils.CreateContent("ovr_icon_addblock.png", OVRGUIContent.Source.BuildingBlocksIcons, "Add Block to current scene");

        public static readonly OVRGUIContent DownloadIcon =
            OVREditorUtils.CreateContent("ovr_icon_download.png", OVRGUIContent.Source.BuildingBlocksIcons, "Download Block to your project");

        public static readonly OVRGUIContent SelectIcon =
            OVREditorUtils.CreateContent("ovr_icon_link.png", OVRGUIContent.Source.BuildingBlocksIcons, "Select Block in current scene");

        public static readonly OVRGUIContent ConfigIcon =
            OVREditorUtils.CreateContent("ovr_icon_cog.png", OVRGUIContent.Source.GenericIcons, "Additional options");

        public static readonly OVRGUIContent DocumentationIcon =
            OVREditorUtils.CreateContent("ovr_icon_documentation.png", OVRGUIContent.Source.GenericIcons, "Go to Documentation");

        public static readonly OVRGUIContent ErrorIcon =
            OVREditorUtils.CreateContent("ovr_error_greybg.png", OVRGUIContent.Source.BuildingBlocksIcons);

        public static readonly OVRGUIContent InfoIcon =
            OVREditorUtils.CreateContent("ovr_info_greybg.png", OVRGUIContent.Source.BuildingBlocksIcons);

        public static readonly OVRGUIContent SuccessIcon =
            OVREditorUtils.CreateContent("ovr_success_greybg.png", OVRGUIContent.Source.BuildingBlocksIcons);


        public static readonly OVRGUIContent HeaderIcon =
            OVREditorUtils.CreateContent("ovr_icon_bbw.png", OVRGUIContent.Source.BuildingBlocksIcons, null);

        public class ColorStates
        {
            public Color Normal;
            public Color Hover;
            public Color Active;

            public Color GetColor(bool active, bool hover)
            {
                return hover ? Hover : active ? Active : Normal;
            }
        }

        public static readonly GUIStyle TagIcon = new GUIStyle(EditorStyles.miniLabel)
        {
            padding = new RectOffset(Padding, Padding, TightPadding, TightPadding),

            alignment = TextAnchor.MiddleCenter,
            fixedWidth = 22,
            fixedHeight = 18
        };

        public static readonly ColorStates TagBackgroundColors = new ColorStates()
        {
            Normal = Colors.CharcoalGray,
            Hover = Colors.DarkGray,
            Active = Colors.DarkGrayActive
        };

        private static readonly OVRGUIContent TagBackground =
            OVREditorUtils.CreateContent("ovr_bg_radius4.png", OVRGUIContent.Source.BuildingBlocksIcons, null);

        public static readonly GUIStyle TagStyle = new GUIStyle(EditorStyles.miniLabel)
        {
            margin = new RectOffset(TightPadding, TightPadding, TightPadding, TightPadding),
            padding = new RectOffset(BlockMargin, BlockMargin, TightPadding, TightPadding),

            wordWrap = false,
            stretchWidth = false,
            fixedHeight = 18,
            fontSize = 10,
            fontStyle = FontStyle.Bold,
            normal =
            {
                textColor = Colors.OffWhite,
                background = TagBackground.Content.image as Texture2D
            },
            hover =
            {
                textColor = Color.white,
                background = TagBackground.Content.image as Texture2D
            },
            border = new RectOffset(6, 6, 6, 6)
        };

        public static readonly GUIStyle TagStyleWithIcon = new GUIStyle(TagStyle)
        {
            padding = new RectOffset((int)TagStyle.fixedHeight + Padding, Padding, TightPadding, TightPadding)

        };

        public static readonly ColorStates TagOverlayBackgroundColors = new ColorStates()
        {
            Normal = Colors.CharcoalGraySemiTransparent,
            Hover = Colors.DarkGraySemiTransparent,
            Active = Colors.CharcoalGraySemiTransparent
        };

        public static readonly GUIStyle FilterByLine = new GUIStyle()
        {
            margin = new RectOffset(0, 0, BlockMargin, BlockMargin),
            padding = new RectOffset(BlockMargin + Border, BlockMargin + Border, Padding, Padding),
            stretchWidth = true,
            stretchHeight = false,
            normal =
            {
                background = OVREditorUtils.MakeTexture(1, 1, OVREditorUtils.HexToColor("#3e3e3e"))
            }
        };
    }
}
