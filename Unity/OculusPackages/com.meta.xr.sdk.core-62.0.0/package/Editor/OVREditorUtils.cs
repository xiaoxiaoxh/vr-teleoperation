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

[InitializeOnLoad]
internal static class OVREditorUtils
{
    internal const string MetaXRPublicName = "Meta XR";
    internal static readonly string MetaXRSettingsName = $"{MetaXRPublicName} Settings";

    internal static double LastUpdateTime;
    internal static float DeltaTime { get; private set; }

    static OVREditorUtils()
    {
        EditorApplication.update -= UpdateEditor;
        EditorApplication.update += UpdateEditor;

        OVRGUIContent.RegisterContentPath(OVRGUIContent.Source.GenericIcons, "Icons");

        var statusItem = new OVRStatusMenu.Item()
        {
            Name = MetaXRSettingsName,
            Color = OVREditorUtils.HexToColor("#c4c4c4"),
            Icon = CreateContent("ovr_icon_settings.png", OVRGUIContent.Source.GenericIcons),
            InfoTextDelegate = ComputeInfoText,
            OnClickDelegate = OnStatusMenuClick,
            Order = 100
        };
        OVRStatusMenu.RegisterItem(statusItem);
    }

    internal static void UpdateEditor()
    {
        var timeSinceStartup = EditorApplication.timeSinceStartup;
        DeltaTime = (float)(timeSinceStartup - LastUpdateTime);
        LastUpdateTime = timeSinceStartup;
    }

    private static (string, Color?) ComputeInfoText() => ("Open settings menu.", null);


    private static void OnStatusMenuClick()
    {
        OVRUserSettingsProvider.OpenSettingsWindow(OVRProjectSetupSettingsProvider.Origins.Icon);
    }

    // Helper function to create a texture with a given color
    public static Texture2D MakeTexture(int width, int height, Color col)
    {
        Color[] pixels = new Color[width * height];
        for (int i = 0; i < pixels.Length; i++)
        {
            pixels[i] = col;
        }

        Texture2D result = new Texture2D(width, height);
        result.hideFlags = HideFlags.DontSave;
        result.SetPixels(pixels);
        result.Apply();

        return result;
    }

    public static Color HexToColor(string hex)
    {
        hex = hex.Replace("#", string.Empty);
        byte r = (byte)(Convert.ToInt32(hex.Substring(0, 2), 16));
        byte g = (byte)(Convert.ToInt32(hex.Substring(2, 2), 16));
        byte b = (byte)(Convert.ToInt32(hex.Substring(4, 2), 16));
        byte a = 255;

        if (hex.Length == 8)
        {
            a = (byte)(Convert.ToInt32(hex.Substring(6, 2), 16));
        }

        return new Color32(r, g, b, a);
    }

    public static string ChoosePlural(int number, string singular, string plural)
    {
        return number > 1 ? plural : singular;
    }

    public static OVRGUIContent CreateContent(string name, OVRGUIContent.Source source, string tooltip = null)
    {
        return new OVRGUIContent(name, source, tooltip);
    }

    public static bool IsUnityVersionCompatible()
    {
#if UNITY_2021_3_OR_NEWER
        return true;
#else
        return false;
#endif
    }

    public static string VersionCompatible => "2021.3";

    public static bool IsMainEditor()
    {
        // Early Return when the process service is not the Editor itself
#if UNITY_2021_1_OR_NEWER
        return (uint)UnityEditor.MPE.ProcessService.level != (uint)UnityEditor.MPE.ProcessLevel.Secondary;
#else
        return (uint)UnityEditor.MPE.ProcessService.level != (uint)UnityEditor.MPE.ProcessLevel.Slave;
#endif
    }

    public struct OVRGUIColorScope : System.IDisposable
    {
        public enum Scope
        {
            All,
            Background,
            Content
        }

        private Color _previousColor;
        private Scope _scope;

        public OVRGUIColorScope(Scope scope, Color newColor)
        {
            _scope = scope;
            _previousColor = Color.white;
            switch (scope)
            {
                case Scope.All:
                    _previousColor = GUI.color;
                    GUI.color = newColor;
                    break;
                case Scope.Background:
                    _previousColor = GUI.backgroundColor;
                    GUI.backgroundColor = newColor;
                    break;
                case Scope.Content:
                    _previousColor = GUI.contentColor;
                    GUI.contentColor = newColor;
                    break;
            }
        }

        public void Dispose()
        {
            switch (_scope)
            {
                case Scope.All:
                    GUI.color = _previousColor;
                    break;
                case Scope.Background:
                    GUI.backgroundColor = _previousColor;
                    break;
                case Scope.Content:
                    GUI.contentColor = _previousColor;
                    break;
            }
        }
    }

    public readonly struct UndoScope : System.IDisposable
    {
        private readonly int _group;
        private readonly string _name;

        public UndoScope(string name)
        {
            Undo.IncrementCurrentGroup();
            _group = Undo.GetCurrentGroup();
            _name = name;
        }

        public void Dispose()
        {
            Undo.SetCurrentGroupName(_name);
            Undo.CollapseUndoOperations(_group);
        }
    }

    public static class HoverHelper
    {
        private static readonly Dictionary<string, bool> Hovers = new Dictionary<string, bool>();

        public static void Reset()
        {
            Hovers.Clear();
        }

        public static bool IsHover(string id, Event ev = null, Rect? area = null)
        {
            if (area.HasValue && ev?.type == EventType.Repaint)
            {
                Hovers[id] = area?.Contains(ev.mousePosition) ?? false;
            }

            Hovers.TryGetValue(id, out var hover);
            return hover;
        }

        public static bool Button(string id, GUIContent content, GUIStyle style, out bool hover)
        {
            var isClicked = GUILayout.Button(content, style);
            hover = IsHover(id, Event.current, GUILayoutUtility.GetLastRect());
            return isClicked;
        }

        public static bool Button(string id, Rect rect, GUIContent content, GUIStyle style, out bool hover)
        {
            var isClicked = GUI.Button(rect, content, style);
            hover = IsHover(id, Event.current, GUILayoutUtility.GetLastRect());
            return isClicked;
        }
    }

    public static class TweenHelper
    {
        private static readonly Dictionary<string, float> Tweens = new Dictionary<string, float>();

        public static void Reset()
        {
            Tweens.Clear();
        }

        public static float GetTweenValue(string id, float target, float? start)
        {
            if (!Tweens.TryGetValue(id, out var current))
            {
                current = start ?? target;
                Tweens[id] = current;
            }

            return current;
        }

        public static float Smooth(string id,
            float target,
            out bool completed,
            float? start = null,
            float speed = 10.0f,
            float epsilon = 5.0f)
        {
            var current = GetTweenValue(id, target, start);

            if (Math.Abs(target - current) <= epsilon)
            {
                current = target;
                Tweens[id] = current;
                completed = true;
            }
            else
            {
                current = Mathf.Lerp(current, target, 1f - Mathf.Exp(-speed * DeltaTime));
                Tweens[id] = current;
                completed = false;
            }

            return current;
        }

        public static float GUISmooth(string id, float target, float? start = null,
            float speed = 10.0f, float epsilon = 5.0f, Action ifNotCompletedDelegate = null)
        {
            var shouldUpdate = Event.current.type == EventType.Layout;
            var completed = true;
            var current = shouldUpdate
                ? Smooth(id, target, out completed, start, speed, epsilon)
                : GetTweenValue(id, target, start);

            if (!completed)
            {
                ifNotCompletedDelegate?.Invoke();
            }

            return current;
        }
    }
}
