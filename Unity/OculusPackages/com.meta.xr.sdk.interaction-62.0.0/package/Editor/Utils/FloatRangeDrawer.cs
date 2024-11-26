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

using UnityEditor;
using UnityEngine;

namespace Oculus.Interaction.Editor
{
    [CustomPropertyDrawer(typeof(TransformerUtils.FloatRange))]
    public class FloatRangeDrawer : PropertyDrawer
    {
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            EditorGUI.BeginProperty(position, label, property);

            // Draw label
            position = EditorGUI.PrefixLabel(position, GUIUtility.GetControlID(FocusType.Passive), label);

            // Don't make child fields be indented
            var indent = EditorGUI.indentLevel;
            EditorGUI.indentLevel = 0;

            // Calculate rects
            var minLabelRect = new Rect(position.x, position.y, 30, position.height);
            var minFieldRect = new Rect(position.x + 30, position.y, position.width / 2 - 35, position.height);
            var maxLabelRect = new Rect(position.x + position.width / 2 + 5, position.y, 30, position.height);
            var maxFieldRect = new Rect(position.x + position.width / 2 + 35, position.y, position.width / 2 - 35, position.height);

            // Draw fields
            EditorGUI.LabelField(minLabelRect, "Min");
            EditorGUI.PropertyField(minFieldRect, property.FindPropertyRelative("Min"), GUIContent.none);
            EditorGUI.LabelField(maxLabelRect, "Max");
            EditorGUI.PropertyField(maxFieldRect, property.FindPropertyRelative("Max"), GUIContent.none);

            // Set indent back to what it was
            EditorGUI.indentLevel = indent;

            EditorGUI.EndProperty();
        }
    }
}
