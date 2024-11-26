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

    [CustomPropertyDrawer(typeof(TransformerUtils.ConstrainedAxis))]
    public class ConstrainedAxisDrawer : PropertyDrawer
    {
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            SerializedProperty constrainAxisProperty = property.FindPropertyRelative("ConstrainAxis");
            SerializedProperty axisRangeProperty = property.FindPropertyRelative("AxisRange");

            EditorGUI.BeginProperty(position, label, property);

            // Draw label
            position = EditorGUI.PrefixLabel(position, GUIUtility.GetControlID(FocusType.Passive), label);

            // Don't make child fields be indented
            int indent = EditorGUI.indentLevel;
            EditorGUI.indentLevel = 0;

            // Calculate rects
            Rect constraintRect = new Rect(position.x, position.y, 65, position.height);
            Rect toggleRect = new Rect(constraintRect.x + constraintRect.width, position.y, 20, position.height);
            Rect rangeRect = new Rect(toggleRect.x + toggleRect.width, position.y, position.width - constraintRect.width - toggleRect.width, position.height);

            // Draw fields
            EditorGUI.LabelField(constraintRect, "Constrain");
            EditorGUI.PropertyField(toggleRect, constrainAxisProperty, GUIContent.none);

            bool guiEnabled = GUI.enabled;
            GUI.enabled = constrainAxisProperty.boolValue;
            EditorGUI.PropertyField(rangeRect, axisRangeProperty, GUIContent.none);
            GUI.enabled = guiEnabled;

            // Set indent back to what it was
            EditorGUI.indentLevel = indent;

            EditorGUI.EndProperty();
        }
    }
}
