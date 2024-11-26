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
using static Oculus.Interaction.ConditionalHideAttribute;

namespace Oculus.Interaction.Editor
{
    [CustomPropertyDrawer(typeof(ConditionalHideAttribute))]
    public class ConditionalHideDrawer : PropertyDrawer
    {
        private bool ShouldDisplay(SerializedProperty property)
        {
            ConditionalHideAttribute hideAttribute = (ConditionalHideAttribute)attribute;

            int index = property.propertyPath.LastIndexOf('.');
            string containerPath = property.propertyPath.Substring(0, index + 1);
            string conditionPath = containerPath + hideAttribute.ConditionalFieldPath;
            SerializedProperty conditionalProperty = property.serializedObject.FindProperty(conditionPath);

            return ShouldDisplay(conditionalProperty, hideAttribute.Value, hideAttribute.Display);
        }

        public static bool ShouldDisplay(SerializedProperty property, object value, DisplayMode displayMode)
        {
            if (displayMode == DisplayMode.Always)
            {
                return true;
            }

            if (displayMode == DisplayMode.Never)
            {
                return false;
            }

            bool areEqual;
            switch (property.type)
            {
                case "Enum":
                    areEqual = property.enumValueIndex == (int)value; break;
                case "int":
                    areEqual = property.intValue == (int)value; break;
                case "float":
                    areEqual = property.floatValue == (float)value; break;
                case "double":
                    areEqual = property.doubleValue == (double)value; break;
                case "bool":
                    areEqual = property.boolValue == (bool)value; break;
                case "string":
                    areEqual = property.stringValue == (string)value; break;
                default:
                    areEqual = property.objectReferenceValue == (object)value; break;
            }
            return (areEqual && displayMode == DisplayMode.ShowIfTrue)
                || (!areEqual && displayMode == DisplayMode.HideIfTrue);
        }

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            if (ShouldDisplay(property))
            {
                EditorGUI.PropertyField(position, property, label, true);
            }
        }

        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            if (ShouldDisplay(property))
            {
                return EditorGUI.GetPropertyHeight(property, label, true);
            }
            else
            {
                return -EditorGUIUtility.standardVerticalSpacing;
            }
        }
    }
}
