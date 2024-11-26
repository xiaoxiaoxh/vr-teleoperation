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
    [CustomPropertyDrawer(typeof(HelpBoxAttribute))]
    public class HelpBoxDrawer : PropertyDrawer
    {
        private const float _minHeight = 40f;

        private bool ShouldDisplay(SerializedProperty property)
        {
            HelpBoxAttribute helpBoxAttribute = (HelpBoxAttribute)attribute;

            string conditionPath = property.propertyPath;
            SerializedProperty conditionalProperty = property.serializedObject.FindProperty(conditionPath);

            return ConditionalHideDrawer.ShouldDisplay(conditionalProperty, helpBoxAttribute.Value, helpBoxAttribute.Display);
        }

        private float GetHeight()
        {
            HelpBoxAttribute helpBoxAttribute = (HelpBoxAttribute)attribute;
            GUIContent content = new GUIContent(helpBoxAttribute.Message);
            GUIStyle style = GUI.skin.GetStyle("helpbox");
            float height = style.CalcHeight(content, EditorGUIUtility.currentViewWidth);
            return Mathf.Max(_minHeight, height);
        }

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            EditorGUI.PropertyField(position, property, label, true);

            if (!ShouldDisplay(property))
            {
                return;
            }

            float height = base.GetPropertyHeight(property, label);
            Rect rect = new Rect(position.x, position.y + height + EditorGUIUtility.standardVerticalSpacing,
                position.width, GetHeight());
            HelpBoxAttribute helpBoxAttribute = (HelpBoxAttribute)attribute;
            EditorGUI.HelpBox(rect, helpBoxAttribute.Message, (MessageType)helpBoxAttribute.Type);
        }

        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            float baseHeight = base.GetPropertyHeight(property, label);

            if (ShouldDisplay(property))
            {
                return baseHeight + GetHeight()
                    + EditorGUIUtility.standardVerticalSpacing;
            }
            else
            {
                return baseHeight;
            }
        }
    }
}
