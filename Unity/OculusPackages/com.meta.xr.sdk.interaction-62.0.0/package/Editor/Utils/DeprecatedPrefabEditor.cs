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

using UnityEngine;
using UnityEditor;

namespace Oculus.Interaction.Editor
{
    [CustomEditor(typeof(DeprecatedPrefab))]
    [CanEditMultipleObjects]
    public class DeprecatedPrefabEditor : UnityEditor.Editor
    {
        private SerializedProperty _replacementProperty;
        private SerializedProperty _supressWarningProperty;

        private void Awake()
        {
            //force internally the icon to be yellow on creation, but allow users to change it.
        }

        void OnEnable()
        {
            _replacementProperty = serializedObject.FindProperty("_replacement");
            _supressWarningProperty = serializedObject.FindProperty("_supressWarning");
        }

        void SetIconForDeprecatedPrefab(DeprecatedPrefab deprecatedPrefab, string iconName)
        {
            if (deprecatedPrefab == null)
            {
                return;
            }

            GameObject gameObject = deprecatedPrefab.gameObject;
            Texture2D texture = EditorGUIUtility.GetIconForObject(gameObject);
            if (texture == null)
            {
                GUIContent iconContent = EditorGUIUtility.IconContent(iconName);
                EditorGUIUtility.SetIconForObject(gameObject, iconContent.image as Texture2D);
            }
        }

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            EditorGUILayout.HelpBox(DeprecatedPrefab.label, MessageType.Warning);

            //users should not be able to change the suggested replacement
            GUI.enabled = false;
            EditorGUILayout.PropertyField(_replacementProperty);
            GUI.enabled = true;

            EditorGUILayout.PropertyField(_supressWarningProperty);

            serializedObject.ApplyModifiedProperties();
        }
    }
}
