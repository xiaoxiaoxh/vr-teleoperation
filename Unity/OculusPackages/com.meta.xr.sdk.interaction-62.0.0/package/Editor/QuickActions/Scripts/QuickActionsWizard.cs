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
using UnityEngine.EventSystems;
using System;
using System.Linq;
using System.Reflection;
using System.ComponentModel;
using System.Collections.Generic;

using Object = UnityEngine.Object;
using Component = UnityEngine.Component;

namespace Oculus.Interaction.Editor.QuickActions
{
    internal abstract class QuickActionsWizard : EditorWindow
    {
        protected class DeviceTypeAttribute : PropertyAttribute
        {
            public IReadOnlyList<(string name, DeviceTypes type)> GetDevices()
            {
                var names = new List<(string, DeviceTypes)>();
                if (InteractorUtils.CanAddHandInteractorsToRig())
                {
                    names.Add(("Add To Hands", DeviceTypes.Hands));
                }
                if (InteractorUtils.CanAddControllerInteractorsToRig())
                {
                    names.Add(("Add To Controllers", DeviceTypes.Controllers));
                }
                return names;
            }
        }

        [CustomPropertyDrawer(typeof(DeviceTypeAttribute))]
        public class DeviceTypeDrawer : PropertyDrawer
        {
            private DeviceTypeAttribute DeviceTypeAttribute => attribute as DeviceTypeAttribute;

            public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
            {
                var names = DeviceTypeAttribute.GetDevices();

                if (names.Count == 0) // No input devices, do not draw dropdown
                {
                    float iconSize = position.height;
                    GUI.Box(new Rect(position.position, new(iconSize, iconSize)), Styles.WarningIcon, GUI.skin.label);
                    position = new Rect(position.x + iconSize, position.y, position.width - iconSize, position.height);
                    EditorGUI.LabelField(position, "Cannot add interactors: No input devices present in scene.");
                    return;
                }

                int mask = 0;
                for (int i = 0; i < names.Count; ++i)
                {
                    if ((property.intValue & (int)names[i].type) != 0)
                    {
                        mask |= 1 << i;
                    }
                }

                mask = EditorGUI.MaskField(position, mask, names.Select(n => n.name).ToArray());

                int value = 0;
                for (int i = 0; i < names.Count; ++i)
                {
                    if ((mask & (1 << i)) != 0)
                    {
                        value |= (int)names[i].type;
                    }
                }
                property.enumValueFlag = value;
            }
        }

        protected class MessageData
        {
            /// <summary>
            /// The message type, which sets the icon. Error messages
            /// are considered fatal and block creation of the object.
            /// </summary>
            public readonly MessageType MessageType;

            /// <summary>
            /// The message to be displayed in the banner
            /// </summary>
            public readonly string Message;

            /// <summary>
            /// If non-null, a button will be shown with this name and action
            /// </summary>
            public readonly ButtonData ButtonData;

            public MessageData(MessageType messageType, string message,
                ButtonData buttonData = null)
            {
                MessageType = messageType;
                Message = message;
                ButtonData = buttonData;
            }
        }

        protected class ButtonData
        {
            public readonly string Label;
            public readonly Action Action;
            public ButtonData(string label, Action action)
            {
                Label = label;
                Action = action;
            }
        }

        private class WizardSetting : WizardField
        {
            public readonly object DefaultValue;

            public WizardSetting(FieldInfo fieldInfo) : base(fieldInfo)
            {
                var defaultValueAttribute = fieldInfo.GetCustomAttribute<DefaultValueAttribute>();
                if (defaultValueAttribute != null)
                {
                    DefaultValue = defaultValueAttribute.Value;
                }
            }

            public void ResetToDefault(Object obj)
            {
                if (DefaultValue != null)
                {
                    _fieldInfo.SetValue(obj, DefaultValue);
                }
            }

            public bool HasDefaultValue(Object obj)
            {
                return DefaultValue == null || _fieldInfo.GetValue(obj).Equals(DefaultValue);
            }
        }

        private class WizardDependency : WizardField
        {
            public Category Category => _attribute.Category;
            public override bool ReadOnly => _attribute.ReadOnly;

            public readonly Action<QuickActionsWizard> FindAction;
            public readonly Action<QuickActionsWizard> FixAction;

            private readonly WizardDependencyAttribute _attribute;

            public WizardDependency(FieldInfo fieldInfo) : base(fieldInfo)
            {
                _attribute = fieldInfo.GetCustomAttribute<WizardDependencyAttribute>();

                if (!string.IsNullOrEmpty(_attribute.FindMethod))
                {
                    var findAction = fieldInfo.DeclaringType.GetMethod(_attribute.FindMethod,
                        BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
                    FindAction = findAction == null ? null :
                        w => { findAction.Invoke(w, null); w.SyncSerializedObject(); };
                }

                if (!string.IsNullOrEmpty(_attribute.FixMethod))
                {
                    var fixAction = fieldInfo.DeclaringType.GetMethod(_attribute.FixMethod,
                        BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
                    FixAction = fixAction == null ? null :
                        w => { fixAction.Invoke(w, null); w.SyncSerializedObject(); };
                }
            }

            /// <summary>
            /// Check if the serialized property has an object value
            /// </summary>
            /// <param name="wizard">The wizard instance to check</param>
            /// <param name="fullArray">If set, any null array elements
            /// will cause this to return false.</param>
            /// <returns></returns>
            public bool HasValue(QuickActionsWizard wizard, bool fullArray = false)
            {
                var prop = wizard.SerializedObject.FindProperty(PropertyPath);
                if (prop.isArray)
                {
                    int nonNull = 0;
                    for (int i = 0; i < prop.arraySize; ++i)
                    {
                        var element = prop.GetArrayElementAtIndex(i);
                        if (element.objectReferenceValue != null)
                        {
                            ++nonNull;
                        }
                    }
                    return fullArray ? nonNull == prop.arraySize : nonNull > 0;
                }
                else
                {
                    return prop.objectReferenceValue != null;
                }
            }
        }

        private abstract class WizardField
        {
            public virtual bool ReadOnly => false;

            public readonly string PropertyPath;
            public readonly string DisplayName;
            public readonly string Tooltip;

            public readonly Action<QuickActionsWizard> ChangeCallback;

            protected readonly FieldInfo _fieldInfo;

            private readonly ConditionalHideAttribute _conditionalHide;

            protected WizardField(FieldInfo fieldInfo)
            {
                _fieldInfo = fieldInfo;

                PropertyPath = fieldInfo.Name;

                TooltipAttribute tooltip = fieldInfo.GetCustomAttribute<TooltipAttribute>();
                if (tooltip != null)
                {
                    Tooltip = tooltip.tooltip;
                }

                InspectorNameAttribute inspectorName =
                    fieldInfo.GetCustomAttribute<InspectorNameAttribute>();
                DisplayName = inspectorName != null ?
                    inspectorName.displayName :
                    ObjectNames.NicifyVariableName(fieldInfo.Name);

                _conditionalHide = fieldInfo.GetCustomAttribute<ConditionalHideAttribute>();

                var changeCheckAttribute = fieldInfo.GetCustomAttribute<ChangeCheckAttribute>();
                if (changeCheckAttribute != null)
                {
                    var changeAction = fieldInfo.DeclaringType.GetMethod(changeCheckAttribute.Callback,
                        BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
                    ChangeCallback = changeAction == null ? null :
                        w => { w.SyncSerializedObject(); changeAction.Invoke(w, null); };
                }
            }

            public bool ShouldConditionalShow(QuickActionsWizard wizard)
            {
                if (_conditionalHide == null)
                {
                    return true;
                }

                return ConditionalHideDrawer.ShouldDisplay
                    (wizard.SerializedObject.FindProperty(_conditionalHide.ConditionalFieldPath),
                    _conditionalHide.Value, _conditionalHide.Display);
            }
        }

        protected enum Category
        {
            Required,
            Optional,
        }

        protected class ChangeCheckAttribute : System.Attribute
        {
            public readonly string Callback;

            public ChangeCheckAttribute(string callback)
            {
                Callback = callback;
            }
        }

        protected class WizardDependencyAttribute : System.Attribute
        {
            public Category Category { get; set; } = Category.Required;

            public string FindMethod { get; set; } = string.Empty;

            public string FixMethod { get; set; } = string.Empty;

            public bool ReadOnly { get; set; } = false;

            public WizardDependencyAttribute() { }
        }

        protected class WizardSettingAttribute : System.Attribute
        {
            public WizardSettingAttribute() { }
        }

        protected const string MENU_FOLDER = "GameObject/Interaction SDK/";

        /// <summary>
        /// Window should not be drawn in batch mode, ie from Unit Tests
        /// </summary>
        internal static bool ShouldDraw => !Application.isBatchMode;

        /// <summary>
        /// The GameObject that the user wishes to augment
        /// </summary>
        protected GameObject Target { get; private set; }

        /// <summary>
        /// Shared messages between Wizard types
        /// </summary>
        protected WizardMessages Messages =>
            _messages ??= new WizardMessages(this);
        private WizardMessages _messages;

        private static WizardStyles Styles =>
            _styles ??= new WizardStyles();
        private static WizardStyles _styles;

        private SerializedObject SerializedObject =>
            _serializedObject ??= new SerializedObject(this);
        private SerializedObject _serializedObject;

        private WizardDependency[] _dependencies;
        private WizardSetting[] _settings;

        private Vector2 _contentScrollPos = Vector2.zero;
        private bool _foldoutSettings = true;
        private bool _foldoutRequired = true;
        private bool _foldoutOptional = true;
        private bool _shouldClose = false;

        /// <summary>
        /// Show the provided window type and set the target to
        /// the provided GameObject
        /// </summary>
        /// <typeparam name="T">The Wizard type</typeparam>
        /// <param name="target">The object the user wishes to modify</param>
        /// <returns>The window instance</returns>
        protected static T ShowWindow<T>(GameObject target)
            where T : QuickActionsWizard
        {
            T wizard = ShouldDraw ? GetWindow<T>() : CreateInstance<T>();

            wizard.Target = target;
            wizard.titleContent =
                new GUIContent(wizard.GetWindowTitle());
            wizard.minSize = new Vector2(480, 320);

            wizard.InitializeFields();

            if (ShouldDraw)
            {
                wizard.Show();
            }

            return wizard;
        }

        /// <summary>
        /// Executes the wizard action with all default values
        /// </summary>
        /// <typeparam name="TWizard">The wizard type to run</typeparam>
        /// <param name="target">The target object</param>
        /// <param name="fixOptionals">Whether or not optionals dependencies should be automatically fixed</param>
        /// <param name="injections">Optional delegate to inject modifications to the wizard</param>
        /// <returns>All prefabs instantiated during the creation</returns>
        internal static IEnumerable<GameObject> CreateWithDefaults<TWizard>(GameObject target, bool fixOptionals = true, Action<TWizard> injections = null)
            where TWizard : QuickActionsWizard
        {
            var wizard = CreateInstance<TWizard>();
            wizard.Target = target;
            injections?.Invoke(wizard);

            List<GameObject> newObjects = new List<GameObject>();
            void OnObjCreated(Template t, GameObject o) => newObjects.Add(o);

            Templates.WhenObjectCreated += OnObjCreated;

            wizard.InitializeFields();
            wizard.FixMissingDependencies(fixOptionals);

            if (wizard.CanCreate())
            {
                wizard.Create();
            }
            else
            {
                Debug.LogError($"Could not execute {typeof(TWizard).Name}");
            }

            Templates.WhenObjectCreated -= OnObjCreated;
            DestroyImmediate(wizard);
            return newObjects;
        }

        private void Awake()
        {
            FieldInfo[] fieldInfos = GetType().GetFields(
                BindingFlags.Public |
                BindingFlags.NonPublic |
                BindingFlags.Instance);

            _dependencies = fieldInfos
                .Where(fi => fi.GetCustomAttribute<WizardDependencyAttribute>() != null &&
                    SerializedObject.FindProperty(fi.Name) != null)
                .Select(fi => new WizardDependency(fi)).ToArray();

            _settings = fieldInfos
                .Where(fi => fi.GetCustomAttribute<WizardSettingAttribute>() != null &&
                    SerializedObject.FindProperty(fi.Name) != null)
                .Select(fi => new WizardSetting(fi)).ToArray();
        }

        internal void OnGUI()
        {
            if (Target == null)
            {
                Debug.LogError("Target object was destroyed.");
                Close();
                return;
            }

            PruneReferences();

            UpdateReadOnlyFields();

            if (ShouldDraw)
            {
                DrawWizard();
            }

            SyncSerializedObject();

            if (_shouldClose)
            {
                Close();
            }
        }

        private void DrawWizard()
        {
            DrawMessages();

            EditorGUILayout.Space();

            DrawTargetField();

            EditorGUILayout.Space();

            DrawContent();

            GUILayout.FlexibleSpace();

            DrawFooter();
        }

        protected void SyncSerializedObject()
        {
            SerializedObject.ApplyModifiedProperties();
            SerializedObject.Update();
        }

        private void UpdateReadOnlyFields()
        {
            // Read only fields cannot be assigned by the user,
            // therefore we attempt to auto-assign each frame

            foreach (var dependency in _dependencies)
            {
                if (dependency.ReadOnly)
                {
                    dependency.FindAction?.Invoke(this);
                }
            }
        }

        private void PruneReferences()
        {
            // When destroying objects with Undo, references can be left
            // dangling (shown as Missing) and must be cleaned up.
            foreach (var dependency in _dependencies)
            {
                static void PruneProperty(SerializedProperty property)
                {
                    if (property != null &&
                        property.objectReferenceValue == null)
                    {
                        property.objectReferenceValue = null;
                    }
                }

                var prop = SerializedObject
                    .FindProperty(dependency.PropertyPath);

                if (prop.isArray)
                {
                    for (int i = 0; i < prop.arraySize; ++i)
                    {
                        PruneProperty(prop.GetArrayElementAtIndex(i));
                    }
                }
                else
                {
                    PruneProperty(prop);
                }
            }
        }

        private void DrawContent()
        {
            bool ShouldDrawField(WizardField field)
            {
                if (!field.ShouldConditionalShow(this))
                {
                    return false;
                }
                return true;
            }

            void DrawHeader(ref bool foldout, Rect rect, string label,
                GUIContent icon, ButtonData buttonData = null)
            {
                var previousLabelWidth = EditorGUIUtility.labelWidth;
                EditorGUIUtility.labelWidth = rect.width - 8;
                using (var scope = new EditorGUILayout.HorizontalScope(Styles.FoldoutHorizontal))
                {
                    foldout = EditorGUILayout.Foldout(foldout, label, Styles.Foldout);
                    if (buttonData != null)
                    {
                        GUILayout.Box(icon.image, Styles.FixIcon);
                        if (GUILayout.Button(buttonData.Label, Styles.FixAllButton))
                        {
                            buttonData.Action.Invoke();
                        }
                    }
                }
                EditorGUIUtility.labelWidth = previousLabelWidth;
            }

            void DrawField(WizardField field, ButtonData buttonData = null)
            {
                using (var scope = new EditorGUILayout.VerticalScope(Styles.ListLabel))
                {
                    using (new EditorGUILayout.HorizontalScope())
                    {
                        EditorGUILayout.LabelField(field.DisplayName, Styles.WizardField);
                        if (buttonData != null && GUILayout.Button(buttonData.Label, Styles.FixButton))
                        {
                            buttonData.Action.Invoke();
                        }
                    }
                    if (!string.IsNullOrEmpty(field.Tooltip))
                    {
                        EditorGUILayout.LabelField(field.Tooltip, Styles.WizardFieldTooltip);
                    }

                    var property = SerializedObject.FindProperty(field.PropertyPath);

                    bool guiEnabled = GUI.enabled;
                    GUI.enabled = !field.ReadOnly;
                    EditorGUI.BeginChangeCheck();
                    EditorGUILayout.PropertyField(property, GUIContent.none);
                    bool changed = EditorGUI.EndChangeCheck();
                    GUI.enabled = guiEnabled;

                    if (changed)
                    {
                        field.ChangeCallback?.Invoke(this);
                    }
                }
            }

            void DrawDependencies(ref bool foldout, Rect rect, string header, GUIContent icon,
                IEnumerable<WizardDependency> dependencies)
            {
                void Fix(params WizardDependency[] dependencies)
                {
                    foreach (var dependency in dependencies)
                    {
                        // Try to find existing or add new
                        if (dependency.FindAction != null)
                        {
                            dependency.FindAction.Invoke(this);
                        }
                        if (!dependency.HasValue(this))
                        {
                            dependency.FixAction.Invoke(this);
                        }
                    }
                }

                var fixable = dependencies
                    .Where(d => !d.HasValue(this) && d.FixAction != null);

                ButtonData fixAllButton = fixable.Any() ?
                    new ButtonData("Fix All", () => Fix(fixable.ToArray())) : null;
                DrawHeader(ref foldout, rect, header, icon, fixAllButton);

                if (!foldout)
                {
                    return;
                }

                foreach (var dependency in dependencies)
                {
                    ButtonData fixButton = fixable.Contains(dependency) ?
                        new ButtonData("Fix", () => Fix(dependency)) : null;
                    DrawField(dependency, fixButton);
                }
            }

            void DrawSettings(ref bool foldout, Rect rect, string header, GUIContent icon,
                IEnumerable<WizardSetting> settings)
            {
                void Reset(params WizardSetting[] settings)
                {
                    foreach (var setting in settings)
                    {
                        setting.ResetToDefault(this);
                    }
                }

                var resettable = settings
                    .Where(d => d.DefaultValue != null && !d.HasDefaultValue(this));

                ButtonData resetAllButton = resettable.Any() ?
                    new ButtonData("Reset All", () => Reset(resettable.ToArray())) : null;
                DrawHeader(ref foldout, rect, header, icon, resetAllButton);

                if (!foldout)
                {
                    return;
                }

                foreach (var setting in settings)
                {
                    ButtonData resetButton = resettable.Contains(setting) ?
                        new ButtonData("Reset", () => Reset(setting)) : null;
                    DrawField(setting, resetButton);
                }
            }

            // Scrolling Content Area
            using (new EditorGUILayout.VerticalScope())
            {
                _contentScrollPos = EditorGUILayout.BeginScrollView(_contentScrollPos);

                // Draw Settings
                var settings = _settings.Where(fd => ShouldDrawField(fd));
                if (settings.Any())
                {
                    using (var v = new EditorGUILayout.VerticalScope(Styles.List))
                    {
                        DrawSettings(ref _foldoutSettings, v.rect,
                            $"Settings ({settings.Count()})", Styles.InfoIcon, settings);
                    }
                }

                // Draw Required Dependencies
                var required = _dependencies.Where(fd => ShouldDrawField(fd) && fd.Category == Category.Required);
                if (required.Any())
                {
                    using (var v = new EditorGUILayout.VerticalScope(Styles.List))
                    {
                        DrawDependencies(ref _foldoutRequired, v.rect,
                            $"Required Components ({required.Count()})", Styles.ErrorIcon, required);
                    }
                }

                // Draw Optional Dependencies
                var optional = _dependencies.Where(fd => ShouldDrawField(fd) && fd.Category == Category.Optional);
                if (optional.Any())
                {
                    using (var v = new EditorGUILayout.VerticalScope(Styles.List))
                    {
                        DrawDependencies(ref _foldoutOptional, v.rect,
                            $"Optional Components ({optional.Count()})", Styles.InfoIcon, optional);
                    }
                }

                EditorGUILayout.EndScrollView();
            }
        }

        private bool CanCreate()
        {
            bool result = true;
            result &= !HasErrorMessages();
            result &= _dependencies.Count() == 0 || !_dependencies.Any(fd =>
                fd.ShouldConditionalShow(this) &&
                fd.Category == Category.Required &&
                !fd.HasValue(this, true));
            return result;
        }

        private bool HasErrorMessages()
        {
            return GetMessages().Any(msg => msg.MessageType == MessageType.Error);
        }

        private void DrawTargetField()
        {
            using (new EditorGUILayout.VerticalScope(Styles.TargetLabel))
            {
                GUI.enabled = false;
                using (new EditorGUILayout.HorizontalScope())
                {
                    EditorGUILayout.LabelField("Target Object",
                        GUILayout.ExpandWidth(false), GUILayout.MaxWidth(90));
                    EditorGUILayout.ObjectField(
                        Target, typeof(Object), true);
                }
                GUI.enabled = true;
            }
        }

        private void DrawMessages()
        {
            // Order in decreasing severity
            var messages = GetMessages()
                .OrderByDescending(msg => msg.MessageType);

            using (new EditorGUILayout.VerticalScope())
            {
                foreach (var message in messages)
                {
                    using (new GUILayout.HorizontalScope())
                    {
                        EditorGUILayout.HelpBox(message.Message, message.MessageType);
                        if (message.ButtonData != null &&
                            GUILayout.Button(message.ButtonData.Label, Styles.MessageButton,
                                GUILayout.ExpandHeight(true)))
                        {
                            message.ButtonData.Action.Invoke();
                        }
                    }
                }
            }
        }

        private void DrawFooter()
        {
            using (new EditorGUILayout.VerticalScope(Styles.ButtonArea))
            {
                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button("Cancel", GUILayout.ExpandHeight(true)))
                    {
                        _shouldClose = true;
                    }
                    GUI.enabled = CanCreate();
                    if (GUILayout.Button("Create", GUILayout.ExpandHeight(true)))
                    {
                        // Track all template objects created during the Create
                        // action in order to select them in the Hierarchy
                        List<GameObject> newObjects = new List<GameObject>();
                        void OnObjCreated(Template t, GameObject o) => newObjects.Add(o);

                        // Undo operations will be collapsed to this point
                        var undoBeforeCreate = Undo.GetCurrentGroup();

                        Templates.WhenObjectCreated += OnObjCreated;
                        Create();
                        Templates.WhenObjectCreated -= OnObjCreated;

                        // Collapse Undo group into a single operation
                        Undo.SetCurrentGroupName("ISDK QuickActions");
                        Undo.CollapseUndoOperations(undoBeforeCreate);

                        Selection.objects = newObjects.ToArray();
                        _shouldClose = true;
                    }
                    GUI.enabled = true;
                }
            }
        }

        /// <summary>
        /// Get the title of the wizard window
        /// </summary>
        private string GetWindowTitle()
        {
            return ObjectNames.NicifyVariableName(GetType().Name);
        }

        /// <summary>
        /// Override with creation logic to be run when user
        /// wishes to create the components.
        /// </summary>
        protected abstract void Create();

        /// <summary>
        /// Override with any initialization logic needed
        /// when wizard is first opened.
        /// </summary>
        protected virtual void InitializeFieldsExtra() { }

        /// <summary>
        /// Initializes the fields of the Wizard with default values,
        /// or components found when searching the hierarchy.
        /// </summary>
        private void InitializeFields()
        {
            foreach (var dependency in _dependencies)
            {
                dependency.FindAction?.Invoke(this);
            }

            foreach (var setting in _settings)
            {
                setting.ResetToDefault(this);
            }

            InitializeFieldsExtra();
            SyncSerializedObject();
        }

        private void FixMissingDependencies(bool fixOptionals = true)
        {
            foreach (var dependency in _dependencies)
            {
                if (!fixOptionals && dependency.Category == Category.Optional)
                {
                    continue;
                }
                
                dependency.FindAction?.Invoke(this);
                if (!dependency.HasValue(this))
                {
                    dependency.FixAction?.Invoke(this);
                }
            }
            SyncSerializedObject();
        }

        /// <summary>
        /// Messages are displayed as notification banners in the wizard window.
        /// </summary>
        /// <returns>A collection of messages to be displayed</returns>
        protected virtual IEnumerable<MessageData> GetMessages()
        {
            return Enumerable.Empty<MessageData>();
        }

        /// <summary>
        /// Add a component to a GameObject and register it in the Undo stack.
        /// </summary>
        /// <typeparam name="T">The component type to add</typeparam>
        /// <param name="gameObject">The GameObject to add the component to</param>
        /// <returns>The newly added component</returns>
        protected T AddComponent<T>(GameObject gameObject)
            where T : Component
        {
            T result = Undo.AddComponent<T>(gameObject);
            EditorGUIUtility.PingObject(gameObject);
            return result;
        }

        /// <summary>
        /// Add a component to a GameObject and register it in the Undo stack.
        /// </summary>
        /// <typeparam name="T">The component type to add</typeparam>
        /// <param name="gameObject">The GameObject to add the component to</param>
        /// <returns>The newly added component</returns>
        protected GameObject AddObject(string name, params Type[] components)
        {
            GameObject result = new GameObject(name, components);
            Undo.RegisterCreatedObjectUndo(result, $"Create {name}");
            EditorGUIUtility.PingObject(result);
            return result;
        }

        /// <summary>
        /// Common messages for the <see cref="QuickActionsWizard"/> window
        /// </summary>
        protected class WizardMessages
        {
            private readonly QuickActionsWizard _wizard;

            public WizardMessages(QuickActionsWizard wizardInstance)
            {
                _wizard = wizardInstance;
            }

            public IEnumerable<MessageData> MissingInteractor<TInteractor, TInteractable>()
                where TInteractor : Object, IInteractor
                where TInteractable : Object, IInteractable
            {
                if (FindAnyObjectByType<TInteractor>() == null)
                {
                    string interactorName = typeof(TInteractor).Name;
                    string interactableName = typeof(TInteractable).Name;
                    var message = new MessageData(MessageType.Warning,
                        $"No {interactorName} found in scene. The new {interactableName} " +
                        $"will not work without a {interactorName} present.");
                    return Enumerable.Repeat(message, 1);
                }
                return Enumerable.Empty<MessageData>();
            }

            public IEnumerable<MessageData> MissingPointableCanvasModule<TInteractor>()
                where TInteractor : Object, IInteractor
            {
                void FixPointableCanvasModule()
                {
                    GameObject eventSystemGO =
                        FindFirstObjectByType<EventSystem>()?.gameObject;

                    Object newObj;
                    if (eventSystemGO != null)
                    {
                        newObj = _wizard.AddComponent<PointableCanvasModule>(eventSystemGO);
                    }
                    else
                    {
                        newObj = _wizard.AddObject("Pointable Canvas Module",
                            typeof(EventSystem), typeof(PointableCanvasModule));
                    }

                    Debug.Log($"{nameof(PointableCanvasModule)} Added to Scene.", newObj);
                }

                if (FindAnyObjectByType<PointableCanvasModule>() == null)
                {
                    string interactorName = typeof(TInteractor).Name;
                    var message = new MessageData(MessageType.Warning,
                        $"No PointableCanvasModule found in scene. The new {interactorName} " +
                        "will not work without a PointableCanvasModule present.",
                        new ButtonData("Fix", FixPointableCanvasModule));
                    return Enumerable.Repeat(message, 1);
                }
                return Enumerable.Empty<MessageData>();
            }
        }
    }
}
