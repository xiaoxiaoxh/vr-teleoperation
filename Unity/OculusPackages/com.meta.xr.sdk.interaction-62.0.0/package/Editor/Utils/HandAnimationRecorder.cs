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

using Oculus.Interaction.HandGrab.Editor;
using Oculus.Interaction.HandGrab.Visuals;
using Oculus.Interaction.Input;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Oculus.Interaction.Utils
{
    public class HandAnimationRecorder : EditorWindow
    {
        [SerializeField]
        private HandVisual _handVisual;
        [SerializeField]
        private string _handLeftPrefix = "_l_";
        [SerializeField]
        private string _handRightPrefix = "_r_";


        [SerializeField]
        private HandFingerJointFlags _includedJoints = HandFingerJointFlags.HandMaxSkinnable - 1;

        [SerializeField]
        private bool _includeJointPosition = false;

        [SerializeField]
        private string _folder = "GeneratedAnimations";
        [SerializeField]
        private string _clipName = "HandAnimation";

        [SerializeField]
        private int _framerate = 30;
        [SerializeField]
        private float _slopeRotationThreshold = 0.1f;
        [SerializeField]
        private float _slopePositionThreshold = 0.0005f;

        [SerializeField]
        private KeyCode _recordKey = KeyCode.Space;

        [SerializeField]
        private HandGhostProvider _ghostProvider;

        [SerializeField]
        private AnimationClip _animationClip;

        private bool _showMin = true;
        private float _trimMin = 0f;
        private float _trimMax = 1f;

        private JointRecord[] _jointRecords;
        private JointRecord _rootRecord;
        private float _startTime;
        private bool _isRecording;

        private HandGhost _handGhost;
        private Handedness _ghostHandedness;

        private GUIStyle _richTextStyle;
        private Vector2 _scrollPos = Vector2.zero;
        private bool _forceUpdateGhost = true;


        [MenuItem("Oculus/Interaction/Hand Animation Recorder")]
        private static void CreateWizard()
        {
            HandAnimationRecorder window = EditorWindow.GetWindow<HandAnimationRecorder>();
            window.titleContent = new GUIContent("Hand Animation Recorder");
            window.Show();
        }


        private void OnEnable()
        {
            _richTextStyle = EditorGUIUtility.GetBuiltinSkin(EditorGUIUtility.isProSkin ? EditorSkin.Scene : EditorSkin.Inspector).label;
            _richTextStyle.richText = true;
            _richTextStyle.wordWrap = true;
            if (_ghostProvider == null)
            {
                HandGhostProviderUtils.TryGetDefaultProvider(out _ghostProvider);
            }
            _forceUpdateGhost = true;
        }

        private void OnDisable()
        {
            if (_isRecording)
            {
                _isRecording = false;
                _handVisual.WhenHandVisualUpdated -= HandleHandUpdated;
            }

            DestroyGhost();
        }

        private void OnGUI()
        {
            Event e = Event.current;
            if (e.type == EventType.KeyDown
                && e.keyCode == _recordKey)
            {
                if (!_isRecording)
                {
                    StartRecording();
                }
                else
                {
                    StopRecording();
                }
                e.Use();
            }

            GUILayout.Label("Use this editor tool to record the movement of the Hand during <b>Play Time</b> and output the result to an Animation Clip.", _richTextStyle);

            _scrollPos = GUILayout.BeginScrollView(_scrollPos);

            GUILayout.Space(20);
            HandAnimationUtils.GenerateObjectField(ref _handVisual, "Hand Visual");

            _includedJoints = (HandFingerJointFlags)EditorGUILayout.EnumFlagsField("Record Joints", _includedJoints);
            _includeJointPosition = EditorGUILayout.Toggle("Include Position", _includeJointPosition);

            _framerate = EditorGUILayout.IntField("Animation framerate", _framerate);
            _slopeRotationThreshold = EditorGUILayout.FloatField("Rotation compression delta", _slopeRotationThreshold);
            _slopePositionThreshold = EditorGUILayout.FloatField("Translation compression delta", _slopePositionThreshold);

            _handLeftPrefix = EditorGUILayout.TextField("Left prefix", _handLeftPrefix);
            _handRightPrefix = EditorGUILayout.TextField("Right prefix", _handRightPrefix);

            GUILayout.Space(20);
            GUILayout.Label("Indicate where to store the generated clip:", _richTextStyle);
            _folder = EditorGUILayout.TextField("Assets sub-folder", _folder);
            _clipName = EditorGUILayout.TextField("Name of the animation file", _clipName);

            GUILayout.Space(20);

            GUILayout.BeginHorizontal();
            GUILayout.Label("Use this key to start/stop the recording via the keyboard:", _richTextStyle);
            _recordKey = (KeyCode)EditorGUILayout.EnumPopup(_recordKey);
            GUILayout.EndHorizontal();

            GUILayout.Label("Or alternatively click this button:", _richTextStyle);
            if (!_isRecording && GUILayout.Button("Start Recording", GUILayout.Height(100)))
            {
                StartRecording();
            }
            if (_isRecording && GUILayout.Button("Stop Recording", GUILayout.Height(100)))
            {
                StopRecording();
            }

            GUILayout.Space(20);
            GUILayout.Label("Prefabs provider for the hands (ghosts) to visualize the recorded animation:");
            _forceUpdateGhost |= HandAnimationUtils.GenerateObjectField(ref _ghostProvider);


            GUILayout.Space(20);
            GUILayout.Label("Recorded animation:");
            _forceUpdateGhost |= HandAnimationUtils.GenerateObjectField(ref _animationClip);

            if (_animationClip != null)
            {
                GUILayout.Label("Preview and Trim the animation:");
                GUILayout.BeginHorizontal();
                float prevMin = _trimMin;
                float prevMax = _trimMax;
                EditorGUILayout.MinMaxSlider(ref _trimMin, ref _trimMax, 0f, 1f);
                if (prevMin != _trimMin)
                {
                    _showMin = true;
                }
                else if (prevMax != _trimMax)
                {
                    _showMin = false;
                }

                if (GUILayout.Button("Trim", GUILayout.Height(20)))
                {
                    Trim(ref _animationClip, _trimMin, _trimMax);
                    _trimMin = 0f;
                    _trimMax = 1f;
                }
                GUILayout.EndHorizontal();

                GUILayout.Space(20);
                if (GUILayout.Button("Generate mirrored copy", GUILayout.Height(20)))
                {
                    Mirror(_animationClip);
                }

                float time = _showMin ? _trimMin : _trimMax;
                UpdateGhost(time, _forceUpdateGhost);
            }

            _forceUpdateGhost = false;

            GUILayout.EndScrollView();
        }

        private void StartRecording()
        {
            if (_isRecording)
            {
                Debug.LogError("Already recording!");
                return;
            }

            _isRecording = true;
            _startTime = Time.time;
            InitializeRecords(out _jointRecords, out _rootRecord);
            _handVisual.WhenHandVisualUpdated += HandleHandUpdated;
        }

        private void StopRecording()
        {
            if (!_isRecording)
            {
                Debug.LogError("It is not recording!");
                return;
            }

            _isRecording = false;
            _handVisual.WhenHandVisualUpdated -= HandleHandUpdated;
            _animationClip = GenerateClipAsset(_clipName, _rootRecord, _jointRecords);
        }

        private void InitializeRecords(out JointRecord[] jointRecords, out JointRecord rootRecord)
        {
            jointRecords = new JointRecord[(int)HandJointId.HandEnd];
            Transform root = _handVisual.Root;
            foreach (HandJointId jointId in IncludedJointIds())
            {
                Transform jointTransform = _handVisual.GetTransformByHandJointId(jointId);
                string path = HandAnimationUtils.GetGameObjectPath(jointTransform, root);
                jointRecords[(int)jointId] = new JointRecord(jointId, path);
            }

            rootRecord = new JointRecord(HandJointId.Invalid, "");
        }

        private void HandleHandUpdated()
        {
            float time = Time.time - _startTime;
            ReadPoses(time);
        }

        private void ReadPoses(float time)
        {
            _rootRecord.RecordPose(time, _handVisual.Root.GetPose(Space.World));
            foreach (HandJointId jointId in IncludedJointIds())
            {
                Pose pose = _handVisual.GetJointPose(jointId, Space.Self);
                _jointRecords[(int)jointId].RecordPose(time, pose);
            }
        }

        private AnimationClip GenerateClipAsset(string title, JointRecord rootRecord, JointRecord[] jointRecords)
        {
            AnimationClip clip = new AnimationClip();
            clip.frameRate = _framerate;

            HandAnimationUtils.WriteAnimationCurves(ref clip, rootRecord, true);
            foreach (HandJointId jointId in IncludedJointIds())
            {
                int index = (int)jointId;
                HandAnimationUtils.WriteAnimationCurves(ref clip, jointRecords[index], _includeJointPosition);
            }
            HandAnimationUtils.Compress(ref clip, _slopeRotationThreshold, _slopePositionThreshold);
            HandAnimationUtils.StoreAsset(clip, _folder, $"{title}.anim");
            return clip;
        }

        private IEnumerable<HandJointId> IncludedJointIds()
        {
            for (HandJointId jointId = HandJointId.HandStart; jointId < HandJointId.HandEnd; jointId++)
            {
                int index = (int)jointId;
                if (((int)_includedJoints & (1 << index)) == 0)
                {
                    continue;
                }
                yield return jointId;
            }
        }

        private void Mirror(AnimationClip clip)
        {
            AnimationClip mirrorClip = HandAnimationUtils.Mirror(clip, _handLeftPrefix, _handRightPrefix, false);
            HandAnimationUtils.Compress(ref mirrorClip, _slopeRotationThreshold, _slopePositionThreshold);
            HandAnimationUtils.StoreAsset(mirrorClip, _folder, $"{clip.name}_mirror.anim");
        }

        private void Trim(ref AnimationClip clip, float minTime, float maxTime)
        {
            HandAnimationUtils.Trim(ref clip, minTime, maxTime);
        }

        #region ghost utilities

        private void CreateGhost(Handedness handedness)
        {
            if (_ghostProvider == null)
            {
                return;
            }

            HandGhost ghostPrototype = _ghostProvider.GetHand(handedness);
            _handGhost = GameObject.Instantiate(ghostPrototype);
            _handGhost.gameObject.hideFlags = HideFlags.HideAndDontSave;
            _ghostHandedness = handedness;
        }

        private void DestroyGhost()
        {
            if (_handGhost == null)
            {
                return;
            }
            GameObject.DestroyImmediate(_handGhost.gameObject);
        }

        private void UpdateGhost(float normalizedTime, bool forceUpdate)
        {
            if (_animationClip == null)
            {
                DestroyGhost();
                return;
            }

            if (forceUpdate)
            {
                if (!HandAnimationUtils.TryGetClipHandedness(_animationClip,
                    _handLeftPrefix, _handRightPrefix,
                    out Handedness handedness))
                {
                    DestroyGhost();
                    return;
                }

                if (_ghostHandedness != handedness)
                {
                    DestroyGhost();
                }

                if (_handGhost == null)
                {
                    CreateGhost(handedness);
                }
            }

            if (_handGhost == null)
            {
                return;
            }


            float time = normalizedTime * _animationClip.length;
            GameObject root = _handGhost.Root.gameObject;
            _animationClip.SampleAnimation(root, time);

        }

        #endregion

    }
}
