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

using Oculus.Interaction.Input;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace Oculus.Interaction.Utils
{
    public static class HandAnimationUtils
    {
        public const string localEulerKey = "localEulerAnglesRaw.";
        public const string positionKey = "m_LocalPosition.";

        public static void Compress(ref AnimationClip clip, float rotationThreshold, float positionThreshold)
        {
            EditorCurveBinding[] bindings = AnimationUtility.GetCurveBindings(clip);

            foreach (EditorCurveBinding binding in bindings)
            {
                float threshold;
                if (binding.propertyName.StartsWith(localEulerKey))
                {
                    threshold = rotationThreshold;
                }
                else if (binding.propertyName.StartsWith(positionKey))
                {
                    threshold = positionThreshold;
                }
                else
                {
                    return;
                }

                AnimationCurve curve = AnimationUtility.GetEditorCurve(clip, binding);

                for (int i = 2; i < curve.keys.Length; i++)
                {
                    Keyframe prevFrame = curve.keys[i - 1];
                    Keyframe prevPrevFrame = curve.keys[i - 2];
                    Keyframe currentFrame = curve.keys[i];

                    Vector2 prevDelta = new Vector2(prevFrame.time - prevPrevFrame.time, prevFrame.value - prevPrevFrame.value);
                    Vector2 currDelta = new Vector2(currentFrame.time - prevFrame.time, currentFrame.value - prevFrame.value);

                    float slopeDifference = prevDelta.x * currDelta.y - prevDelta.y * currDelta.x;
                    if (Mathf.Abs(slopeDifference) < threshold)
                    {
                        curve.RemoveKey(i--);
                    }
                }

                SmoothCurveTangets(ref curve);
                AnimationUtility.SetEditorCurve(clip, binding, curve);
            }
        }

        public static void Trim(ref AnimationClip clip, float minTime, float maxTime)
        {
            EditorCurveBinding[] bindings = AnimationUtility.GetCurveBindings(clip);

            float min = minTime * clip.length;
            float max = maxTime * clip.length;

            foreach (var binding in bindings)
            {
                AnimationCurve curve = AnimationUtility.GetEditorCurve(clip, binding);

                float minValue = curve.Evaluate(min);
                float maxValue = curve.Evaluate(max);

                while (curve.length > 0
                    && curve.keys[0].time < min)
                {
                    curve.RemoveKey(0);
                }

                while (curve.length > 0
                    && curve.keys[curve.keys.Length - 1].time > max)
                {
                    curve.RemoveKey(curve.keys.Length - 1);
                }

                if (curve.length > 0)
                {
                    if (curve.keys[0].time != min)
                    {
                        curve.AddKey(new Keyframe(min, minValue));
                    }

                    if (curve.keys[curve.keys.Length - 1].time != max)
                    {
                        curve.AddKey(new Keyframe(max, maxValue));
                    }

                    for (int i = 0; i < curve.keys.Length; i++)
                    {
                        Keyframe keyframe = curve.keys[i];
                        keyframe.time -= min;
                        curve.MoveKey(i, keyframe);
                    }
                }

                SmoothCurveTangets(ref curve);
                AnimationUtility.SetEditorCurve(clip, binding, curve);
            }
        }

        public static AnimationClip Mirror(AnimationClip clip, string leftPrefix, string rightPrefix, bool skipRoot)
        {
            string basePrefix;
            string mirrorPrefix;
            if (TryGetClipHandedness(clip, leftPrefix, rightPrefix, out Handedness handedness))
            {
                basePrefix = handedness == Handedness.Left ? leftPrefix : rightPrefix;
                mirrorPrefix = handedness == Handedness.Left ? rightPrefix : leftPrefix;
            }
            else
            {
                Debug.LogError("Could not identify the handedness of the clip." +
                    "Ensure there is an AnimationClip linked and that it matches the handedness prefixes");
                return null;
            }


            AnimationClip mirrorClip = new AnimationClip();
            mirrorClip.frameRate = clip.frameRate;

            EditorCurveBinding[] curveBindings = AnimationUtility.GetCurveBindings(clip);
            foreach (EditorCurveBinding curveBinding in curveBindings)
            {
                if (!skipRoot && curveBinding.path == "")
                {
                    continue;
                }

                string mirrorPath = curveBinding.path.Replace(basePrefix, mirrorPrefix);
                AnimationCurve curve = AnimationUtility.GetEditorCurve(clip, curveBinding);
                mirrorClip.SetCurve(mirrorPath, curveBinding.type, curveBinding.propertyName, curve);
            }

            if (skipRoot)
            {
                return mirrorClip;
            }

            EditorCurveBinding bindingEulerX = curveBindings.First(cb => cb.path == "" && cb.propertyName.StartsWith($"{localEulerKey}x"));
            EditorCurveBinding bindingEulerY = curveBindings.First(cb => cb.path == "" && cb.propertyName.StartsWith($"{localEulerKey}y"));
            EditorCurveBinding bindingEulerZ = curveBindings.First(cb => cb.path == "" && cb.propertyName.StartsWith($"{localEulerKey}z"));

            EditorCurveBinding bindingPositionX = curveBindings.First(cb => cb.path == "" && cb.propertyName.StartsWith($"{positionKey}x"));
            EditorCurveBinding bindingPositionY = curveBindings.First(cb => cb.path == "" && cb.propertyName.StartsWith($"{positionKey}y"));
            EditorCurveBinding bindingPositionZ = curveBindings.First(cb => cb.path == "" && cb.propertyName.StartsWith($"{positionKey}z"));

            AnimationCurve curveEulerX = AnimationUtility.GetEditorCurve(clip, bindingEulerX);
            AnimationCurve curveEulerY = AnimationUtility.GetEditorCurve(clip, bindingEulerY);
            AnimationCurve curveEulerZ = AnimationUtility.GetEditorCurve(clip, bindingEulerZ);

            AnimationCurve curvePositionX = AnimationUtility.GetEditorCurve(clip, bindingPositionX);
            AnimationCurve curvePositionY = AnimationUtility.GetEditorCurve(clip, bindingPositionY);
            AnimationCurve curvePositionZ = AnimationUtility.GetEditorCurve(clip, bindingPositionZ);

            Quaternion rotationOffset = Quaternion.Euler(180f, 0f, 0f);

            JointRecord rootMirroredRecord = new JointRecord(HandJointId.Invalid, "");

            for (float time = 0; time <= clip.length; time += 1f / clip.frameRate)
            {
                Quaternion rotation =
                    rotationOffset *
                    Quaternion.Euler(
                        curveEulerX.Evaluate(time),
                        curveEulerY.Evaluate(time),
                        curveEulerZ.Evaluate(time));
                Vector3 position = new Vector3(
                        curvePositionX.Evaluate(time),
                        curvePositionY.Evaluate(time),
                        curvePositionZ.Evaluate(time));

                rootMirroredRecord.RecordPose(time, new Pose(position, rotation));
            }

            WriteAnimationCurves(ref mirrorClip, rootMirroredRecord, true);

            return mirrorClip;
        }

        public static void WriteAnimationCurves(ref AnimationClip clip, JointRecord record, bool includePosition)
        {
            WriteCurve(ref clip, record.Path, $"{localEulerKey}x", record.RotationX);
            WriteCurve(ref clip, record.Path, $"{localEulerKey}y", record.RotationY);
            WriteCurve(ref clip, record.Path, $"{localEulerKey}z", record.RotationZ);

            if (includePosition)
            {
                WriteCurve(ref clip, record.Path, $"{positionKey}x", record.PositionX);
                WriteCurve(ref clip, record.Path, $"{positionKey}y", record.PositionY);
                WriteCurve(ref clip, record.Path, $"{positionKey}z", record.PositionZ);
            }
        }

        public static void WriteCurve(ref AnimationClip clip, string path, string propertyName, List<Keyframe> frames)
        {
            AnimationCurve curve = new AnimationCurve(frames.ToArray());
            clip.SetCurve(path, typeof(Transform), propertyName, curve);
        }

        public static bool TryGetClipHandedness(AnimationClip clip,
            string leftPrefix, string rightPrefix,
            out Handedness handedness)
        {
            if (clip == null)
            {
                handedness = Handedness.Left;
                return false;
            }

            EditorCurveBinding[] curveBindings = AnimationUtility.GetCurveBindings(clip);
            foreach (EditorCurveBinding curveBinding in curveBindings)
            {
                if (curveBinding.path.Contains(leftPrefix))
                {
                    handedness = Handedness.Left;
                    return true;
                }
                else if (curveBinding.path.Contains(rightPrefix))
                {
                    handedness = Handedness.Right;
                    return true;
                }
            }

            handedness = Handedness.Left;
            return false;
        }

        public static void StoreAsset(Object asset, string folder, string name)
        {
            string targetFolder = Path.Combine("Assets", folder);
            CreateFolder(targetFolder);
            string path = Path.Combine(targetFolder, name);
            AssetDatabase.CreateAsset(asset, path);
            AssetDatabase.Refresh();
            Debug.Log($"Asset generated at {path}");
        }

        public static void CreateFolder(string targetFolder)
        {
            if (!Directory.Exists(targetFolder))
            {
                Directory.CreateDirectory(targetFolder);
            }
        }

        public static string GetGameObjectPath(Transform transform, Transform root)
        {
            string path = transform.name;
            while (transform.parent != null
                && transform.parent != root)
            {
                transform = transform.parent;
                path = $"{transform.name}/{path}";
            }
            return path;
        }

        public static bool GenerateObjectField<T>(ref T obj, string label = "") where T : Object
        {
            EditorGUI.BeginChangeCheck();
            obj = EditorGUILayout.ObjectField(label, obj, typeof(T), true) as T;
            return EditorGUI.EndChangeCheck();
        }

        private static void SmoothCurveTangets(ref AnimationCurve curve)
        {
            for (int i = 0; i < curve.keys.Length; i++)
            {
                AnimationUtility.SetKeyLeftTangentMode(curve, i, AnimationUtility.TangentMode.ClampedAuto);
                AnimationUtility.SetKeyRightTangentMode(curve, i, AnimationUtility.TangentMode.ClampedAuto);
            }
        }

    }

    public struct PoseFrame
    {
        public float time;
        public Pose pose;

        public PoseFrame(float time, Pose pose)
        {
            this.time = time;
            this.pose = pose;
        }
    }


    public class JointRecord
    {
        public HandJointId JointId { get; private set; }
        public string Path { get; private set; }

        private List<PoseFrame> _poseFrames = new List<PoseFrame>();

        public List<Keyframe> RotationX => FilterAngles(_poseFrames.Select(pf => new Keyframe(pf.time, pf.pose.rotation.eulerAngles.x)).ToList());
        public List<Keyframe> RotationY => FilterAngles(_poseFrames.Select(pf => new Keyframe(pf.time, pf.pose.rotation.eulerAngles.y)).ToList());
        public List<Keyframe> RotationZ => FilterAngles(_poseFrames.Select(pf => new Keyframe(pf.time, pf.pose.rotation.eulerAngles.z)).ToList());

        public List<Keyframe> PositionX => _poseFrames.Select(pf => new Keyframe(pf.time, pf.pose.position.x)).ToList();
        public List<Keyframe> PositionY => _poseFrames.Select(pf => new Keyframe(pf.time, pf.pose.position.y)).ToList();
        public List<Keyframe> PositionZ => _poseFrames.Select(pf => new Keyframe(pf.time, pf.pose.position.z)).ToList();

        public JointRecord(HandJointId jointId, string path)
        {
            JointId = jointId;
            Path = path;
        }

        public void RecordPose(float time, Pose pose)
        {
            _poseFrames.Add(new PoseFrame(time, pose));
        }

        private static List<Keyframe> FilterAngles(List<Keyframe> keyframes)
        {
            if (keyframes.Count < 1)
            {
                return keyframes;
            }

            List<Keyframe> filtered = new List<Keyframe>();
            filtered.Add(keyframes[0]);
            for (int i = 1; i < keyframes.Count; i++)
            {
                float prevValue = filtered[filtered.Count - 1].value;
                float value = keyframes[i].value;

                while (Mathf.Abs(prevValue - value) > 180f)
                {
                    value += 360f * Mathf.Sign(prevValue - value);
                }

                filtered.Add(new Keyframe(keyframes[i].time, value));
            }

            return filtered;
        }
    }
}
