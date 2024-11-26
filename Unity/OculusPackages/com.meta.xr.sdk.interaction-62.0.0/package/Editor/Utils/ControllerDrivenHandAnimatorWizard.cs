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
using UnityEngine;
using UnityEditor;
using UnityEditor.Animations;
using System.Collections.Generic;
using System.IO;

namespace Oculus.Interaction.Utils
{
    /// <summary>
    /// This wizard helps creating a set of fixed Animation Clips using HandTracking
    /// to be used in a skinned synthetic hand with an Animator.
    /// Assign a HandVisual and click the buttons as you perform the relevant
    /// poses with your tracked hand. The output will be an Animator that can be directly
    /// used in a Skinned hand. Once you are done you can automatically create the opposite
    /// hand data by providing the strings internally used for differentiating the left and
    /// right transforms. (typically _l_ and _r_)
    /// </summary>
    public class ControllerDrivenHandAnimatorWizard : EditorWindow
    {
        [SerializeField]
        private HandVisual _handVisual;
        [SerializeField]
        private string _folder = "GeneratedAnimations";
        [SerializeField]
        private string _controllerName = "HandController";

        [SerializeField]
        private AnimationClip _handFist;
        [SerializeField]
        private AnimationClip _hand3qtrFist;
        [SerializeField]
        private AnimationClip _handMidFist;
        [SerializeField]
        private AnimationClip _handPinch;
        [SerializeField]
        private AnimationClip _handSlide;
        [SerializeField]
        private AnimationClip _handCap;
        [SerializeField]
        private AnimationClip _thumbUp;
        [SerializeField]
        private AnimationClip _indexPoint;

        [SerializeField]
        private AvatarMask _indexMask;
        [SerializeField]
        private AvatarMask _thumbMask;

        [SerializeField]
        private string _handLeftPrefix = "_l_";
        [SerializeField]
        private string _handRightPrefix = "_r_";

        private Transform Root => _handVisual.Joints[0].parent;

        private GUIStyle _richTextStyle;
        private Vector2 _scrollPos = Vector2.zero;

        private static readonly List<HandJointId> INDEX_MASK = new List<HandJointId>()
        {
            HandJointId.HandIndex1,
            HandJointId.HandIndex2,
            HandJointId.HandIndex3,
            HandJointId.HandIndexTip
        };

        private static readonly List<HandJointId> THUMB_MASK = new List<HandJointId>()
        {
            HandJointId.HandThumb0,
            HandJointId.HandThumb1,
            HandJointId.HandThumb2,
            HandJointId.HandThumb3,
            HandJointId.HandThumbTip
        };

        private const string FLEX_PARAM = "Flex";
        private const string PINCH_PARAM = "Pinch";
        private const string INDEXSLIDE_PARAM = "IndexSlide";

        [MenuItem("Oculus/Interaction/Controller Driven Hand Animator Generator")]
        private static void CreateWizard()
        {
            ControllerDrivenHandAnimatorWizard window = EditorWindow.GetWindow<ControllerDrivenHandAnimatorWizard>();
            window.titleContent = new GUIContent("Controller Driven Hand Animator Generator");
            window.Show();
        }

        private void OnEnable()
        {
            _richTextStyle = EditorGUIUtility.GetBuiltinSkin(EditorGUIUtility.isProSkin ? EditorSkin.Scene : EditorSkin.Inspector).label;
            _richTextStyle.richText = true;
            _richTextStyle.wordWrap = true;
        }

        private void OnGUI()
        {
            GUILayout.Label("This tool generates an Animator for a Skinned Hand that moves accordingly to the controller input.", _richTextStyle);

            _scrollPos = GUILayout.BeginScrollView(_scrollPos);

            GUILayout.Label("<b>1</b> Assign the Hand Visual to record animations from in <b>Play Mode</b> and paths to store the generated files. ", _richTextStyle);

            HandAnimationUtils.GenerateObjectField(ref _handVisual, "Hand Visual");
            _folder = EditorGUILayout.TextField("Assets sub-folder", _folder);
            _controllerName = EditorGUILayout.TextField("Animator name", _controllerName);

            GUILayout.Space(20);
            GUILayout.Label("<b>2</b> Record the HandVisual poses in <b>Play Mode</b> or directly assign the animation clips to use for the different animator states.", _richTextStyle);
            DrawAnimationControls(ref _handFist, "HandFist");
            DrawAnimationControls(ref _hand3qtrFist, "Hand3qtrFist");
            DrawAnimationControls(ref _handMidFist, "HandMidFist");
            DrawAnimationControls(ref _handPinch, "HandPinch");
            DrawAnimationControls(ref _handSlide, "HandSlide");
            DrawAnimationControls(ref _handCap, "HandCap");
            DrawAnimationControls(ref _thumbUp, "ThumbUp");
            DrawAnimationControls(ref _indexPoint, "IndexPoint");

            GUILayout.Space(20);
            GUILayout.Label("<b>3</b> Generate the AvatarMasks for the Thumb and Index fingers from the HandVisual", _richTextStyle);
            DrawMaskControls(ref _indexMask, INDEX_MASK, "indexMask");
            DrawMaskControls(ref _thumbMask, THUMB_MASK, "thumbMask");

            GUILayout.Space(20);
            GUILayout.Label("<b>4</b> With all the clips and masks, generate the animator", _richTextStyle);

            if (GUILayout.Button("Generate Animator", GUILayout.Height(40)))
            {
                GenerateAnimatorAsset();
            }

            GUILayout.Space(20);
            GUILayout.Label("<b>5</b> Generate a mirrored copy of the Animator for the opposite hand", _richTextStyle);
            _handLeftPrefix = EditorGUILayout.TextField("Left prefix", _handLeftPrefix);
            _handRightPrefix = EditorGUILayout.TextField("Right prefix", _handRightPrefix);
            if (GUILayout.Button("Generate Mirrored Animator"))
            {
                GenerateMirrorAnimatorAsset();
            }

            GUILayout.EndScrollView();
        }

        private void DrawAnimationControls(ref AnimationClip clip, string name)
        {
            GUILayout.BeginHorizontal();
            HandAnimationUtils.GenerateObjectField(ref clip, name);
            if (GUILayout.Button("Record"))
            {
                clip = GenerateClipAsset(name);
            }
            GUILayout.EndHorizontal();
        }

        private void DrawMaskControls(ref AvatarMask mask, List<HandJointId> maskedJoints, string name)
        {
            GUILayout.BeginHorizontal();
            HandAnimationUtils.GenerateObjectField(ref mask, name);
            if (GUILayout.Button("Generate"))
            {
                mask = GenerateMaskAsset(maskedJoints, name);
            }
            GUILayout.EndHorizontal();
        }

        private void GenerateAnimatorAsset()
        {
            HandClips clips = new HandClips()
            {
                handFist = _handFist,
                hand3qtrFist = _hand3qtrFist,
                handMidFist = _handMidFist,
                handPinch = _handPinch,
                handSlide = _handSlide,
                handCap = _handCap,
                thumbUp = _thumbUp,
                indexPoint = _indexPoint,

                indexMask = _indexMask,
                thumbMask = _thumbMask
            };

            string path = GenerateAnimatorPath(string.Empty);
            CreateAnimator(path, clips);
        }

        private void GenerateMirrorAnimatorAsset()
        {
            AnimationClip handFist = GenerateMirrorClipAsset(_handFist);
            AnimationClip hand3qtrFist = GenerateMirrorClipAsset(_hand3qtrFist);
            AnimationClip handMidFist = GenerateMirrorClipAsset(_handMidFist);
            AnimationClip handPinch = GenerateMirrorClipAsset(_handPinch);
            AnimationClip handSlide = GenerateMirrorClipAsset(_handSlide);
            AnimationClip handCap = GenerateMirrorClipAsset(_handCap);
            AnimationClip thumbUp = GenerateMirrorClipAsset(_thumbUp);
            AnimationClip indexPoint = GenerateMirrorClipAsset(_indexPoint);

            AvatarMask indexMask = GenerateMirrorMaskAsset(_indexMask);
            AvatarMask thumbMask = GenerateMirrorMaskAsset(_thumbMask);

            HandClips clips = new HandClips()
            {
                handFist = handFist,
                hand3qtrFist = hand3qtrFist,
                handMidFist = handMidFist,
                handPinch = handPinch,
                handSlide = handSlide,
                handCap = handCap,
                thumbUp = thumbUp,
                indexPoint = indexPoint,
                indexMask = indexMask,
                thumbMask = thumbMask
            };

            string path = GenerateAnimatorPath("_mirror");
            CreateAnimator(path, clips);
        }

        private AnimationClip GenerateClipAsset(string title)
        {
            AnimationClip clip = new AnimationClip();

            for (HandJointId jointId = HandJointId.HandStart; jointId < HandJointId.HandEnd; ++jointId)
            {
                Transform jointTransform = _handVisual.GetTransformByHandJointId(jointId);
                string path = GetGameObjectPath(jointTransform, Root);
                JointRecord record = new JointRecord(jointId, path);
                Pose pose = jointTransform.GetPose(Space.Self);
                record.RecordPose(0f, pose);
                HandAnimationUtils.WriteAnimationCurves(ref clip, record, false);
            }

            HandAnimationUtils.StoreAsset(clip, _folder, $"{title}.anim");
            return clip;
        }

        private AvatarMask GenerateMaskAsset(List<HandJointId> maskData, string title)
        {
            AvatarMask mask = new AvatarMask();
            List<string> paths = new List<string>(maskData.Count);

            foreach (var maskJoints in maskData)
            {
                Transform jointTransform = _handVisual.Joints[(int)maskJoints];
                string localPath = GetGameObjectPath(jointTransform, Root);
                paths.Add(localPath);
            }

            mask.transformCount = paths.Count;
            for (int i = 0; i < paths.Count; ++i)
            {
                mask.SetTransformPath(i, paths[i]);
                mask.SetTransformActive(i, true);
            }

            HandAnimationUtils.StoreAsset(mask, _folder, $"{title}.mask");
            return mask;
        }

        private AnimationClip GenerateMirrorClipAsset(AnimationClip originalClip)
        {
            AnimationClip mirrorClip = HandAnimationUtils.Mirror(originalClip,
                _handLeftPrefix, _handRightPrefix, true);
            HandAnimationUtils.StoreAsset(mirrorClip, _folder, $"{originalClip.name}_mirror.anim");
            return mirrorClip;
        }

        private AvatarMask GenerateMirrorMaskAsset(AvatarMask originalMask)
        {
            if (originalMask == null)
            {
                Debug.LogError("Please generate a valid mask first");
                return null;
            }

            AvatarMask mirrorMask = new AvatarMask();
            mirrorMask.transformCount = originalMask.transformCount;
            for (int i = 0; i < originalMask.transformCount; ++i)
            {
                string path = originalMask.GetTransformPath(i);
                if (path.Contains(_handLeftPrefix))
                {
                    path = path.Replace(_handLeftPrefix, _handRightPrefix);
                }
                else
                {
                    path = path.Replace(_handRightPrefix, _handLeftPrefix);
                }
                bool active = originalMask.GetTransformActive(i);
                mirrorMask.SetTransformPath(i, path);
                mirrorMask.SetTransformActive(i, active);
            }

            HandAnimationUtils.StoreAsset(mirrorMask, _folder, $"{originalMask.name}_mirror.mask");

            return mirrorMask;
        }

        private AnimatorController CreateAnimator(string path, HandClips clips)
        {
            if (!clips.IsComplete())
            {
                Debug.LogError("Missing clips and masks to generate the animator");
                return null;
            }
            AnimatorController animator = AnimatorController.CreateAnimatorControllerAtPath(path);

            animator.AddParameter(FLEX_PARAM, AnimatorControllerParameterType.Float);
            animator.AddParameter(PINCH_PARAM, AnimatorControllerParameterType.Float);
            animator.AddParameter(INDEXSLIDE_PARAM, AnimatorControllerParameterType.Float);

            animator.RemoveLayer(0);

            CreateLayer(animator, "Flex Layer", null);
            CreateLayer(animator, "Thumb Layer", clips.thumbMask);
            CreateLayer(animator, "Point Layer", clips.indexMask);

            CreateFlexStates(animator, 0, clips);
            CreateThumbUpStates(animator, 1, clips);
            CreatePointStates(animator, 2, clips);

            return animator;
        }

        private AnimatorControllerLayer CreateLayer(AnimatorController animator, string layerName, AvatarMask mask = null)
        {
            AnimatorControllerLayer layer = new AnimatorControllerLayer();
            layer.name = layerName;
            AnimatorStateMachine stateMachine = new AnimatorStateMachine();
            stateMachine.name = layer.name;
            AssetDatabase.AddObjectToAsset(stateMachine, animator);
            stateMachine.hideFlags = HideFlags.HideInHierarchy;
            layer.stateMachine = stateMachine;
            layer.avatarMask = mask;
            animator.AddLayer(layer);
            return layer;
        }

        private void CreateFlexStates(AnimatorController animator, int layerIndex, HandClips clips)
        {
            BlendTree blendTree;
            AnimatorState flexState = animator.CreateBlendTreeInController("Flex", out blendTree, layerIndex);
            blendTree.blendType = BlendTreeType.FreeformCartesian2D;
            blendTree.blendParameter = FLEX_PARAM;
            blendTree.blendParameterY = PINCH_PARAM;

            {
                BlendTree blendTreeSlide = blendTree.CreateBlendTreeChild(new Vector2(0f, 0f));
                blendTreeSlide.blendType = BlendTreeType.FreeformCartesian2D;
                blendTreeSlide.blendParameter = PINCH_PARAM;
                blendTreeSlide.blendParameterY = INDEXSLIDE_PARAM;

                blendTreeSlide.AddChild(clips.handCap, new Vector2(0f, 0f));
                blendTreeSlide.AddChild(clips.handPinch, new Vector2(1f, 0f));
                blendTreeSlide.AddChild(clips.handSlide, new Vector2(0f, 1f));
            }

            blendTree.AddChild(clips.handPinch, new Vector2(0f, 0.835f));
            blendTree.AddChild(clips.handPinch, new Vector2(0f, 1f));
            blendTree.AddChild(clips.handMidFist, new Vector2(0.5f, 0f));
            blendTree.AddChild(clips.handMidFist, new Vector2(0.5f, 1f));
            blendTree.AddChild(clips.hand3qtrFist, new Vector2(0.835f, 0f));
            blendTree.AddChild(clips.hand3qtrFist, new Vector2(0.835f, 1f));
            blendTree.AddChild(clips.handFist, new Vector2(1f, 0f));
            blendTree.AddChild(clips.handFist, new Vector2(1f, 1f));



            animator.layers[layerIndex].stateMachine.defaultState = flexState;
        }

        private void CreateThumbUpStates(AnimatorController animator, int layerIndex, HandClips clips)
        {
            if (clips.thumbUp == null)
            {
                Debug.LogError("No thumb clip provided");
                return;
            }
            AnimatorState thumbupState = animator.AddMotion(clips.thumbUp, layerIndex);
            animator.layers[layerIndex].stateMachine.defaultState = thumbupState;
        }

        private void CreatePointStates(AnimatorController animator, int layerIndex, HandClips clips)
        {
            BlendTree blendTree;
            AnimatorState flexState = animator.CreateBlendTreeInController("Point", out blendTree, layerIndex);
            blendTree.blendType = BlendTreeType.Simple1D;
            blendTree.blendParameter = FLEX_PARAM;
            blendTree.AddChild(clips.handCap, 0f);
            blendTree.AddChild(clips.indexPoint, 1f);
            blendTree.useAutomaticThresholds = true;
            animator.layers[layerIndex].stateMachine.defaultState = flexState;
        }

        private string GenerateAnimatorPath(string prefix)
        {
            string targetFolder = Path.Combine("Assets", _folder);
            HandAnimationUtils.CreateFolder(targetFolder);
            string path = Path.Combine(targetFolder, $"{_controllerName}{prefix}.controller");
            return path;
        }

        private static string GetGameObjectPath(Transform transform, Transform root)
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

        private class HandClips
        {
            public AnimationClip handFist;
            public AnimationClip hand3qtrFist;
            public AnimationClip handMidFist;
            public AnimationClip handPinch;
            public AnimationClip handSlide;
            public AnimationClip handCap;
            public AnimationClip thumbUp;
            public AnimationClip indexPoint;

            public AvatarMask indexMask;
            public AvatarMask thumbMask;

            public bool IsComplete()
            {
                return handFist != null
                    && hand3qtrFist != null
                    && handMidFist != null
                    && handPinch != null
                    && handSlide != null
                    && handCap != null
                    && thumbUp != null
                    && indexPoint != null
                    && indexMask != null
                    && thumbMask != null;
            }
        }
    }
}
