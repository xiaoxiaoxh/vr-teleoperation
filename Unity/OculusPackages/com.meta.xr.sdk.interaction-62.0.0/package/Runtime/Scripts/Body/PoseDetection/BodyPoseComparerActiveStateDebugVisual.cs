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

using Oculus.Interaction.Body.Input;
using UnityEngine;

namespace Oculus.Interaction.Body.PoseDetection
{
    /// <summary>
    /// Debugs each JointComparerConfig in a BodyPoseComparerActiveState.
    /// Debug spheres are placed at each joint, and will change from Red to Orange to Green to signify the status of each joint (how close the joint is to being within the recognition threshold).
    /// When all joints are green, the pose recognizer would be Active.
    /// </summary>
    public class BodyPoseComparerActiveStateDebugVisual : MonoBehaviour
    {
        /// <summary>
        /// The PoseComparer to debug.
        /// </summary>
        [Tooltip("The PoseComparer to debug.")]
        [SerializeField]
        private BodyPoseComparerActiveState _bodyPoseComparer;

        /// <summary>
        /// Gizmos will be drawn at joint positions of this body pose.
        /// </summary>
        [Tooltip("Gizmos will be drawn at joint positions of this body pose.")]
        [SerializeField, Interface(typeof(IBodyPose))]
        private UnityEngine.Object _bodyPose;
        private IBodyPose BodyPose;

        /// <summary>
        /// The root transform of the body skeleton. Debug gizmos will be drawn in the local space of this transform.
        /// </summary>
        [Tooltip("The root transform of the body skeleton. Debug " +
            "gizmos will be drawn in the local space of this transform.")]
        [SerializeField]
        private Transform _root;

        /// <summary>
        /// The radius of the debug spheres.
        /// </summary>
        [Tooltip("The radius of the debug spheres.")]
        [SerializeField, Delayed]
        private float _radius = 0.1f;

        public float Radius
        {
            get => _radius;
            set => _radius = value;
        }

        protected virtual void Awake()
        {
            BodyPose = _bodyPose as IBodyPose;
        }

        protected virtual void Start()
        {
            this.AssertField(_bodyPoseComparer, nameof(_bodyPoseComparer));
            this.AssertField(BodyPose, nameof(BodyPose));
            this.AssertField(_root, nameof(BodyPose));
        }

        protected virtual void Update()
        {
            DrawJointSpheres();
        }

        private void DrawJointSpheres()
        {
            var featureStates = _bodyPoseComparer.FeatureStates;
            foreach (var kvp in featureStates)
            {
                BodyJointId joint = kvp.Key.Joint;
                var state = kvp.Value;
                if (BodyPose.GetJointPoseFromRoot(joint, out Pose pose))
                {
                    Vector3 jointPos = _root.TransformPoint(pose.position);

                    Color color;
                    if (state.Delta <= state.MaxDelta)
                    {
                        color = Color.green;
                    }
                    else if (state.MaxDelta > 0)
                    {
                        float amt = (state.Delta / state.MaxDelta) / 2f;
                        color = Color.Lerp(Color.yellow, Color.red, amt);
                    }
                    else
                    {
                        color = Color.red;
                    }

                    DebugGizmos.LineWidth = _radius / 2f;
                    DebugGizmos.Color = color;
                    DebugGizmos.DrawPoint(jointPos);
                }
            }
        }

        #region Inject

        public void InjectAllBodyPoseComparerActiveStateDebugVisual(
            BodyPoseComparerActiveState bodyPoseComparer,
            IBodyPose bodyPose, Transform root)
        {
            InjectBodyPoseComparer(bodyPoseComparer);
            InjectBodyPose(bodyPose);
            InjectRootTransform(root);
        }

        public void InjectRootTransform(Transform root)
        {
            _root = root;
        }

        public void InjectBodyPoseComparer(BodyPoseComparerActiveState bodyPoseComparer)
        {
            _bodyPoseComparer = bodyPoseComparer;
        }

        public void InjectBodyPose(IBodyPose bodyPose)
        {
            _bodyPose = bodyPose as UnityEngine.Object;
            BodyPose = bodyPose;
        }

        #endregion
    }
}
