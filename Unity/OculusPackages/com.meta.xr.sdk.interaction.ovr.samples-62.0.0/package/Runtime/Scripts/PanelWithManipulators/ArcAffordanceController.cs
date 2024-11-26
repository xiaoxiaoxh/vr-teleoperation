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

namespace Oculus.Interaction.Samples
{
    /// <summary>
    /// It is not expected that typical users of the SlateWithManipulators prefab
    /// should need to either interact with or understand this script.
    ///
    /// This script contains the logic for controlling the arc affordance, specifically
    /// the animation parameters controlling the "bend" as well as the shading parameters
    /// permitting the material to fade.
    /// </summary>
    public class ArcAffordanceController : MonoBehaviour
    {
        [SerializeField, Tooltip("The animator controlling the curvature of the affordance")]
        private Animator _animator;

        [SerializeField, Tooltip("The transform from which world-space distance will be calculated; intuitively, 'the center of the arc's circle'")]
        private Transform _pivot;

        [SerializeField, Tooltip("The function converting distance (from the pivot, a world-space observation) into curvature (an animation parameter)")]
        private AnimationCurve _distanceToCurvatureCurve;

        [SerializeField, Tooltip("The renderer for the arc affordance, on which transparency values must be set.")]
        private SkinnedMeshRenderer _renderer;

        [SerializeField, Tooltip("The bone at the 'top' end of the arc's armature")]
        private Transform _topBone;

        [SerializeField, Tooltip("The bone at the 'bottom' end of the arc's armature")]
        private Transform _bottomBone;

        private Vector4[] _endPositions;

        private void Start()
        {
            _endPositions = new Vector4[2];
            _endPositions[0].w = 1f;
            _endPositions[1].w = 1f;
        }

        private void Update()
        {
            _animator.SetFloat("curvature", _distanceToCurvatureCurve.Evaluate(Vector3.Distance(this.transform.position, _pivot.position)));

            _endPositions[0].x = _topBone.position.x;
            _endPositions[0].y = _topBone.position.y;
            _endPositions[0].z = _topBone.position.z;
            _endPositions[1].x = _bottomBone.position.x;
            _endPositions[1].y = _bottomBone.position.y;
            _endPositions[1].z = _bottomBone.position.z;
            _renderer.material.SetVectorArray("_WorldSpaceFadePoints", _endPositions);
        }
    }
}
