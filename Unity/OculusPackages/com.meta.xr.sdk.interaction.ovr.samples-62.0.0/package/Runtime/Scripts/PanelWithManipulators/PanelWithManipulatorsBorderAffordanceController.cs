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

using Oculus.Interaction.HandGrab;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Oculus.Interaction.Samples
{
    /// <summary>
    /// It is not expected that typical users of the SlateWithManipulators prefab
    /// should need to either interact with or understand this script.
    ///
    /// This script contains the logic controlling the border affordance for the slate,
    /// as well as prefab state logic. This affordance's behavior is highly specialized,
    /// and so too is the logic implemented here. Specifically, this logic deals with the
    /// shading of the "rail" affordance, the placement, animation, and shading of the
    /// "capsule" affordances, and the logic associated with prefab statefulness (i.e.,
    /// disabling the slate interactions and affordances when something else on the prefab
    /// is selected).
    /// </summary>
    public class PanelWithManipulatorsBorderAffordanceController : MonoBehaviour
    {
        /// <summary>
        /// Calculates the projected position of a world space point onto the edge of a rounded box. Note that this function
        /// is designed specifically for the slate border visual use case and includes a built in "dead zone" which rejects
        /// projections from within the rounded box's defining rectangle.
        /// </summary>
        /// <param name="worldSpacePoint">The world space point to be projected</param>
        /// <param name="targetTransform">The transform defining the space into which the projection should occur</param>
        /// <param name="boneTransform">Transform of the "bone" of the rounded box, which lies at the corner of the defining rectangle</param>
        /// <param name="arcRadius">The radius of the arcs which form the corners of the rounded box</param>
        /// <returns>If the world space point's planar projection lies within the defining rectangle, returns null. Otherwise, returns the projected position in world space.</returns>
        private static Vector3? _projectToRoundedBoxEdge(Vector3 worldSpacePoint, Transform targetTransform, Transform boneTransform, float arcRadius)
        {
            // Project the world space point onto the XY plane in the local space of the target transform.
            var localPoint = targetTransform.InverseTransformPointUnscaled(worldSpacePoint);
            localPoint.z = 0f;

            // Project the bone position to the target transform local space; this is the corner of the
            // defining rectangle for the rounded box. (A rounded box can be defined as the set of all
            // points which are exactly a given distance from, but not contained by, a rectangle.)
            var cornerPoint = targetTransform.InverseTransformPointUnscaled(boneTransform.position);

            float x = Mathf.Sign(localPoint.x) * Mathf.Min(Mathf.Abs(cornerPoint.x), Mathf.Abs(localPoint.x));
            float y = Mathf.Sign(localPoint.y) * Mathf.Min(Mathf.Abs(cornerPoint.y), Mathf.Abs(localPoint.y));
            bool fullWidth = Mathf.Abs(Mathf.Abs(x) - Mathf.Abs(cornerPoint.x)) < Mathf.Epsilon;
            bool fullHeight = Mathf.Abs(Mathf.Abs(y) - Mathf.Abs(cornerPoint.y)) < Mathf.Epsilon;
            if (fullWidth || fullHeight)
            {
                var pt = new Vector3(x, y, 0f);
                var dir = (localPoint - pt).normalized;
                return targetTransform.TransformPointUnscaled(pt + dir * arcRadius);
            }

            return null;
        }

        enum RailState
        {
            Hidden = 0,
            Hover = 1,
            Selected = 2
        }

        enum AffordanceState
        {
            Hidden = 0,
            Visible = 1
        }

        [Header("Interactables")]
        [SerializeField, Tooltip("The grab interactable for the slate itself (as opposed to the surrounding affordances)")]
        private GrabInteractable _grabInteractable;

        [SerializeField, Tooltip("The hand grab interactable for the slate itself (as opposed to the surrounding affordances)")]
        private HandGrabInteractable _handGrabInteractable;

        [SerializeField, Optional, Tooltip("The hand grab interactable for the slate itself (as opposed to the surrounding affordances)")]
        private RayInteractable _rayInteractable;

        [Header("Panel Signals")]
        [SerializeField, Tooltip("The state signaler for the SlateWithManipulators prefab")]
        private PanelWithManipulatorsStateSignaler _stateSignaler;

        [SerializeField, Optional, Tooltip("Holds the panel hover state")]
        private PanelHoverState _panelHoverState;

        [SerializeField, Tooltip("The grabbable associated with the slate itself (i.e., the grabbable with One- and TwoGrabFreeTransformers")]
        private Grabbable _grabbale;

        [Space(10)]
        [SerializeField, Tooltip("The transform of one of the bones of the rail affordance (used in calculating capsule placement)")]
        private Transform _boneTransform;

        [SerializeField, Tooltip("The radius of the arcs at the corners of the rail affordance (used in calculating capsule placement)")]
        private float _cornerArcRadius;

        [Space(10)]
        [SerializeField, Tooltip("The animator controlling the overall opacity of the rail affordance " +
            "(note that this is independent of the localized opacities associated with the capsule affordances)")]
        private Animator _railOpacityAnimator;
        [SerializeField, Tooltip("The transform being controlled by the rail opacity animator")]
        private Transform _railOpacityTransform;

        [Serializable]
        class Affordance
        {
            [SerializeField, Tooltip("The parent transform of the geometry (i.e., visuals) which should be moved to place the capsule affordance")]
            private Transform _geometry;

            [SerializeField, Tooltip("Then transform controlled by an animation whose X axis magnitude will be used to control the affordance's opacity")]
            private Transform _opacityTransform;

            [SerializeField, Tooltip("The animators (canonically geometry and opacity) whose 'state' variables should be controlled by this affordance")]
            private Animator[] _animators;

            private AffordanceState _animationState = AffordanceState.Hidden;
            public AffordanceState AnimationState
            {
                get
                {
                    return _animationState;
                }
                set
                {
                    if (value == _animationState)
                    {
                        return;
                    }

                    _animationState = value;

                    foreach (var animator in _animators)
                    {
                        animator.SetInteger("state", (int)_animationState);
                    }
                }
            }

            private Vector3 _lastKnownPositionParentSpace;
            public Vector3 LastKnownPositionParentSpace
            {
                get
                {
                    return _lastKnownPositionParentSpace;
                }
                set
                {
                    _lastKnownPositionParentSpace = value;
                }
            }

            public Transform Geometry
            {
                get
                {
                    return _geometry;
                }
            }

            public float Opacity
            {
                get
                {
                    return Mathf.Abs(_opacityTransform.localPosition.x);
                }
            }
        }

        [SerializeField, Tooltip("The capsule affordances")]
        private Affordance[] _affordances;

        [SerializeField, Tooltip("The renderer controlling shading for the rail affordance")]
        private SkinnedMeshRenderer _railRenderer;

        private Vector4[] _fadePoints;

        private MaterialPropertyBlock _materialPropertyBlock;

        class FadePoint
        {
            public int affordanceIndex;
            public bool removeFlag;
            public FadePoint(int index)
            {
                affordanceIndex = index;
                removeFlag = false;
            }
        }
        private Dictionary<int, FadePoint> _points;
        private HashSet<int> _affordancesInUse;
        private List<int> _deletePointKeys;

        private void Start()
        {
            this.AssertField(_grabInteractable, nameof(_grabInteractable));
            this.AssertField(_handGrabInteractable, nameof(_handGrabInteractable));
            this.AssertField(_stateSignaler, nameof(_stateSignaler));
            this.AssertField(_grabbale, nameof(_grabbale));
            this.AssertField(_railOpacityAnimator, nameof(_railOpacityAnimator));

            _materialPropertyBlock = new MaterialPropertyBlock();
            _fadePoints = new Vector4[4] { Vector4.zero, Vector4.zero, Vector4.zero, Vector4.zero };

            _points = new();
            _affordancesInUse = new();
            _deletePointKeys = new();

            _grabInteractable.WhenStateChanged += HandleInteractableStateChanged;
            _handGrabInteractable.WhenStateChanged += HandleInteractableStateChanged;
            if (_rayInteractable != null) _rayInteractable.WhenStateChanged += HandleInteractableStateChanged;

            _railOpacityAnimator.SetInteger("state", (int)RailState.Hidden);
            _stateSignaler.WhenStateChanged += HandleStateChanged;
            _grabbale.WhenPointerEventRaised += HandlePointerEvent;
        }

        private void OnDestroy()
        {
            _grabInteractable.WhenStateChanged -= HandleInteractableStateChanged;
            _handGrabInteractable.WhenStateChanged -= HandleInteractableStateChanged;
            if (_rayInteractable != null) _rayInteractable.WhenStateChanged -= HandleInteractableStateChanged;
            _stateSignaler.WhenStateChanged -= HandleStateChanged;
            _grabbale.WhenPointerEventRaised -= HandlePointerEvent;
        }

        private void CreateFadePoint(int eventIdentifier)
        {
            var affordanceIndex = -1;
            for (int i = 0; i < _affordances.Length; i++)
            {
                if (!_affordancesInUse.Contains(i))
                {
                    affordanceIndex = i;
                    _affordancesInUse.Add(i);
                    break;
                }
            }
            if (affordanceIndex >= 0)
            {
                _points.Add(eventIdentifier, new FadePoint(affordanceIndex));
            }
        }

        private void HandlePointerEvent(PointerEvent evt)
        {
            switch (evt.Type)
            {
                case PointerEventType.Hover:
                case PointerEventType.Unselect:
                case PointerEventType.Move:
                    var position = _projectToRoundedBoxEdge(evt.Pose.position, _grabbale.Transform, _boneTransform, _cornerArcRadius);
                    var pointPosition = evt.Pose.position;
                    if (position.HasValue)
                    {
                        pointPosition = position.Value;
                    }

                    if (!_points.ContainsKey(evt.Identifier))
                    {
                        CreateFadePoint(evt.Identifier);
                    }
                    else
                    {
                        _points[evt.Identifier].removeFlag = false;
                        _affordances[_points[evt.Identifier].affordanceIndex].LastKnownPositionParentSpace = _grabbale.Transform.InverseTransformPoint(pointPosition);
                    }
                    break;
                case PointerEventType.Unhover:
                case PointerEventType.Cancel:
                    if (_points.ContainsKey(evt.Identifier))
                    {
                        _points[evt.Identifier].removeFlag = true;
                    }
                    break;
                default:
                    break;
            }
        }

        private void SetRailAnimatorState()
        {
            var railState = RailState.Hidden;

            if (_panelHoverState is not null)
            {
                if (_panelHoverState.Hovered)
                {
                    railState = RailState.Hover;
                }
                else
                {
                    railState = RailState.Hidden;
                }
            }
            else
            {
                railState = RailState.Hover;
            }

            if (_grabbale.SelectingPoints.Count > 0)
            {
                railState = RailState.Selected;
            }

            var rayInteractableDisabled = _rayInteractable != null ? _rayInteractable.State == InteractableState.Disabled : true;
            var interactablesDisabled = _grabInteractable.State == InteractableState.Disabled && _handGrabInteractable.State == InteractableState.Disabled && rayInteractableDisabled;
            if (interactablesDisabled)
            {
                railState = RailState.Hidden;
            }

            _railOpacityAnimator.SetInteger("state", (int)railState);
        }

        private void UpdateFadePoints()
        {
            if (_points.Count > 0)
            {
                _deletePointKeys.Clear();
                foreach (var keyValuePair in _points)
                {
                    var fadePoint = keyValuePair.Value;
                    var affordance = _affordances[fadePoint.affordanceIndex];
                    affordance.AnimationState = fadePoint.removeFlag ? AffordanceState.Hidden : AffordanceState.Visible;

                    if (fadePoint.removeFlag && affordance.Opacity < 0.05f)
                    {
                        _deletePointKeys.Add(keyValuePair.Key);
                    }
                }
                foreach (var key in _deletePointKeys)
                {
                    _points.Remove(key, out var point);
                    _affordancesInUse.Remove(point.affordanceIndex);
                }
            }
        }

        private void UpdateMaterialProperties()
        {
            _materialPropertyBlock.SetFloat("_OpacityMultiplier", _railOpacityTransform.localPosition.x);
            _materialPropertyBlock.SetFloat("_SelectedOpacityParam", _railOpacityTransform.localPosition.y);

            var pointIndex = 0;
            foreach (var affordance in _affordances)
            {
                if (pointIndex >= _fadePoints.Length) break;
                var position = _grabbale.Transform.TransformPoint(affordance.LastKnownPositionParentSpace);
                affordance.Geometry.position = position;
                _fadePoints[pointIndex].x = position.x;
                _fadePoints[pointIndex].y = position.y;
                _fadePoints[pointIndex].z = position.z;
                _fadePoints[pointIndex].w = affordance.Opacity;
                pointIndex++;
            }

            _materialPropertyBlock.SetVectorArray("_WorldSpaceFadePoints", _fadePoints);
            _materialPropertyBlock.SetInteger("_UsedPointCount", pointIndex);

            _railRenderer.SetPropertyBlock(_materialPropertyBlock);
        }

        private void Update()
        {
            SetRailAnimatorState();
            UpdateFadePoints();
            UpdateMaterialProperties();
        }

        private void HandleInteractableStateChanged(InteractableStateChangeArgs args)
        {
            if (args.NewState == InteractableState.Select)
            {
                _stateSignaler.CurrentState = PanelWithManipulatorsStateSignaler.State.Selected;
            }
            else if (args.PreviousState == InteractableState.Select)
            {
                _stateSignaler.CurrentState = PanelWithManipulatorsStateSignaler.State.Default;
            }
        }

        private void HandleStateChanged(PanelWithManipulatorsStateSignaler.State state)
        {
            if (state != PanelWithManipulatorsStateSignaler.State.Default)
            {
                var rayInteractableNotSelected = _rayInteractable != null ? _rayInteractable.State != InteractableState.Select : true;
                if (_grabInteractable.State != InteractableState.Select && _handGrabInteractable.State != InteractableState.Select && rayInteractableNotSelected)
                {
                    _grabInteractable.enabled = false;
                    _handGrabInteractable.enabled = false;
                    if (_rayInteractable != null) _rayInteractable.enabled = false;
                }
            }
            else
            {
                _grabInteractable.enabled = true;
                _handGrabInteractable.enabled = true;
                if (_rayInteractable != null) _rayInteractable.enabled = true;
            }
        }
    }
}
