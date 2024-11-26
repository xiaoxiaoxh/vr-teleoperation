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

using Oculus.Interaction.Throw;
using UnityEngine;

namespace Oculus.Interaction
{
    /// <summary>
    /// DistanceGrabInteractor lets you grab interactables at a distance with controllers and will move them using configurable IMovements.
    /// It uses a IDistantCandidateComputer in order to Hover the best candidate.
    /// </summary>
    public class DistanceGrabInteractor : PointerInteractor<DistanceGrabInteractor, DistanceGrabInteractable>,
        IDistanceInteractor
    {
        /// <summary>
        /// The selection mechanism used to trigger the grab.
        /// </summary>
        [Tooltip("The selection mechanism to trigger the grab.")]
        [SerializeField, Interface(typeof(ISelector))]
        private UnityEngine.Object _selector;

        /// <summary>
        /// The center of the grab.
        /// </summary>
        [Tooltip("The center of the grab.")]
        [SerializeField, Optional]
        private Transform _grabCenter;

        /// <summary>
        /// The location where the interactable will move when selected.
        /// </summary>
        [Tooltip("The location where the interactable will move when selected.")]
        [SerializeField, Optional]
        private Transform _grabTarget;

        /// <summary>
        /// Determines how the object will move when thrown.
        /// </summary>
        [Tooltip("Determines how the object will move when thrown.")]
        [SerializeField, Interface(typeof(IThrowVelocityCalculator)), Optional]
        private UnityEngine.Object _velocityCalculator;
        public IThrowVelocityCalculator VelocityCalculator { get; set; }

        [SerializeField]
        private DistantCandidateComputer<DistanceGrabInteractor, DistanceGrabInteractable> _distantCandidateComputer
            = new DistantCandidateComputer<DistanceGrabInteractor, DistanceGrabInteractable>();

        private IMovement _movement;

        /// <summary>
        /// The origin of the frustrums used by <cref="DistantCandidateComputer" />.
        /// </summary>
        public Pose Origin => _distantCandidateComputer.Origin;

        /// <summary>
        /// The hitpoint of your controller's frustrum.
        /// </summary>
        public Vector3 HitPoint { get; private set; }

        /// <summary>
        /// A reference to the main Transform of the current Interactable.
        /// </summary>
        public IRelativeToRef DistanceInteractable => this.Interactable;

        protected override void Awake()
        {
            base.Awake();
            Selector = _selector as ISelector;
            VelocityCalculator = _velocityCalculator as IThrowVelocityCalculator;
            _nativeId = 0x4469737447726162;
        }

        protected override void Start()
        {
            this.BeginStart(ref _started, () => base.Start());
            this.AssertField(Selector, nameof(Selector));
            this.AssertField(_distantCandidateComputer, nameof(_distantCandidateComputer));

            if (_grabCenter == null)
            {
                _grabCenter = transform;
            }

            if (_grabTarget == null)
            {
                _grabTarget = _grabCenter;
            }

            if (_velocityCalculator != null)
            {
                this.AssertField(VelocityCalculator, nameof(VelocityCalculator));
            }
            this.EndStart(ref _started);
        }

        protected override void DoPreprocess()
        {
            transform.position = _grabCenter.position;
            transform.rotation = _grabCenter.rotation;
        }

        protected override DistanceGrabInteractable ComputeCandidate()
        {
            DistanceGrabInteractable bestCandidate = _distantCandidateComputer.ComputeCandidate(
                DistanceGrabInteractable.Registry, this, out Vector3 hitPoint);
            HitPoint = hitPoint;
            return bestCandidate;
        }

        protected override void InteractableSelected(DistanceGrabInteractable interactable)
        {
            _movement = interactable.GenerateMovement(_grabTarget.GetPose());
            base.InteractableSelected(interactable);
            interactable.WhenPointerEventRaised += HandleOtherPointerEventRaised;
        }

        protected override void InteractableUnselected(DistanceGrabInteractable interactable)
        {
            interactable.WhenPointerEventRaised -= HandleOtherPointerEventRaised;
            _movement?.StopAndSetPose(_movement.Pose);
            base.InteractableUnselected(interactable);
            _movement = null;

            ReleaseVelocityInformation throwVelocity = VelocityCalculator != null ?
                VelocityCalculator.CalculateThrowVelocity(interactable.transform) :
                new ReleaseVelocityInformation(Vector3.zero, Vector3.zero, Vector3.zero);
            interactable.ApplyVelocities(throwVelocity.LinearVelocity, throwVelocity.AngularVelocity);
        }

        private void HandleOtherPointerEventRaised(PointerEvent evt)
        {
            if (SelectedInteractable == null)
            {
                return;
            }

            if (evt.Type == PointerEventType.Select || evt.Type == PointerEventType.Unselect)
            {
                Pose toPose = _grabTarget.GetPose();
                if (SelectedInteractable.ResetGrabOnGrabsUpdated)
                {
                    _movement = SelectedInteractable.GenerateMovement(toPose);
                    SelectedInteractable.PointableElement.ProcessPointerEvent(
                        new PointerEvent(Identifier, PointerEventType.Move, _movement.Pose, Data));
                }
            }

            if (evt.Identifier == Identifier && evt.Type == PointerEventType.Cancel)
            {
                SelectedInteractable.WhenPointerEventRaised -= HandleOtherPointerEventRaised;
            }
        }

        protected override Pose ComputePointerPose()
        {
            if (SelectedInteractable != null)
            {
                return _movement.Pose;
            }
            return _grabTarget.GetPose();
        }

        protected override void DoSelectUpdate()
        {
            DistanceGrabInteractable interactable = _selectedInteractable;
            if (interactable == null)
            {
                return;
            }

            _movement.UpdateTarget(_grabTarget.GetPose());
            _movement.Tick();
        }

        #region Inject
        /// <summary>
        /// Adds a <cref="DistanceGrabInteractor"/> to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectAllDistanceGrabInteractor(ISelector selector,
            DistantCandidateComputer<DistanceGrabInteractor, DistanceGrabInteractable> distantCandidateComputer)
        {
            InjectSelector(selector);
            InjectDistantCandidateComputer(distantCandidateComputer);
        }

        /// <summary>
        /// Adds an <cref="ISelector"/> to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectSelector(ISelector selector)
        {
            _selector = selector as UnityEngine.Object;
            Selector = selector;
        }

        /// <summary>
        /// Adds a <cref="DistantCandidateComputer"/> to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectDistantCandidateComputer(DistantCandidateComputer<DistanceGrabInteractor, DistanceGrabInteractable> distantCandidateComputer)
        {
            _distantCandidateComputer = distantCandidateComputer;
        }

        /// <summary>
        /// Adds a grab center to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalGrabCenter(Transform grabCenter)
        {
            _grabCenter = grabCenter;
        }

        /// <summary>
        /// Adds a grab target to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalGrabTarget(Transform grabTarget)
        {
            _grabTarget = grabTarget;
        }

        /// <summary>
        /// Adds a <cref="IThrowVelocityCalculator"/> to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalVelocityCalculator(IThrowVelocityCalculator velocityCalculator)
        {
            _velocityCalculator = velocityCalculator as UnityEngine.Object;
            VelocityCalculator = velocityCalculator;
        }

        #endregion
    }
}
