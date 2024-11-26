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
using UnityEngine;

namespace Oculus.Interaction
{
    /// <summary>
    /// This interactable makes an object grabbable at a distance.
    /// </summary>
    public class DistanceGrabInteractable : PointerInteractable<DistanceGrabInteractor, DistanceGrabInteractable>,
        IRigidbodyRef, IRelativeToRef, ICollidersRef
    {
        private Collider[] _colliders;
        public Collider[] Colliders => _colliders;

        /// <summary>
        /// The RigidBody of the interactable.
        /// </summary>
        [Tooltip("The RigidBody of the interactable.")]
        [SerializeField]
        Rigidbody _rigidbody;
        public Rigidbody Rigidbody => _rigidbody;

        /// <summary>
        /// An optional origin point for the grab.
        /// </summary>
        [Tooltip("An optional origin point for the grab.")]
        [SerializeField, Optional]
        private Transform _grabSource;

        /// <summary>
        /// Forces a release on all other grabbing interactors when grabbed by a new interactor.
        /// </summary>
        [Tooltip("Forces a release on all other grabbing interactors when grabbed by a new interactor.")]
        [SerializeField]
        private bool _resetGrabOnGrabsUpdated = true;

        /// <summary>
        /// <cref="PhysicsGrabbable" /> used when you grab the interactable.
        /// </summary>
        [Tooltip("PhysicsGrabbable used when you grab the interactable.")]
        [SerializeField, Optional]
        private PhysicsGrabbable _physicsGrabbable = null;

        /// <summary>
        /// The <cref="IMovementProvider" /> specifies how the interactable will align with the grabber when selected.
        /// If no <cref="IMovementProvider" /> is set, the <cref="MoveTowardsTargetProvider" /> is created and used as the provider.
        /// </summary>
        [Tooltip("The IMovementProvider specifies how the interactable will align with the grabber when selected. If no IMovementProvider is set, the MoveTowardsTargetProvider is created and used as the provider.")]
        [Header("Snap")]
        [SerializeField, Optional, Interface(typeof(IMovementProvider))]
        private UnityEngine.Object _movementProvider;
        private IMovementProvider MovementProvider { get; set; }

        #region Properties
        public bool ResetGrabOnGrabsUpdated
        {
            get
            {
                return _resetGrabOnGrabsUpdated;
            }
            set
            {
                _resetGrabOnGrabsUpdated = value;
            }
        }

        public Transform RelativeTo => _grabSource;

        #endregion

        #region Editor Events

        protected virtual void Reset()
        {
            _rigidbody = this.GetComponentInParent<Rigidbody>();
            _physicsGrabbable = this.GetComponentInParent<PhysicsGrabbable>();
        }

        #endregion

        protected override void Awake()
        {
            base.Awake();
            MovementProvider = _movementProvider as IMovementProvider;
        }

        protected override void Start()
        {
            this.BeginStart(ref _started, () => base.Start());
            this.AssertField(Rigidbody, nameof(Rigidbody));
            _colliders = Rigidbody.GetComponentsInChildren<Collider>();
            if (MovementProvider == null)
            {
                MoveTowardsTargetProvider movementProvider = this.gameObject.AddComponent<MoveTowardsTargetProvider>();
                InjectOptionalMovementProvider(movementProvider);
            }
            if (_grabSource == null)
            {
                _grabSource = Rigidbody.transform;
            }
            this.EndStart(ref _started);
        }

        /// <summary>
        /// Moves the interactable to the provided position.
        /// </summary>
        public IMovement GenerateMovement(in Pose to)
        {
            Pose source = _grabSource.GetPose();
            IMovement movement = MovementProvider.CreateMovement();
            movement.StopAndSetPose(source);
            movement.MoveTo(to);
            return movement;
        }

        /// <summary>
        /// Applies velocities to the interactable's <cref="PhysicsGrabbable" /> if it has one.
        /// </summary>
        public void ApplyVelocities(Vector3 linearVelocity, Vector3 angularVelocity)
        {
            if (_physicsGrabbable == null)
            {
                return;
            }
            _physicsGrabbable.ApplyVelocities(linearVelocity, angularVelocity);
        }

        #region Inject

        /// <summary>
        /// Adds a Rigidbody to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectAllGrabInteractable(Rigidbody rigidbody)
        {
            InjectRigidbody(rigidbody);
        }

        /// <summary>
        /// Adds a Rigidbody to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectRigidbody(Rigidbody rigidbody)
        {
            _rigidbody = rigidbody;
        }

        /// <summary>
        /// Adds a grab source to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalGrabSource(Transform grabSource)
        {
            _grabSource = grabSource;
        }

        /// <summary>
        /// Adds a <cref="PhysicsGrabbable" /> to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalPhysicsGrabbable(PhysicsGrabbable physicsGrabbable)
        {
            _physicsGrabbable = physicsGrabbable;
        }

        /// <summary>
        /// Adds a <cref="IMovementProvider" /> to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalMovementProvider(IMovementProvider provider)
        {
            _movementProvider = provider as UnityEngine.Object;
            MovementProvider = provider;
        }
        #endregion
    }
}
