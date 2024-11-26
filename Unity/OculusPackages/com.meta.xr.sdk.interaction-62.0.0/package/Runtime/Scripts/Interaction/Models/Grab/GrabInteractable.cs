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

namespace Oculus.Interaction
{
    /// <summary>
    /// Makes an object grabbable by controllers so long as it's within arm's reach.
    /// </summary>
    public class GrabInteractable : PointerInteractable<GrabInteractor, GrabInteractable>,
                                      IRigidbodyRef, ICollidersRef
    {
        private Collider[] _colliders;
        public Collider[] Colliders => _colliders;

        /// <summary>
        /// The Rigidbody of the object.
        /// </summary>
        [Tooltip("The Rigidbody of the object.")]
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
        /// If true, use the closest point to the interactor as the grab source.
        /// </summary>
        [Tooltip("If true, use the closest point to the interactor as the grab source.")]
        [SerializeField]
        private bool _useClosestPointAsGrabSource;

        /// <summary>
        ///
        /// </summary>
        [Tooltip(" ")]
        [SerializeField]
        private float _releaseDistance = 0f;

        /// <summary>
        /// Forces a release on all other grabbing interactors when grabbed by a new interactor.
        /// </summary>
        [Tooltip("Forces a release on all other grabbing interactors when grabbed by a new interactor.")]
        [SerializeField]
        private bool _resetGrabOnGrabsUpdated = true;

        /// <summary>
        /// The <cref="PhysicsGrabbable" /> used when you grab the interactable.
        /// </summary>
        [Tooltip("The PhysicsGrabbable used when you grab the interactable.")]
        [SerializeField, Optional]
        private PhysicsGrabbable _physicsGrabbable = null;

        private static CollisionInteractionRegistry<GrabInteractor, GrabInteractable> _grabRegistry = null;

        #region Properties
        public bool UseClosestPointAsGrabSource
        {
            get
            {
                return _useClosestPointAsGrabSource;
            }
            set
            {
                _useClosestPointAsGrabSource = value;
            }
        }
        public float ReleaseDistance
        {
            get
            {
                return _releaseDistance;
            }
            set
            {
                _releaseDistance = value;
            }
        }

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
        #endregion

        protected override void Awake()
        {
            base.Awake();
        }

        protected override void Start()
        {
            this.BeginStart(ref _started, () => base.Start());
            this.AssertField(Rigidbody, nameof(Rigidbody));
            if (_grabRegistry == null)
            {
                _grabRegistry = new CollisionInteractionRegistry<GrabInteractor, GrabInteractable>();
                SetRegistry(_grabRegistry);
            }
            _colliders = Rigidbody.GetComponentsInChildren<Collider>();
            this.AssertCollectionField(_colliders, nameof(_colliders),
               $"The associated {AssertUtils.Nicify(nameof(Rigidbody))} must have at least one Collider.");
            this.EndStart(ref _started);
        }

        /// <summary>
        /// Determines the position of the grabbed object. This is used as the location from which the object will be grabbed.
        /// </summary>
        /// <param name="target">The Transform of the interactor.</param>
        public Pose GetGrabSourceForTarget(Pose target)
        {
            if (_grabSource == null && !_useClosestPointAsGrabSource)
            {
                return target;
            }

            if (_useClosestPointAsGrabSource)
            {
                return new Pose(
                    Collisions.ClosestPointToColliders(target.position, _colliders),
                    target.rotation);
            }

            return _grabSource.GetPose();
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
        /// Adds a release distance to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalReleaseDistance(float releaseDistance)
        {
            _releaseDistance = releaseDistance;
        }

        /// <summary>
        /// Adds a physics grabbable to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalPhysicsGrabbable(PhysicsGrabbable physicsGrabbable)
        {
            _physicsGrabbable = physicsGrabbable;
        }
        #endregion
    }
}
