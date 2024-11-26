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

using Oculus.Interaction.Grab;
using Oculus.Interaction.GrabAPI;
using Oculus.Interaction.Input;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Oculus.Interaction.HandGrab
{
    /// <summary>
    /// Makes an object grabbable by hands so long as it's within arm's reach.
    /// </summary>
    /// <remarks>
    /// A HandGrabInteractable indicates the properties about how a hand can HandGrab an object.
    /// It specifies the fingers that must perform the grab and the release and generates the events
    /// for the Pointable to move the object.
    /// Optionally it can  reference a list of differently scaled HandGrabPoses to inform the
    /// interactor about the best pose the hand should adopt when grabbing the object with different
    /// sized hands.
    /// </remarks>
    [Serializable]
    public partial class HandGrabInteractable : PointerInteractable<HandGrabInteractor, HandGrabInteractable>,
        IHandGrabInteractable, IRigidbodyRef, ICollidersRef
    {
        /// <summary>
        /// The Rigidbody of the object.
        /// </summary>
        [Tooltip("The Rigidbody of the object.")]
        [SerializeField]
        private Rigidbody _rigidbody;
        public Rigidbody Rigidbody => _rigidbody;

        /// <summary>
        /// The <cref="PhysicsGrabbable" /> used when you grab the object.
        /// </summary>
        [Tooltip("The PhysicsGrabbable used when you grab the object.")]
        [SerializeField, Optional]
        private PhysicsGrabbable _physicsGrabbable = null;

        /// <summary>
        /// Forces a release on all other grabbing interactors when grabbed by a new interactor.
        /// </summary>
        [Tooltip("Forces a release on all other grabbing interactors when grabbed by a new interactor.")]
        [SerializeField]
        private bool _resetGrabOnGrabsUpdated = true;
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

        /// <summary>
        /// A <cref="PoseMeasureParameters" /> used to modify the score of a pose.
        /// </summary>
        [Tooltip("A PoseMeasureParameters used to modify the score of a pose.")]
        [SerializeField, Optional]
        private PoseMeasureParameters _scoringModifier = new PoseMeasureParameters(0.8f);

        /// <summary>
        /// Defines the slippiness threshold so the interactor can slide along the interactable based on the
        /// strength of the grip. GrabSurfaces are required to slide. At min slippiness = 0, the interactor never moves.
        /// </summary>
        [SerializeField, Optional, Range(0f, 1f)]
        [Tooltip("Defines the slippiness threshold so the interactor can slide along the interactable based on the" +
            "strength of the grip. GrabSurfaces are required to slide. At min slippiness = 0, the interactor never moves.")]
        private float _slippiness = 0f;
        public float Slippiness
        {
            get
            {
                return _slippiness;
            }
            set
            {
                _slippiness = value;
            }
        }

        /// <summary>
        /// The grab types that the object supports.
        /// </summary>
        [Tooltip("The grab types that the object supports.")]
        [Space]
        [SerializeField]
        private GrabTypeFlags _supportedGrabTypes = GrabTypeFlags.All;

        /// <summary>
        /// Uses the state of the fingers to define when a pinch grab starts and ends.
        /// </summary>
        [Tooltip("Uses the state of the fingers to define when a pinch grab starts and ends.")]
        [SerializeField]
        private GrabbingRule _pinchGrabRules = GrabbingRule.DefaultPinchRule;

        /// <summary>
        /// Uses the state of the fingers to define when a palm grab starts and ends.
        /// </summary>
        [Tooltip("Uses the state of the fingers to define when a palm grab starts and ends.")]
        [SerializeField]
        private GrabbingRule _palmGrabRules = GrabbingRule.DefaultPalmRule;

        [Header("Movement", order = -1)]

        /// <summary>
        /// Determines how the object will move when selected.
        /// </summary>
        [Tooltip("Determines how the object will move when selected.")]
        [SerializeField, Interface(typeof(IMovementProvider))]
        [Optional(OptionalAttribute.Flag.AutoGenerated)]
        private UnityEngine.Object _movementProvider;
        private IMovementProvider MovementProvider { get; set; }

        /// <summary>
        /// Determines when the hand will be aligned with the object.
        /// </summary>
        [Tooltip("Determines when the hand will be aligned with the object.")]
        [SerializeField]
        private HandAlignType _handAligment = HandAlignType.AlignOnGrab;
        public HandAlignType HandAlignment
        {
            get
            {
                return _handAligment;
            }
            set
            {
                _handAligment = value;
            }
        }

        /// <summary>
        /// Scaled <cref="HandGrabPose" />s that represent the hand's position during a grab.
        /// </summary>
        [Tooltip(" ")]
        [SerializeField]
        [Optional(OptionalAttribute.Flag.DontHide)]
        [UnityEngine.Serialization.FormerlySerializedAs("_handGrabPoints")]
        private List<HandGrabPose> _handGrabPoses = new List<HandGrabPose>();
        public List<HandGrabPose> HandGrabPoses => _handGrabPoses;

        /// <summary>
        /// General getter for the transform of the object this interactable refers to.
        /// </summary>
        public Transform RelativeTo => _rigidbody.transform;

        public PoseMeasureParameters ScoreModifier => _scoringModifier;

        public GrabTypeFlags SupportedGrabTypes => _supportedGrabTypes;
        public GrabbingRule PinchGrabRules => _pinchGrabRules;
        public GrabbingRule PalmGrabRules => _palmGrabRules;

        public Collider[] Colliders { get; private set; }

        private GrabPoseFinder _grabPoseFinder;

        private static CollisionInteractionRegistry<HandGrabInteractor, HandGrabInteractable> _registry = null;

        #region editor events
        protected virtual void Reset()
        {
            InjectRigidbody(this.GetComponentInParent<Rigidbody>());
            InjectOptionalPointableElement(this.GetComponentInParent<Grabbable>());
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
            if (_registry == null)
            {
                _registry = new CollisionInteractionRegistry<HandGrabInteractor, HandGrabInteractable>();
                SetRegistry(_registry);
            }
            Colliders = Rigidbody.GetComponentsInChildren<Collider>();
            this.AssertCollectionField(Colliders, nameof(Colliders),
                whyItFailed: $"The associated {nameof(Rigidbody)} must have at least one collider.");

            if (MovementProvider == null)
            {
                IMovementProvider movementProvider;
                movementProvider = this.gameObject.AddComponent<MoveTowardsTargetProvider>();
                InjectOptionalMovementProvider(movementProvider);
            }
            _grabPoseFinder = new GrabPoseFinder(_handGrabPoses, this.RelativeTo);
            this.EndStart(ref _started);
        }

        #region pose moving

        /// <summary>
        /// When you grab the object, this determines where the object currently is and where it should move.
        /// </summary>
        /// <param name="from">The starting position.</param>
        /// <param name="to">The ending position.</param>
        public IMovement GenerateMovement(in Pose from, in Pose to)
        {
            IMovement movement = MovementProvider.CreateMovement();
            movement.StopAndSetPose(from);
            movement.MoveTo(to);
            return movement;
        }

        /// <summary>
        /// Applies velocities to the interactable's <cref="PhysicsGrabbable" /> if it has one.
        /// </summary>
        /// <param name="linearVelocity">The linear velocity to apply.</param>
        /// <param name="angularVelocity">The angular velocity to apply.</param>
        public void ApplyVelocities(Vector3 linearVelocity, Vector3 angularVelocity)
        {
            if (_physicsGrabbable == null)
            {
                return;
            }
            _physicsGrabbable.ApplyVelocities(linearVelocity, angularVelocity);
        }

        #endregion
        /// <summary>
        /// Determines the best pose for the hand to snap to.
        /// </summary>
        /// <param name="userPose">The pose of the hand.</param>
        /// <param name="handScale">The scale of the hand.</param>
        /// <param name="handedness">The handedness of the hand.</param>
        /// <param name="result">The best pose.</param>
        public bool CalculateBestPose(Pose userPose, float handScale, Handedness handedness,
            ref HandGrabResult result)
        {
            GrabPoseFinder.FindResult findResult = _grabPoseFinder.FindBestPose(userPose,
                handScale, handedness, _scoringModifier, ref result);

            if (findResult == GrabPoseFinder.FindResult.NotFound)
            {
                result.HasHandPose = false;
                result.Score = GrabPoseHelper.CollidersScore(userPose.position, Colliders, out Vector3 hit);
                Pose worldSnap = new Pose(hit, userPose.rotation);
                result.RelativePose = RelativeTo.Delta(worldSnap);
            }

            return findResult != GrabPoseFinder.FindResult.NotCompatible;
        }

        public bool UsesHandPose => _grabPoseFinder.UsesHandPose;

        /// <summary>
        /// Checks if the pose supports grabbing with the given hand.
        /// </summary>
        public bool SupportsHandedness(Handedness handedness)
        {
            return _grabPoseFinder.SupportsHandedness(handedness);
        }

        #region Inject

        /// <summary>
        /// Adds all required scripts for <cref="HandGrabInteractable" /> to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectAllHandGrabInteractable(GrabTypeFlags supportedGrabTypes,
            Rigidbody rigidbody,
            GrabbingRule pinchGrabRules, GrabbingRule palmGrabRules)
        {
            InjectSupportedGrabTypes(supportedGrabTypes);
            InjectRigidbody(rigidbody);
            InjectPinchGrabRules(pinchGrabRules);
            InjectPalmGrabRules(palmGrabRules);
        }

        /// <summary>
        /// Adds supported grab types to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectSupportedGrabTypes(GrabTypeFlags supportedGrabTypes)
        {
            _supportedGrabTypes = supportedGrabTypes;
        }

        /// <summary>
        /// Adds pinch grab rules to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectPinchGrabRules(GrabbingRule pinchGrabRules)
        {
            _pinchGrabRules = pinchGrabRules;
        }

        /// <summary>
        /// Adds palm grab rules to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectPalmGrabRules(GrabbingRule palmGrabRules)
        {
            _palmGrabRules = palmGrabRules;
        }

        /// <summary>
        /// Adds a Rigidbody to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectRigidbody(Rigidbody rigidbody)
        {
            _rigidbody = rigidbody;
        }

        /// <summary>
        /// Adds a score modifier to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalScoreModifier(PoseMeasureParameters scoreModifier)
        {
            _scoringModifier = scoreModifier;
        }

        /// <summary>
        /// Adds a physics grabbable to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalPhysicsGrabbable(PhysicsGrabbable physicsGrabbable)
        {
            _physicsGrabbable = physicsGrabbable;
        }

        /// <summary>
        /// Adds hand grab poses to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalHandGrabPoses(List<HandGrabPose> handGrabPoses)
        {
            _handGrabPoses = handGrabPoses;
        }

        /// <summary>
        /// Adds a movement provider to a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalMovementProvider(IMovementProvider provider)
        {
            _movementProvider = provider as UnityEngine.Object;
            MovementProvider = provider;
        }

        #endregion

    }
}
