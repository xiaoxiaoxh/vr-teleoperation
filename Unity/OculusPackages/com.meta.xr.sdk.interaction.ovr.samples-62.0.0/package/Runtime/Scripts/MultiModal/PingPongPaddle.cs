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
using System.Collections;
using UnityEngine;
using static Oculus.Interaction.AudioPhysics;

namespace Oculus.Interaction.Samples
{
    public class PingPongPaddle : MonoBehaviour, ITransformer
    {
        [SerializeField]
        private HandGrabInteractable _leftHandGrabInteractable;
        [SerializeField]
        private HandGrabInteractable _rightHandGrabInteractable;

        [SerializeField]
        private Rigidbody _rigidbody;
        [SerializeField]
        private AnimationCurve _collisionStrength;

        private const float _timeBetweenCollisions = 0.1f;
        private WaitForSeconds _hapticsWait = new WaitForSeconds(0.1f);

        private CollisionEvents _collisionEvents;
        private float _timeAtLastCollision = 0f;

        protected bool _started = false;

        private OVRInput.Controller _activeController;
        private IGrabbable _grabbable;
        private Pose _grabDeltaInLocalSpace;


        protected virtual void Start()
        {
            this.BeginStart(ref _started);
            this.AssertField(_rigidbody, nameof(_rigidbody));
            _collisionEvents = _rigidbody.gameObject.AddComponent<CollisionEvents>();
            this.EndStart(ref _started);
        }


        protected virtual void OnEnable()
        {
            if (_started)
            {
                _collisionEvents.WhenCollisionEnter += HandleCollisionEnter;
                _leftHandGrabInteractable.WhenStateChanged += HandleLeftHandGrabInteractableStateChanged;
                _rightHandGrabInteractable.WhenStateChanged += HandleRightHandGrabInteractableStateChanged;
            }
        }

        protected virtual void OnDisable()
        {
            if (_started)
            {
                _collisionEvents.WhenCollisionEnter -= HandleCollisionEnter;
                _leftHandGrabInteractable.WhenStateChanged -= HandleLeftHandGrabInteractableStateChanged;
                _rightHandGrabInteractable.WhenStateChanged -= HandleRightHandGrabInteractableStateChanged;
            }
        }

        private void HandleLeftHandGrabInteractableStateChanged(InteractableStateChangeArgs stateChange)
        {
            if (stateChange.NewState == InteractableState.Select)
            {
                _activeController |= OVRInput.Controller.LTouch;
            }
            else if (stateChange.PreviousState == InteractableState.Select)
            {
                _activeController &= ~OVRInput.Controller.LTouch;
            }
        }

        private void HandleRightHandGrabInteractableStateChanged(InteractableStateChangeArgs stateChange)
        {
            if (stateChange.NewState == InteractableState.Select)
            {
                _activeController |= OVRInput.Controller.RTouch;
            }
            else if (stateChange.PreviousState == InteractableState.Select)
            {
                _activeController &= ~OVRInput.Controller.RTouch;
            }
        }

        private void HandleCollisionEnter(Collision collision)
        {
            TryPlayCollisionAudio(collision);
        }

        private void TryPlayCollisionAudio(Collision collision)
        {
            float collisionMagnitude = collision.relativeVelocity.sqrMagnitude;

            if (collision.collider.gameObject == null)
            {
                return;
            }

            float deltaTime = Time.time - _timeAtLastCollision;
            if (_timeBetweenCollisions > deltaTime)
            {
                return;
            }

            _timeAtLastCollision = Time.time;

            PlayCollisionHaptics(collisionMagnitude);
        }

        private void PlayCollisionHaptics(float strength)
        {
            float pitch = _collisionStrength.Evaluate(strength);
            StartCoroutine(HapticsRoutine(pitch, _activeController));
        }

        private IEnumerator HapticsRoutine(float pitch, OVRInput.Controller controller)
        {
            OVRInput.SetControllerVibration(pitch * 0.5f, pitch * 0.2f, controller);
            yield return _hapticsWait;
            OVRInput.SetControllerVibration(0, 0, controller);
        }

        public void Initialize(IGrabbable grabbable)
        {
            _grabbable = grabbable;
        }

        public void BeginTransform()
        {
            Pose grabPoint = _grabbable.GrabPoints[0];
            Transform targetTransform = _rigidbody.transform;
            _grabDeltaInLocalSpace = new Pose(targetTransform.InverseTransformVector(grabPoint.position - targetTransform.position),
                                            Quaternion.Inverse(grabPoint.rotation) * targetTransform.rotation);
        }

        public void UpdateTransform()
        {
            Pose grabPoint = _grabbable.GrabPoints[0];
            _rigidbody.MoveRotation(grabPoint.rotation * _grabDeltaInLocalSpace.rotation);
            _rigidbody.MovePosition(grabPoint.position - _rigidbody.transform.TransformVector(_grabDeltaInLocalSpace.position));
        }

        public void EndTransform() { }
    }
}
