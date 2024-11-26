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

namespace Oculus.Interaction.Samples
{
    public class Slingshot : MonoBehaviour, ITransformer
    {
        [SerializeField]
        private Pose _neutralPose;
        [SerializeField]
        private Transform _holder;
        [SerializeField]
        private Transform _leftRubberPoint;
        [SerializeField]
        private Transform _rightRubberPoint;
        [SerializeField]
        private float _rubberAngle = 60f;

        [SerializeField]
        private AnimationCurve _translationResistance;
        [SerializeField]
        private AnimationCurve _aimingResistance;
        [SerializeField]
        private float _springForce = 0.1f;
        [SerializeField]
        private float _damping = 0.95f;
        [SerializeField]
        private float _slingshotStrength = 10f;

        [SerializeField]
        private HandGrabInteractable[] _handGrabInteractables;

        [Header("Feedback")]
        [SerializeField]
        private AudioSource _audioSource;
        [SerializeField]
        private AudioClip _stretchAudioClip;
        [SerializeField]
        private AnimationCurve _strecthAudioPitch;
        [SerializeField]
        private AnimationCurve _stretchAudioStep;


        private IGrabbable _grabbable;
        private Pose _grabDeltaInLocalSpace;

        private bool _isGrabbed;

        private Vector3 _positionVelocity = Vector3.zero;
        private float _rotationVelocity = 0.0f;

        private SlingshotPellet _loadedPellet;

        private WaitForSeconds _hapticsWait = new WaitForSeconds(_tensionStepLength * 0.5f);

        private float _lastTensionStep = 0f;
        private float _lastTensionTime = 0f;
        private const float _tensionStepLength = 0.1f;

        private void OnTriggerEnter(Collider other)
        {
            if (_loadedPellet != null)
            {
                return;
            }

            if (other.TryGetComponent(out SlingshotPellet pellet))
            {
                HandlePelletSnapped(pellet);
            }
        }


        private void HandlePelletSnapped(SlingshotPellet pellet)
        {
            HandGrabInteractor handGrabInteractor = pellet.HandGrabber;
            if (handGrabInteractor == null
                || handGrabInteractor.State != InteractorState.Select)
            {
                return;
            }
            foreach (var interactable in _handGrabInteractables)
            {
                if (handGrabInteractor.CanInteractWith(interactable))
                {
                    handGrabInteractor.ForceRelease();
                    handGrabInteractor.ForceSelect(interactable, true);
                    _loadedPellet = pellet;
                    _loadedPellet.Attach();
                    return;
                }
            }
        }

        public void Initialize(IGrabbable grabbable)
        {
            _grabbable = grabbable;
        }


        public void BeginTransform()
        {
            Pose grabPoint = _grabbable.GrabPoints[0];
            var targetTransform = _grabbable.Transform;
            _grabDeltaInLocalSpace = new Pose(targetTransform.InverseTransformVector(grabPoint.position - targetTransform.position),
                                            Quaternion.Inverse(grabPoint.rotation) * targetTransform.rotation);

            _isGrabbed = true;
            _positionVelocity = Vector3.zero;
            _rotationVelocity = 0f;

            CurveHolder(_rubberAngle);
        }

        public void UpdateTransform()
        {
            Pose grabPoint = _grabbable.GrabPoints[0];
            Transform targetTransform = _grabbable.Transform;

            Vector3 currentPosition = targetTransform.localPosition;
            Vector3 desiredPosition = grabPoint.position - targetTransform.TransformVector(_grabDeltaInLocalSpace.position);
            Quaternion desiredRotation = grabPoint.rotation * _grabDeltaInLocalSpace.rotation;
            Pose desiredLocalPose = PoseUtils.Delta(targetTransform.parent, new Pose(desiredPosition, desiredRotation));

            Vector3 aimVector = (desiredLocalPose.position - _neutralPose.position);

            float tension = Vector3.Distance(currentPosition, _neutralPose.position);
            float desiredTension = aimVector.magnitude;

            if (desiredTension > tension)
            {
                float translationResistance = _translationResistance.Evaluate(tension) * Time.deltaTime;
                desiredTension = Mathf.MoveTowards(tension, desiredTension, translationResistance);
            }

            Vector3 idealAim = Vector3.ProjectOnPlane(aimVector, Vector3.right).normalized;
            aimVector = Vector3.Slerp(aimVector, idealAim, _aimingResistance.Evaluate(tension)).normalized;

            Vector3 targetPosition = _neutralPose.position + aimVector * desiredTension;

            targetTransform.localPosition = targetPosition;

            tension = Tension(targetPosition);
            float rotationResistance = _aimingResistance.Evaluate(tension);
            Quaternion aimingDirection = Quaternion.LookRotation(-aimVector, desiredLocalPose.up);

            targetTransform.localRotation = Quaternion.SlerpUnclamped(desiredLocalPose.rotation, aimingDirection, rotationResistance);

            OnStretch(tension);
        }

        public void EndTransform()
        {
            _isGrabbed = false;
            if (_loadedPellet != null)
            {
                Vector3 force = SlingshotLaunchForce();
                _loadedPellet.Eject(force);
                _loadedPellet = null;
            }
            CurveHolder(0f);
        }

        private void Update()
        {
            if (!_isGrabbed)
            {
                Transform targetTransform = this.transform;

                Vector3 force = (_neutralPose.position - targetTransform.localPosition) * _springForce;
                _positionVelocity = _positionVelocity * _damping + force * Time.deltaTime;
                targetTransform.localPosition += _positionVelocity;

                targetTransform.localRotation.ToAngleAxis(out float angle, out Vector3 axis);
                if (angle > 180)
                {
                    angle -= 360;
                }

                _rotationVelocity = _rotationVelocity * _damping + angle * _springForce * Time.deltaTime;
                targetTransform.localRotation = Quaternion.AngleAxis(_rotationVelocity, -axis.normalized) * targetTransform.localRotation;
            }
        }

        private void LateUpdate()
        {
            if (_loadedPellet != null)
            {
                _loadedPellet.Move(_holder);
            }
        }


        private void CurveHolder(float angle)
        {
            _rightRubberPoint.localEulerAngles = Vector3.up * angle;
            _leftRubberPoint.localEulerAngles = -Vector3.up * angle;
        }


        private float Tension(Vector3 localPoint)
        {
            return Vector3.Distance(localPoint, _neutralPose.position);
        }

        private Vector3 SlingshotLaunchForce()
        {
            Transform targetTransform = _grabbable.Transform;
            float tension = Tension(targetTransform.localPosition);
            Vector3 direction = (targetTransform.parent.position - targetTransform.position).normalized;

            return direction * tension * _slingshotStrength;
        }

        public void OnStretch(float currentTension)
        {
            if (Mathf.Abs(_lastTensionStep - currentTension) > _stretchAudioStep.Evaluate(currentTension)
                && (Time.unscaledTime - _lastTensionTime) > _tensionStepLength)
            {
                PlayStretchAudio(currentTension);
                PlayStretchHaptics(currentTension);
                _lastTensionStep = currentTension;
                _lastTensionTime = Time.unscaledTime;
            }
        }

        private void PlayStretchAudio(float tension)
        {
            float pitch = _strecthAudioPitch.Evaluate(tension);
            _audioSource.pitch = pitch;
            _audioSource.PlayOneShot(_stretchAudioClip, 1f);

        }

        private void PlayStretchHaptics(float tension)
        {
            float pitch = _strecthAudioPitch.Evaluate(tension);
            StartCoroutine(HapticsRoutine(pitch));
        }

        private IEnumerator HapticsRoutine(float pitch)
        {
            OVRInput.Controller controllers = OVRInput.Controller.RTouch | OVRInput.Controller.LTouch;
            OVRInput.SetControllerVibration(pitch * 0.5f, pitch * 0.2f, controllers);
            yield return _hapticsWait;
            OVRInput.SetControllerVibration(0, 0, controllers);
        }
    }
}
