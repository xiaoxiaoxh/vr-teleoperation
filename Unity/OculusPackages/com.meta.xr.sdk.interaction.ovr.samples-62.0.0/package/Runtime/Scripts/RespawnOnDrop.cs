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
using UnityEngine.Events;

namespace Oculus.Interaction.Samples
{
    public class RespawnOnDrop : MonoBehaviour
    {
        /// <summary>
        /// Respawn will happen when the transform moves below this World Y position.
        /// </summary>
        [SerializeField]
        [Tooltip("Respawn will happen when the transform moves below this World Y position.")]
        private float _yThresholdForRespawn;

        /// <summary>
        /// UnityEvent triggered when a respawn occurs.
        /// </summary>
        [SerializeField]
        [Tooltip("UnityEvent triggered when a respawn occurs.")]
        private UnityEvent _whenRespawned = new UnityEvent();

        /// <summary>
        /// If the transform has an associated rigidbody, make it kinematic during this
        /// number of frames after a respawn, in order to avoid ghost collisions.
        /// </summary>
        [SerializeField]
        [Tooltip("If the transform has an associated rigidbody, make it kinematic during this number of frames after a respawn, in order to avoid ghost collisions.")]
        private int _sleepFrames = 0;

        public UnityEvent WhenRespawned => _whenRespawned;

        // cached starting transform
        private Vector3 _initialPosition;
        private Quaternion _initialRotation;
        private Vector3 _initialScale;

        private TwoGrabFreeTransformer[] _freeTransformers;
        private Rigidbody _rigidBody;
        private int _sleepCountDown;

        protected virtual void OnEnable()
        {
            _initialPosition = transform.position;
            _initialRotation = transform.rotation;
            _initialScale = transform.localScale;
            _freeTransformers = GetComponents<TwoGrabFreeTransformer>();
            _rigidBody = GetComponent<Rigidbody>();
        }

        protected virtual void Update()
        {
            if (transform.position.y < _yThresholdForRespawn)
            {
                Respawn();
            }
        }

        protected virtual void FixedUpdate()
        {
            if (_sleepCountDown > 0)
            {
                if (--_sleepCountDown == 0)
                {
                    _rigidBody.isKinematic = false;
                }
            }
        }

        public void Respawn()
        {
            transform.position = _initialPosition;
            transform.rotation = _initialRotation;
            transform.localScale = _initialScale;

            if (_rigidBody)
            {
                _rigidBody.velocity = Vector3.zero;
                _rigidBody.angularVelocity = Vector3.zero;

                if (!_rigidBody.isKinematic && _sleepFrames > 0)
                {
                    _sleepCountDown = _sleepFrames;
                    _rigidBody.isKinematic = true;
                }
            }

            foreach (var freeTransformer in _freeTransformers)
            {
                freeTransformer.MarkAsBaseScale();
            }

            _whenRespawned.Invoke();
        }
    }
}
