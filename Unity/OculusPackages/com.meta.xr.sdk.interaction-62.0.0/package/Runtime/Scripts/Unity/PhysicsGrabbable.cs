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

using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace Oculus.Interaction
{
    public class PhysicsGrabbable : MonoBehaviour
    {
        [SerializeField, Interface(typeof(IPointable))]
        [FormerlySerializedAs("_grabbable")]
        private UnityEngine.Object _pointable;
        private IPointable Pointable { get; set; }

        [SerializeField]
        private Rigidbody _rigidbody;

        [SerializeField]
        [Tooltip("If enabled, the object's mass will scale appropriately as the scale of the object changes.")]
        private bool _scaleMassWithSize = true;

        private Vector3 _initialScale;
        private bool _hasPendingForce;
        private Vector3 _linearVelocity;
        private Vector3 _angularVelocity;
        private int _selectorsCount = 0;

        protected bool _started = false;

        public event Action<Vector3, Vector3> WhenVelocitiesApplied = delegate { };

        #region Editor
        private void Reset()
        {
            _pointable = this.GetComponent<IPointable>() as UnityEngine.Object;
            _rigidbody = this.GetComponent<Rigidbody>();
        }
        #endregion

        protected virtual void Awake()
        {
            Pointable = _pointable as IPointable;
        }

        protected virtual void Start()
        {
            this.BeginStart(ref _started);
            this.AssertAspect(Pointable, nameof(_pointable));
            this.AssertField(_rigidbody, nameof(_rigidbody));
            this.EndStart(ref _started);
        }

        protected virtual void OnEnable()
        {
            if (_started)
            {
                Pointable.WhenPointerEventRaised += HandlePointerEventRaised;
            }
        }

        protected virtual void OnDisable()
        {
            if (_started)
            {
                Pointable.WhenPointerEventRaised -= HandlePointerEventRaised;

                if (_selectorsCount != 0)
                {
                    _selectorsCount = 0;
                    ReenablePhysics();
                }
            }
        }

        private void HandlePointerEventRaised(PointerEvent evt)
        {
            switch (evt.Type)
            {
                case PointerEventType.Select:
                    AddSelection();
                    break;
                case PointerEventType.Cancel:
                case PointerEventType.Unselect:
                    RemoveSelection();
                    break;
            }
        }

        private void AddSelection()
        {
            if (_selectorsCount++ == 0)
            {
                DisablePhysics();
            }
        }

        private void RemoveSelection()
        {
            if (--_selectorsCount == 0)
            {
                ReenablePhysics();
            }
            _selectorsCount = Mathf.Max(0, _selectorsCount);
        }

        private void DisablePhysics()
        {
            CachePhysicsState();
            _rigidbody.LockKinematic();
        }

        private void ReenablePhysics()
        {
            // update the mass based on the scale change
            if (_scaleMassWithSize)
            {
                float initialScaledVolume = _initialScale.x * _initialScale.y * _initialScale.z;

                Vector3 currentScale = _rigidbody.transform.localScale;
                float currentScaledVolume = currentScale.x * currentScale.y * currentScale.z;

                float changeInMassFactor = currentScaledVolume / initialScaledVolume;
                _rigidbody.mass *= changeInMassFactor;
            }

            // revert the original kinematic state
            _rigidbody.UnlockKinematic();
        }

        public void ApplyVelocities(Vector3 linearVelocity, Vector3 angularVelocity)
        {
            _hasPendingForce = true;
            _linearVelocity = linearVelocity;
            _angularVelocity = angularVelocity;
        }

        private void FixedUpdate()
        {
            if (_hasPendingForce)
            {
                _hasPendingForce = false;
                _rigidbody.AddForce(_linearVelocity, ForceMode.VelocityChange);
                _rigidbody.AddTorque(_angularVelocity, ForceMode.VelocityChange);
                WhenVelocitiesApplied(_linearVelocity, _angularVelocity);
            }
        }

        private void CachePhysicsState()
        {
            _initialScale = _rigidbody.transform.localScale;
        }

        #region Inject

        public void InjectAllPhysicsGrabbable(IPointable pointable, Rigidbody rigidbody)
        {
            InjectPointable(pointable);
            InjectRigidbody(rigidbody);
        }

        [Obsolete("Use " + nameof(InjectAllPhysicsGrabbable) + " with " + nameof(IPointable) + " instead")]
        public void InjectAllPhysicsGrabbable(Grabbable grabbable, Rigidbody rigidbody)
        {
            InjectPointable(grabbable);
            InjectRigidbody(rigidbody);
        }

        [Obsolete("Use " + nameof(InjectPointable) + " instead")]
        public void InjectGrabbable(Grabbable grabbable)
        {
            InjectPointable(grabbable);
        }

        public void InjectPointable(IPointable pointable)
        {
            _pointable = pointable as UnityEngine.Object;
            Pointable = pointable;
        }

        public void InjectRigidbody(Rigidbody rigidbody)
        {
            _rigidbody = rigidbody;
        }

        public void InjectOptionalScaleMassWithSize(bool scaleMassWithSize)
        {
            _scaleMassWithSize = scaleMassWithSize;
        }

        #endregion
    }
}
