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
using System.Collections.Generic;
using UnityEngine;

namespace Oculus.Interaction.Input
{
    public class HandPhysicsCapsules : MonoBehaviour
    {
        [SerializeField, Interface(typeof(IHandVisual))]
        [Obsolete("Replaced by _hand")]
        private UnityEngine.Object _handVisual;
        [Obsolete("Replaced by Hand")]
        private IHandVisual HandVisual;

        [SerializeField, Interface(typeof(IHand))]
        private UnityEngine.Object _hand;
        private IHand Hand;

        [SerializeField]
        private JointsRadiusFeature _jointsRadiusFeature;

        [Space]
        [SerializeField]
        [Tooltip("Capsules will be generated as triggers")]
        private bool _asTriggers = false;
        [SerializeField]
        [Tooltip("Capsules will be generated in this Layer")]
        private int _useLayer = 0;
        [SerializeField]
        [Tooltip("Capsules reaching this joint will not be generated")]
        private HandFingerJointFlags _mask = HandFingerJointFlags.All;

        private Action _whenCapsulesGenerated = delegate { };
        public event Action WhenCapsulesGenerated
        {
            add
            {
                _whenCapsulesGenerated += value;
                if (_capsulesGenerated)
                {
                    value.Invoke();
                }
            }
            remove
            {
                _whenCapsulesGenerated -= value;
            }
        }

        private Transform _rootTransform;
        public Transform RootTransform => _rootTransform;

        private List<BoneCapsule> _capsules;
        public IList<BoneCapsule> Capsules { get; private set; }

        private Rigidbody[] _rigidbodies;
        private bool _capsulesAreActive;
        private bool _capsulesGenerated;

        protected bool _started;

        #region Editor events
        protected virtual void Reset()
        {
            _useLayer = this.gameObject.layer;
        }
        #endregion

        protected virtual void Awake()
        {
            HandVisual = _handVisual as IHandVisual;
            Hand = _hand as IHand;
        }

        protected virtual void Start()
        {
            this.BeginStart(ref _started);
            if (Hand == null && HandVisual != null)
            {
                Hand = HandVisual.Hand;
            }
            this.AssertField(Hand, nameof(Hand));
            this.AssertField(_jointsRadiusFeature, nameof(_jointsRadiusFeature));
            this.EndStart(ref _started);
        }

        protected virtual void OnEnable()
        {
            if (_started)
            {
                _capsulesAreActive = true;
                Hand.WhenHandUpdated += HandleHandUpdated;
            }
        }

        protected virtual void OnDisable()
        {
            if (_started)
            {
                Hand.WhenHandUpdated -= HandleHandUpdated;
                DisableRigidbodies();
            }
        }


        private void GenerateCapsules()
        {
            if (!Hand.IsTrackedDataValid)
            {
                return;
            }
            _rigidbodies = new Rigidbody[(int)HandJointId.HandMaxSkinnable];

            Transform _holder = new GameObject("Capsules").transform;
            _holder.SetParent(transform, false);
            _holder.localPosition = Vector3.zero;
            _holder.localRotation = Quaternion.identity;
            _holder.gameObject.layer = _useLayer;

            int capsulesCount = Constants.NUM_HAND_JOINTS;
            _capsules = new List<BoneCapsule>(capsulesCount);
            Capsules = _capsules.AsReadOnly();
            for (int i = (int)HandJointId.HandThumb0; i < (int)HandJointId.HandEnd; ++i)
            {
                HandJointId currentJoint = (HandJointId)i;
                HandJointId parentJoint = HandJointUtils.JointParentList[i];
                if (parentJoint == HandJointId.Invalid
                    || ((1 << (int)currentJoint) & (int)_mask) == 0)
                {
                    continue;
                }

                Hand.GetJointPose(parentJoint, out Pose parentPose);
                if (!TryGetJointRigidbody(parentJoint, out Rigidbody body))
                {
                    body = CreateJointRigidbody(parentJoint, _holder, parentPose);
                    _rigidbodies[(int)parentJoint] = body;
                }

                string boneName = $"{parentJoint}-{currentJoint} CapsuleCollider";
                float boneRadius = _jointsRadiusFeature.GetJointRadius(parentJoint);
                float offset = currentJoint >= HandJointId.HandMaxSkinnable ? -boneRadius
                    : parentJoint == HandJointId.HandStart ? boneRadius
                    : 0f;

                Hand.GetJointPose(currentJoint, out Pose jointPose);

                CapsuleCollider collider = CreateCollider(boneName,
                    body.transform, parentPose.position, jointPose.position, boneRadius, offset);

                BoneCapsule capsule = new BoneCapsule(parentJoint, currentJoint, body, collider);
                _capsules.Add(capsule);

            }

            IgnoreSelfCollisions();
            _capsulesGenerated = true;
            _whenCapsulesGenerated.Invoke();
        }

        private void IgnoreSelfCollisions()
        {
            for (int i = 0; i < _capsules.Count; i++)
            {
                for (int j = i + 1; j < _capsules.Count; j++)
                {
                    Physics.IgnoreCollision(_capsules[i].CapsuleCollider, _capsules[j].CapsuleCollider);
                }
            }
        }

        private bool TryGetJointRigidbody(HandJointId joint, out Rigidbody body)
        {
            int index = (int)joint;
            if (_rigidbodies == null
                || index < 0
                || index >= _rigidbodies.Length)
            {
                body = null;
                return false;
            }
            body = _rigidbodies[index];
            return body != null;
        }

        private Rigidbody CreateJointRigidbody(HandJointId joint,
            Transform holder, Pose pose)
        {
            string name = $"{joint} Rigidbody";
            Rigidbody rigidbody = new GameObject(name)
                .AddComponent<Rigidbody>();
            rigidbody.mass = 1.0f;
            rigidbody.isKinematic = true;
            rigidbody.useGravity = false;
            rigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;

            rigidbody.transform.SetParent(holder, false);
            rigidbody.transform.SetPose(pose);
            rigidbody.Sleep();
            rigidbody.gameObject.SetActive(false);
            rigidbody.gameObject.layer = _useLayer;

            return rigidbody;
        }

        private CapsuleCollider CreateCollider(string name,
            Transform holder, Vector3 from, Vector3 to, float radius, float offset)
        {
            CapsuleCollider collider = new GameObject(name)
                .AddComponent<CapsuleCollider>();
            collider.isTrigger = _asTriggers;

            Vector3 boneDirection = to - from;
            Quaternion boneRotation = Quaternion.LookRotation(boneDirection);
            float boneLength = boneDirection.magnitude - Mathf.Abs(offset);

            collider.radius = radius;
            collider.height = boneLength + radius * 2.0f;
            collider.direction = 2;
            collider.center = Vector3.forward * (boneLength * 0.5f + Mathf.Max(0f, offset));

            Transform capsuleTransform = collider.transform;
            capsuleTransform.SetParent(holder, false);
            capsuleTransform.SetPositionAndRotation(from, boneRotation);
            collider.gameObject.layer = _useLayer;

            return collider;
        }

        private void DisableRigidbodies()
        {
            if (!_capsulesAreActive)
            {
                return;
            }

            for (HandJointId jointId = HandJointId.HandStart; jointId < HandJointId.HandMaxSkinnable; ++jointId)
            {
                if (!TryGetJointRigidbody(jointId, out Rigidbody jointbody))
                {
                    continue;
                }

                jointbody.Sleep();
                jointbody.gameObject.SetActive(false);
            }
            _capsulesAreActive = false;
        }

        private void HandleHandUpdated()
        {
            if (!_capsulesGenerated)
            {
                GenerateCapsules();
            }

            if (_capsulesGenerated)
            {
                UpdateRigidbodies();
            }

        }

        private void UpdateRigidbodies()
        {
            for (HandJointId jointId = HandJointId.HandStart; jointId < HandJointId.HandMaxSkinnable; ++jointId)
            {
                if (!TryGetJointRigidbody(jointId, out Rigidbody jointbody))
                {
                    continue;
                }

                GameObject jointGO = jointbody.gameObject;
                if (_capsulesAreActive
                    && Hand.GetJointPose(jointId, out Pose bonePose))
                {
                    jointbody.MovePosition(bonePose.position);
                    jointbody.MoveRotation(bonePose.rotation);

                    if (!jointGO.activeSelf)
                    {
                        jointGO.SetActive(true);
                        jointbody.WakeUp();
                    }
                }
                else if (jointGO.activeSelf)
                {
                    jointbody.Sleep();
                    jointGO.SetActive(false);
                }
            }
        }

        #region Inject

        public void InjectAllOVRHandPhysicsCapsules(IHand hand,
            bool asTriggers, int useLayer)
        {
            InjectHand(hand);
            InjectAsTriggers(asTriggers);
            InjectUseLayer(useLayer);
        }

        public void InjectHand(IHand hand)
        {
            _hand = hand as UnityEngine.Object;
            Hand = hand;
        }

        public void InjectAsTriggers(bool asTriggers)
        {
            _asTriggers = asTriggers;
        }
        public void InjectUseLayer(int useLayer)
        {
            _useLayer = useLayer;
        }
        
        public void InjectMask(HandFingerJointFlags mask)
        {
            _mask = mask;
        }
        
        public void InjectJointsRadiusFeature(JointsRadiusFeature jointsRadiusFeature)
        {
            _jointsRadiusFeature = jointsRadiusFeature;
        }

        #endregion
    }

    public class BoneCapsule
    {
        public HandJointId StartJoint { get; private set; }
        public HandJointId EndJoint { get; private set; }
        public Rigidbody CapsuleRigidbody { get; private set; }
        public CapsuleCollider CapsuleCollider { get; private set; }

        public BoneCapsule(HandJointId fromJoint, HandJointId toJoint, Rigidbody body, CapsuleCollider collider)
        {
            StartJoint = fromJoint;
            EndJoint = toJoint;
            CapsuleRigidbody = body;
            CapsuleCollider = collider;
        }
    }
}
