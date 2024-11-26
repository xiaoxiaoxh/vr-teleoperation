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

namespace Oculus.Interaction.Throw
{
    public class RANSACVelocityCalculator : MonoBehaviour,
        IThrowVelocityCalculator, ITimeConsumer
    {
        [SerializeField, Interface(typeof(IPoseInputDevice))]
        private UnityEngine.Object _poseInputDevice;
        public IPoseInputDevice PoseInputDevice { get; private set; }

        private Func<float> _timeProvider = () => Time.time;
        public void SetTimeProvider(Func<float> timeProvider)
        {
            _timeProvider = timeProvider;
        }

        private struct TimedPose
        {
            public float time;
            public Pose pose;

            public TimedPose(float time, Pose pose)
            {
                this.time = time;
                this.pose = pose;
            }
        }

        private RingBuffer<TimedPose> _poses;

        private int _consecutiveValidFrames;
        private float _previousPositionId;
        private float _lastTime;

        private const int SAMPLES_COUNT = 8;
        private const int SAMPLES_DEAD_ZONE = 2;
        private const int MIN_HIGHCONFIDENCE_SAMPLES = 2;

        private delegate Vector3 RANSACSampler(Pose offset, int idx1, int idx2);
        private delegate float RANSACScorer(Vector3 sample, Vector3[,] samplesTable);

        private Vector3[,] _samplesTable = new Vector3[SAMPLES_COUNT, SAMPLES_COUNT];

        protected virtual void Awake()
        {
            PoseInputDevice = _poseInputDevice as IPoseInputDevice;
        }

        protected virtual void Start()
        {
            this.AssertField(PoseInputDevice, nameof(PoseInputDevice));

            int capacity = SAMPLES_COUNT + SAMPLES_DEAD_ZONE;
            _poses = new RingBuffer<TimedPose>(capacity, new TimedPose(_timeProvider(), Pose.identity));
        }

        private void Update()
        {
            ProcessInput();
        }

        public ReleaseVelocityInformation CalculateThrowVelocity(Transform objectThrown)
        {
            ProcessInput();

            return GetThrowInformation(objectThrown.GetPose());
        }


        private void ProcessInput()
        {
            float time = _timeProvider();
            if (_poses.Peek().time == time)
            {
                return;
            }

            if (!PoseInputDevice.IsInputValid
                || !PoseInputDevice.IsHighConfidence
                || !PoseInputDevice.GetRootPose(out Pose rootPose)
                || rootPose.position.sqrMagnitude == _previousPositionId)
            {
                _consecutiveValidFrames = 0;

            }
            else
            {
                if (_consecutiveValidFrames == 0)
                {
                    _poses.Add(RepeatLast(_lastTime));
                }
                _consecutiveValidFrames++;
                _previousPositionId = rootPose.position.sqrMagnitude;
                _poses.Add(new TimedPose(time, rootPose));
            }

            _lastTime = time;
        }

        private ReleaseVelocityInformation GetThrowInformation(Pose grabPoint)
        {
            PoseInputDevice.GetRootPose(out Pose rootPose);
            Pose offset = PoseUtils.Delta(rootPose, grabPoint);

            Vector3 position = grabPoint.position;
            Vector3 velocity;
            Vector3 torque;

            if (_consecutiveValidFrames >= MIN_HIGHCONFIDENCE_SAMPLES)
            {
                velocity = RANSAC(offset, CalculateVelocityFromSamples, ScoreDistance);
                torque = RANSAC(offset, CalculateTorqueFromSamples, ScoreTorque);
            }
            else
            {
                GetLastPoseVelocity(offset, out velocity, out torque);
            }

            return new ReleaseVelocityInformation(velocity, torque, position, true);
        }

        private TimedPose RepeatLast(float time)
        {
            TimedPose lastPose = _poses.Peek();
            lastPose.time = time;

            return lastPose;
        }

        private void GetLastPoseVelocity(Pose offset, out Vector3 velocity, out Vector3 torque)
        {
            TimedPose younger = _poses.Peek(0);
            TimedPose older = _poses.Peek(-1);

            float timeShift = younger.time - older.time;
            velocity = (PoseUtils.Multiply(younger.pose, offset).position - PoseUtils.Multiply(older.pose, offset).position) / timeShift;

            torque = GetAngularVelocity(older, younger);
        }

        private Vector3 RANSAC(Pose offset, RANSACSampler sampler, RANSACScorer scorer)
        {
            for (int i = 0; i < SAMPLES_COUNT; ++i)
            {
                for (int j = i + 1; j < SAMPLES_COUNT; ++j)
                {
                    _samplesTable[i, j] = sampler(offset, i + SAMPLES_DEAD_ZONE, j + SAMPLES_DEAD_ZONE);
                }
            }

            Vector3 bestSample = Vector3.zero;
            float bestScore = float.PositiveInfinity;
            for (int i = 0; i < SAMPLES_COUNT; ++i)
            {
                int y = Mathf.FloorToInt(UnityEngine.Random.value * (SAMPLES_COUNT - 1));
                int x = y + Mathf.FloorToInt(UnityEngine.Random.value * (SAMPLES_COUNT - y - 1)) + 1;

                Vector3 sample = _samplesTable[y, x];
                float score = scorer(sample, _samplesTable);

                if (score < bestScore)
                {
                    bestSample = sample;
                    bestScore = score;
                }
            }

            return bestSample;
        }

        private Vector3 CalculateVelocityFromSamples(Pose offset, int idx1, int idx2)
        {
            GetSortedTimePoses(idx1, idx2, out TimedPose older, out TimedPose younger);
            float timeShift = younger.time - older.time;
            Vector3 positionShift = PoseUtils.Multiply(younger.pose, offset).position - PoseUtils.Multiply(older.pose, offset).position;
            return positionShift / timeShift;
        }

        private Vector3 CalculateTorqueFromSamples(Pose offset, int idx1, int idx2)
        {
            GetSortedTimePoses(idx1, idx2, out TimedPose older, out TimedPose younger);
            Vector3 torque = GetAngularVelocity(older, younger);
            return torque;
        }

        private void GetSortedTimePoses(int idx1, int idx2,
            out TimedPose older, out TimedPose younger)
        {
            int youngerIdx = idx1;
            int olderIdx = idx2;
            if (idx2 > idx1)
            {
                youngerIdx = idx2;
                olderIdx = idx1;
            }

            older = _poses[olderIdx];
            younger = _poses[youngerIdx];
        }

        private static float ScoreDistance(Vector3 sample, Vector3[,] samplesTable)
        {
            float score = 0f;
            for (int y = 0; y < SAMPLES_COUNT; ++y)
            {
                for (int x = y + 1; x < SAMPLES_COUNT; ++x)
                {
                    score += (sample - samplesTable[y, x]).sqrMagnitude;
                }
            }
            return score;
        }

        private static float ScoreTorque(Vector3 sample, Vector3[,] samplesTable)
        {
            float score = 0f;
            for (int y = 0; y < SAMPLES_COUNT; ++y)
            {
                for (int x = y + 1; x < SAMPLES_COUNT; ++x)
                {
                    score += Mathf.Abs(Quaternion.Dot(
                        Quaternion.Euler(sample),
                        Quaternion.Euler(samplesTable[y, x])));
                }
            }
            return score;
        }

        private static Vector3 GetAngularVelocity(TimedPose older, TimedPose younger)
        {
            float timeShift = younger.time - older.time;
            Quaternion deltaRotation = younger.pose.rotation * Quaternion.Inverse(older.pose.rotation);
            deltaRotation.ToAngleAxis(out float angularSpeed, out Vector3 torqueAxis);
            angularSpeed = (angularSpeed * Mathf.Deg2Rad) / timeShift;

            return torqueAxis * angularSpeed;
        }

        private class RingBuffer<T>
        {
            private T[] _buffer;
            private int _head;
            private int _count;

            public RingBuffer(int capacity, T defaultValue)
            {
                _buffer = new T[capacity];
                _count = capacity;
                _head = 0;

                for (int i = 0; i < capacity; i++)
                {
                    _buffer[i] = defaultValue;
                }
            }

            public void Add(T item)
            {
                _buffer[_head] = item;
                _head = (_head + 1) % _count;
            }

            public T this[int index] => _buffer[index % _count];
            public T Peek(int offset = 0) => _buffer[(_head + offset + _count) % _count];
        }

        #region Inject

        public void InjectAllRANSACVelocityCalculator(IPoseInputDevice poseInputDevice)
        {
            InjectPoseInputDevice(poseInputDevice);
        }

        public void InjectPoseInputDevice(IPoseInputDevice poseInputDevice)
        {
            PoseInputDevice = poseInputDevice;
            _poseInputDevice = poseInputDevice as UnityEngine.Object;
        }

        #endregion

    }
}
