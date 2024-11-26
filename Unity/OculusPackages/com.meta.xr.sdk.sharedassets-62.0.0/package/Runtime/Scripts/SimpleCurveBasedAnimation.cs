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

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Meta.XR.SharedAssets
{
    public class SimpleCurveBasedAnimation : MonoBehaviour
    {
        [SerializeField] AnimationCurve _positionX = new AnimationCurve(new Keyframe(0, 0), new Keyframe(1, 0));
        [SerializeField] AnimationCurve _positionY = new AnimationCurve(new Keyframe(0, 0), new Keyframe(1, 0));
        [SerializeField] AnimationCurve _positionZ = new AnimationCurve(new Keyframe(0, 0), new Keyframe(1, 0));
        [SerializeField] AnimationCurve _rotationX = new AnimationCurve(new Keyframe(0, 0), new Keyframe(1, 0));
        [SerializeField] AnimationCurve _rotationY = new AnimationCurve(new Keyframe(0, 0), new Keyframe(1, 0));
        [SerializeField] AnimationCurve _rotationZ = new AnimationCurve(new Keyframe(0, 0), new Keyframe(1, 0));
        [SerializeField] AnimationCurve _scaleX = new AnimationCurve(new Keyframe(0, 1), new Keyframe(1, 1));
        [SerializeField] AnimationCurve _scaleY = new AnimationCurve(new Keyframe(0, 1), new Keyframe(1, 1));
        [SerializeField] AnimationCurve _scaleZ = new AnimationCurve(new Keyframe(0, 1), new Keyframe(1, 1));
        [SerializeField] bool _playOnStart = false;
        [SerializeField] bool _loop = false;
        [Tooltip("If enabled the object will start with the first frame of the animation baked in its transform.")]
        [SerializeField] bool _bakeInitialState = false;
        float _time;
        AnimationCurve[] _allAnimCurves;
        Vector3 startingLocalPosition;
        Vector3 startingLocalEuler;
        Vector3 startingLocalScale;
        // Start is called before the first frame update
        void Start()
        {
            _allAnimCurves = new AnimationCurve[] { _positionX, _positionY, _positionZ, _rotationX, _rotationY, _rotationZ, _scaleX, _scaleY, _scaleZ };
            startingLocalPosition = this.transform.position;
            startingLocalEuler = this.transform.eulerAngles;
            startingLocalScale = this.transform.localScale;
            if (_playOnStart)
            {
                StartCoroutine(Play());
            }
            if (_bakeInitialState)
            {
                this.transform.Translate(new Vector3(_positionX.Evaluate(0), _positionY.Evaluate(0), _positionZ.Evaluate(0)), Space.Self);
                this.transform.Rotate(new Vector3(_rotationX.Evaluate(0), _rotationY.Evaluate(0), _rotationZ.Evaluate(0)), Space.Self);
                this.transform.localScale = new Vector3(_scaleX.Evaluate(0) * startingLocalScale.x, _scaleY.Evaluate(0) * startingLocalScale.y, _scaleZ.Evaluate(0) * startingLocalScale.z);
            }
        }

        [ContextMenu("StartAnimation")]
        public void StartAnimation()
        {
            StartCoroutine(Play());
        }
        [ContextMenu("StartAnimationReversed")]
        public void ReverseAnimation()
        {
            StartCoroutine(PlayReverse());
        }

        IEnumerator Play()
        {
            startingLocalPosition = this.transform.localPosition;
            startingLocalEuler = this.transform.localEulerAngles;
            startingLocalScale = this.transform.localScale;
            AnimationCurve longestAnim = _positionX;
            foreach (var c in _allAnimCurves)
            {
                if (c.keys[c.keys.Length - 1].time > longestAnim.keys[longestAnim.keys.Length - 1].time)
                {
                    longestAnim = c;
                }
            }
            float step = 0;
            if (!_loop)
            {
                while (step < longestAnim.keys[longestAnim.keys.Length - 1].time)
                {
                    UpdateTransform(step);
                    step += Time.deltaTime;
                    yield return null;
                }
                GoToFinalState();
            }
            else
            {
                foreach (var c in _allAnimCurves)
                {
                    if (c.postWrapMode != WrapMode.Loop)
                    {
                        c.postWrapMode = WrapMode.Loop;
                        c.preWrapMode = WrapMode.Loop;
                    }
                }
                while (true)
                {
                    UpdateTransform(step);
                    step += Time.deltaTime;
                    yield return null;
                }
            }
        }
        IEnumerator PlayReverse()
        {
            AnimationCurve longestAnim = FindLongestAnimCurve();
            float step = longestAnim.keys[longestAnim.keys.Length - 1].time;
            if (!_loop)
            {
                while (step > 0)
                {
                    UpdateTransform(step);
                    step -= Time.deltaTime;
                    yield return null;
                }
                GoToInitialState();
            }
            else
            {
                foreach (var c in _allAnimCurves)
                {
                    if (c.postWrapMode != WrapMode.Loop)
                    {
                        c.postWrapMode = WrapMode.Loop;
                        c.preWrapMode = WrapMode.Loop;
                    }
                }
                while (true)
                {
                    UpdateTransform(step);
                    print(step);
                    step -= Time.deltaTime;
                    yield return null;
                }
            }
        }

        void UpdateTransform(float step)
        {
            this.transform.localPosition = startingLocalPosition;
            this.transform.Translate(new Vector3(_positionX.Evaluate(step), _positionY.Evaluate(step), _positionZ.Evaluate(step)), Space.Self);

            this.transform.localEulerAngles = startingLocalEuler;
            this.transform.Rotate(new Vector3(_rotationX.Evaluate(step), _rotationY.Evaluate(step), _rotationZ.Evaluate(step)), Space.Self);

            this.transform.localScale = new Vector3(_scaleX.Evaluate(step) * startingLocalScale.x, _scaleY.Evaluate(step) * startingLocalScale.y, _scaleZ.Evaluate(step) * startingLocalScale.z);
        }
        public void StopAndReset()
        {
            print("Stop and reset");
            StopAllCoroutines();
            GoToInitialState();
        }

        AnimationCurve FindLongestAnimCurve()
        {
            AnimationCurve longestAnim = _positionX;
            foreach (var c in _allAnimCurves)
            {
                if (c.keys[c.keys.Length - 1].time > longestAnim.keys[longestAnim.keys.Length - 1].time)
                {
                    longestAnim = c;
                }
            }
            return longestAnim;
        }

        void GoToInitialState()
        {
            if (_bakeInitialState)
            {
                this.transform.Translate(new Vector3(_positionX.Evaluate(0), _positionY.Evaluate(0), _positionZ.Evaluate(0)), Space.Self);
                this.transform.Rotate(new Vector3(_rotationX.Evaluate(0), _rotationY.Evaluate(0), _rotationZ.Evaluate(0)), Space.Self);
                this.transform.localScale = new Vector3(_scaleX.Evaluate(0) * startingLocalScale.x, _scaleY.Evaluate(0) * startingLocalScale.y, _scaleZ.Evaluate(0) * startingLocalScale.z);
            }
            else
            {
                this.transform.localPosition = startingLocalPosition;
                this.transform.localEulerAngles = startingLocalEuler;
                this.transform.localScale = startingLocalScale;
            }
        }

        void GoToFinalState()
        {
            AnimationCurve longestAnimCurve = FindLongestAnimCurve();
            float lastKeyFrame = longestAnimCurve.keys[longestAnimCurve.keys.Length - 1].time;
            this.transform.localPosition = startingLocalPosition + new Vector3(_positionX.Evaluate(lastKeyFrame), _positionY.Evaluate(lastKeyFrame), _positionZ.Evaluate(lastKeyFrame));
            this.transform.localEulerAngles = startingLocalEuler + new Vector3(_rotationX.Evaluate(lastKeyFrame), _rotationY.Evaluate(lastKeyFrame), _rotationZ.Evaluate(lastKeyFrame));
            this.transform.localScale = new Vector3(_scaleX.Evaluate(lastKeyFrame) * startingLocalScale.x, _scaleY.Evaluate(lastKeyFrame) * startingLocalScale.y, _scaleZ.Evaluate(lastKeyFrame) * startingLocalScale.z);
        }
    }
}
