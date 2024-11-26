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

public class AnimatorOverrideLayerWeigth : MonoBehaviour
{
    public Animator animator;
    public string overrideLayer;

    public float transitionDuration;
    public AnimationCurve transitionCurve;

    public void SetOverrideLayerActive(bool active)
    {
        var layerIndex = animator.GetLayerIndex(overrideLayer);

        if (transitionDuration > 0.0)
        {
            StopAllCoroutines();
            StartCoroutine(LayerTransition(layerIndex, active ? 1.0f : 0.0f));
        }
        else
        {
            animator.SetLayerWeight(layerIndex, active ? 1.0f : 0.0f);
        }
    }

    IEnumerator LayerTransition(int layerIndex, float targetWeight)
    {
        float startTime = Time.time;
        float startWeight = animator.GetLayerWeight(layerIndex);
        while (true)
        {
            float transitionTime = (Time.time - startTime) / transitionDuration;
            float targetWeightParam = transitionCurve.Evaluate(Mathf.Clamp01(transitionTime));
            float weigth = Mathf.Lerp(startWeight, targetWeight, targetWeightParam);
            animator.SetLayerWeight(layerIndex, weigth);

            if (transitionTime >= 1.0)
            {
                yield break;
            }

            yield return null;
        }
    }
}
