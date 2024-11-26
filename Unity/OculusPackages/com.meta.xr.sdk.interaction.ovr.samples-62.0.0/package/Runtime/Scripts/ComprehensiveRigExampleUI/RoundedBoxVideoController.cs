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

using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class RoundedBoxVideoController : MonoBehaviour
{
    public UnityEngine.UI.Slider timeSlider;
    public float animationDuration;
    public float animationTime;
    public int cycleCount;

    public Sprite playIcon;
    public Sprite pauseIcon;
    public Image playPauseImg;

    public bool isPlaying;
    public List<Color> boxColors;
    public List<RectTransform> boxes;

    private float animationCycleDuration;

    [Header("Time Labels")]
    public TextMeshProUGUI leftLabel;
    public TextMeshProUGUI rightLabel;

    [Header("Background Material Settings")]
    public Image backgroundImage;
    public Vector2 direction;
    public Color colorA;
    public Color colorB;

    private readonly int columnDirectionID = Shader.PropertyToID("columnDirection");
    private readonly int rowDirectionID = Shader.PropertyToID("rowDirection");
    private readonly int animationTimeID = Shader.PropertyToID("animationTime");
    private readonly int colorAID = Shader.PropertyToID("colorA");
    private readonly int colorBID = Shader.PropertyToID("colorB");

    private struct BoxAnimation
    {
        public RectTransform rectTransform;
        public Image image;
        public float duration;
        public float startHeight;
        public float animationMaxHeight;
        public float startVelocity;
        public float startTime;
        public float acceleration;

        public void Update(float animationTime)
        {
            var animTime = animationTime - startTime;
            animTime = Mathf.Clamp(animTime, 0.0f, duration);
            var animTime2 = animTime * animTime;

            var parabolicHeight = (startVelocity * animTime) - (0.5f * acceleration * animTime2);
            var heightParam = parabolicHeight / animationMaxHeight;
            var currentHeight = startHeight - parabolicHeight;
            var position = rectTransform.anchoredPosition;
            position.y = currentHeight;
            rectTransform.anchoredPosition = position;

            rectTransform.rotation = Quaternion.Euler(0.0f, 0.0f, heightParam * 360.0f);
        }

        public void SetColor(Color color)
        {
            image.color = color;
        }
    }

    private List<BoxAnimation> animations;

    private void OnEnable()
    {
        UpdateBackgroundMaterialProperties();
    }

    private void Start()
    {
        animations = new List<BoxAnimation>();
        var viewRect = (RectTransform) transform;
        var size = viewRect.rect.size;
        var sections = size.x / ((float)boxes.Count);

        timeSlider.onValueChanged.AddListener(delegate { OnSliderValueChange(); });

        var boxCountMinusOne = (float)boxes.Count - 1.0f;
        var boxMultiplier = boxCountMinusOne * 0.35f + 1.0f;
        animationCycleDuration = animationDuration / ((float)cycleCount);
        var boxAnimationDuration = animationCycleDuration / boxMultiplier;

        var boxStartingHeight = boxes[0].rect.height * 0.6f;
        var boxAnimationHeight = (size.y * 0.5f) + boxStartingHeight;

        var boxStartVelocity = (2.0f * boxAnimationHeight) / (boxAnimationDuration * 0.5f);
        var boxAcceleration = boxStartVelocity / (boxAnimationDuration * 0.5f);

        for (int i = 0; i < boxes.Count; i++)
        {
            var box = boxes[i];
            var mult = (float)i + 0.5f;
            box.anchoredPosition = new Vector2(mult * sections, 0.0f);

            var anim = new BoxAnimation()
            {
                duration = boxAnimationDuration,
                startHeight = boxStartingHeight,
                animationMaxHeight = boxAnimationHeight,
                rectTransform = box,
                startVelocity = boxStartVelocity,
                acceleration = boxAcceleration,
                startTime = boxAnimationDuration * 0.35f * (float)i,
                image = box.GetComponent<Image>(),
            };

            animations.Add(anim);
        }

        SetPlay();
        UpdateBackgroundMaterialProperties();
    }

    public void UpdateBackgroundMaterialProperties()
    {
        var normalizedDirection = direction.normalized;
        backgroundImage.materialForRendering.SetVector(columnDirectionID, normalizedDirection);
        backgroundImage.materialForRendering.SetVector(rowDirectionID, new Vector2(-normalizedDirection.y, normalizedDirection.x));

        backgroundImage.materialForRendering.SetColor(colorAID, colorA.linear);
        backgroundImage.materialForRendering.SetColor(colorBID, colorB.linear);

        backgroundImage.materialForRendering.SetFloat(animationTimeID, animationTime);
    }

    public void OnSliderValueChange()
    {
        animationTime = timeSlider.value * animationDuration;
    }

    public void TogglePlayPause()
    {
        if (isPlaying)
        {
            SetPaused();
        }
        else
        {
            if(Mathf.Abs(animationDuration - animationTime) < 0.1f)
            {
                animationTime = 0.0f;
            }
            SetPlay();
        }
    }

    private void SetPaused()
    {
        isPlaying = false;
        playPauseImg.sprite = playIcon;
    }

    private void SetPlay()
    {
        isPlaying = true;
        playPauseImg.sprite = pauseIcon;
    }

    private string FormatTime(float seconds)
    {
        var mins = seconds / 60.0f;
        var secs = (mins - Mathf.Floor(mins)) * 60.0f;
        mins = Mathf.Floor(mins);

        var iMins = (int)mins;
        var iSecs = (int)secs;

        var secsFormat = iSecs < 10 ? $"0{iSecs}" : $"{iSecs}";
        return $"{iMins}:{secsFormat}";
    }

    private void LateUpdate()
    {
        if (isPlaying)
        {
            animationTime += Time.deltaTime;
            timeSlider.SetValueWithoutNotify(animationTime / animationDuration);
            if (animationTime > animationDuration)
            {
                animationTime = animationDuration;
                SetPaused();
            }
        }
        else
        {
            animationTime = timeSlider.value * animationDuration;
        }

        for (int i = 0; i < animations.Count; i++)
        {
            var colorIndex = Mathf.Floor(animationTime / animationCycleDuration) % boxColors.Count;
            animations[i].SetColor(boxColors[(int)colorIndex]);
            animations[i].Update(animationTime % animationCycleDuration);
        }

        //Time labels
        var remainingTime = Mathf.Round(animationDuration - animationTime);

        leftLabel.SetText(FormatTime(animationTime));
        rightLabel.SetText(FormatTime(remainingTime));

        //Animated background
        backgroundImage.materialForRendering.SetFloat(animationTimeID, animationTime);
    }
}
