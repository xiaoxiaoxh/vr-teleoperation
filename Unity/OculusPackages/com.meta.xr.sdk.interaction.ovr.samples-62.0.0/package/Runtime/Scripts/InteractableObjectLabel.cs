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

using Oculus.Interaction;
using Oculus.Interaction.UnityCanvas;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Label use to tag the possible interaction in each object. It appears when the user faces the object and disappears if the object is
/// not being faced directly or an interaction was triggered. The default material makes sure the labels appear in front of everything,
/// and uses the CanvasRenderTexture to render anything that is added to the canvas.
/// </summary>
public class InteractableObjectLabel : MonoBehaviour
{
    [Tooltip("The positions of this transoforms are used to check if the user is facing the object")]
    public List<Transform> viewTargets;
    [Tooltip("The possible positions for the label, the component always selected the one that has the highest y position component")]
    public List<Transform> labelPositions;
    [Tooltip("The position between the left and right cameras")]
    public Transform playerHead;
    [Tooltip("This group should contain all the interactions in the object, and when one is triggered the label is hidden")]
    public InteractableGroupView interactableGroup;
    private Vector3 _startScale;

    [Space(10)]
    [Tooltip("Canvas group at the root of the label canvas, used to make the canvas completely transparent")]
    public CanvasGroup group;
    public float alphaAnimationSpeed;
    public float focusDelay;
    public float hideDelay;
    public float alignmentThreshold;
    public float minScale;
    public float positionAnimationSpeed;

    [Space(10)]
    public Mesh quadMesh;
    public Material quadMaterial;
    public RectTransform canvasTransform;
    public CanvasRenderTexture canvasTexture;

    LabelState _currentState;

    enum LabelState
    {
        Hidden,
        FocusCheck,
        Focused,
        HideCheck,
        Used
    }

    private float _targetAlpha;
    private float _currentAlpha;
    private float _startTime;
    private MaterialPropertyBlock _block;
    private Transform _quadTransform;
    private MeshRenderer _quadRenderer;
    private Vector3 currentLabelPosition;

    protected bool _started;

    void Start()
    {
        _targetAlpha = 0.0f;

        this.BeginStart(ref _started);

        this.AssertIsTrue((alphaAnimationSpeed > 0.0f) && (positionAnimationSpeed > 0.0f), "Animation speed is 0.0 or less");
        this.AssertField(playerHead, nameof(playerHead));
        this.AssertField(interactableGroup, nameof(interactableGroup));
        this.AssertField(group, nameof(group));

        this.AssertField(quadMesh, nameof(quadMesh));
        this.AssertField(quadMaterial, nameof(quadMaterial));
        this.AssertField(canvasTransform, nameof(canvasTransform));
        this.AssertField(canvasTexture, nameof(canvasTexture));

        CreateTextureQuad();

        _startScale = _quadTransform.localScale;
        _block = new MaterialPropertyBlock();

        this.EndStart(ref _started);
    }

    private void CreateTextureQuad()
    {
        var quadGO = new GameObject();
        quadGO.name = "CanvasTexture";
        _quadTransform = quadGO.transform;
        _quadTransform.parent = transform;
        _quadTransform.localScale = new Vector3()
        {
            x = canvasTransform.sizeDelta.x * canvasTransform.localScale.x,
            y = canvasTransform.sizeDelta.y * canvasTransform.localScale.y,
            z = 1.0f
        };
        var quadMeshFilter = quadGO.AddComponent<MeshFilter>();
        quadMeshFilter.sharedMesh = quadMesh;
        _quadRenderer = quadGO.AddComponent<MeshRenderer>();
        _quadRenderer.sharedMaterial = quadMaterial;
    }

    private void OnEnable()
    {
        interactableGroup.WhenStateChanged += InteractableStateChange;
    }

    private void OnDisable()
    {
        interactableGroup.WhenStateChanged -= InteractableStateChange;
    }

    void SetTargetAlpha()
    {
        switch (_currentState)
        {
            case LabelState.Hidden:
                _targetAlpha = 0.0f;
                break;
            case LabelState.FocusCheck:
                _targetAlpha = 0.0f;
                break;
            case LabelState.Focused:
                _targetAlpha = 1.0f;
                break;
            case LabelState.HideCheck:
                _targetAlpha = 1.0f;
                break;
            case LabelState.Used:
                _targetAlpha = 0.0f;
                break;
            default:
                break;
        }
    }

    float MaximizedDotView()
    {
        if (viewTargets == null)
        {
            return 0.0f;
        }
        var maxDotView = -1.0f;
        foreach (var target in viewTargets)
        {
            var viewVector = Vector3.Normalize(target.position - playerHead.position);
            var dotView = Vector3.Dot(playerHead.forward, viewVector);
            maxDotView = Mathf.Max(maxDotView, dotView);
        }
        return maxDotView;
    }

    void StateTransition()
    {
        var dotView = MaximizedDotView();
        switch (_currentState)
        {
            case LabelState.Hidden:
                if (dotView >= alignmentThreshold)
                {
                    _currentState = LabelState.FocusCheck;
                    _startTime = Time.time;
                }
                break;
            case LabelState.FocusCheck:
                if (dotView < alignmentThreshold)
                {
                    _currentState = LabelState.Hidden;
                }
                else if (Time.time - _startTime >= focusDelay)
                {
                    _currentState = LabelState.Focused;
                }
                break;
            case LabelState.Focused:
                if (dotView <= alignmentThreshold)
                {
                    _currentState = LabelState.HideCheck;
                    _startTime = Time.time;
                }
                break;
            case LabelState.HideCheck:
                if (dotView > alignmentThreshold)
                {
                    _currentState = LabelState.Focused;
                }
                else if (Time.time - _startTime >= hideDelay)
                {
                    _currentState = LabelState.Hidden;
                }
                break;
            default:
                break;
        }
    }

    void InteractableStateChange(InteractableStateChangeArgs args)
    {
        switch (args.NewState)
        {
            case InteractableState.Select:
                _currentState = LabelState.Used;
                break;
            case InteractableState.Normal:
                if(_currentState == LabelState.Used)
                {
                    _currentState = LabelState.Hidden;
                }
                break;
            default:
                break;
        }
    }

    Vector3 FindHighestLabelPosition()
    {
        if (labelPositions == null)
        {
            return transform.position;
        }
        else
        {
            var selectedIndex = -1;
            var selectedY = transform.position.y;

            for (int i = 0; i < labelPositions.Count; i++)
            {
                var y = labelPositions[i].position.y;
                if(y > selectedY)
                {
                    selectedIndex = i;
                    selectedY = y;
                }
            }

            if (selectedIndex == -1)
            {
                return transform.position;
            }
            else
            {
                return labelPositions[selectedIndex].position;
            }
        }
    }

    void UpdateLabelTransform()
    {
        var labelPosition = FindHighestLabelPosition();
        currentLabelPosition = Vector3.Lerp(currentLabelPosition, labelPosition, positionAnimationSpeed);

        var distance = Vector3.Distance(currentLabelPosition, playerHead.position);
        var scaleMultiplier = Mathf.Max(minScale, distance);
        _quadTransform.localScale = _startScale * scaleMultiplier;

        var halfSize = _quadTransform.localScale.y * 0.5f;
        _quadTransform.position = currentLabelPosition + _quadTransform.up * halfSize;

        _quadTransform.LookAt(playerHead.position);
        _quadTransform.localRotation *= Quaternion.Euler(0.0f, 180.0f, 0.0f);
    }

    void Update()
    {
        UpdateLabelTransform();

        SetTargetAlpha();
        StateTransition();

        _currentAlpha = Mathf.Lerp(_currentAlpha, _targetAlpha, alphaAnimationSpeed);
        group.alpha = Mathf.Clamp01(_currentAlpha);

        _block.SetTexture("_MainTex", canvasTexture.Texture);
        _quadRenderer.SetPropertyBlock(_block);
    }
}
