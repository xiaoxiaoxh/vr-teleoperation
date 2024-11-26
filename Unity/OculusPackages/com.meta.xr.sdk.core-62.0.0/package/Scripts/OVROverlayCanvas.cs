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
using UnityEngine.Rendering;
using UnityEngine.Serialization;

[RequireComponent(typeof(Canvas))]
[ExecuteAlways]
public class OVROverlayCanvas : OVRRayTransformer
{
    public enum DrawMode
    {
        Opaque,
        OpaqueWithClip,
        TransparentDefaultAlpha,
        TransparentCorrectAlpha
    }

    public enum CanvasShape
    {
        Flat,
        Curved
    }

    [SerializeField, HideInInspector]
    private Shader _overrideCanvasShader = null;

    [FormerlySerializedAs("_transparentShader")]
    [SerializeField, HideInInspector]
    private Shader _transparentImposterShader = null;

    [FormerlySerializedAs("_opaqueShader")]
    [SerializeField, HideInInspector]
    private Shader _opaqueImposterShader = null;

    private RectTransform _rectTransform;
    private Canvas _canvas;
    private Camera _camera;
    private OVROverlay _overlay;
    private MeshRenderer _meshRenderer;
    private OVROverlayMeshGenerator _meshGenerator;

    private RenderTexture _renderTexture;

    private Material _imposterMaterial;

    private float _optimalResolutionWidth;
    private float _optimalResolutionHeight;

    private float _lastPixelWidth;
    private float _lastPixelHeight;

    private Vector2 _imposterTextureOffset;
    private Vector2 _imposterTextureScale;

    private bool _hasRenderedFirstFrame;

    private readonly bool _scaleViewport = Application.isMobilePlatform;

    [FormerlySerializedAs("MaxTextureSize")] public int maxTextureSize = 4096;
    [FormerlySerializedAs("DrawRate")] public int renderInterval = 1;
    [FormerlySerializedAs("DrawFrameOffset")] public int renderIntervalFrameOffset = 0;
    [FormerlySerializedAs("Expensive")] public bool expensive = false;
    [FormerlySerializedAs("Layer")] public int layer = 5;
    [FormerlySerializedAs("Opacity")] public DrawMode opacity = DrawMode.TransparentDefaultAlpha;
    public bool overrideDefaultCanvasMaterial = false;
    public CanvasShape shape = CanvasShape.Flat;
    public float curveRadius = 1.0f;

    [SerializeField]
    private bool _overlayEnabled = true;

    private static readonly Plane[] _FrustumPlanes = new Plane[6];
    private static readonly Vector3[] _WorldCorners = new Vector3[4];

    public bool overlayEnabled
    {
        get { return _overlayEnabled; }
        set
        {
            if (_overlay && Application.isPlaying)
            {
                _overlay.enabled = value;
                // Update our impostor color to switch between visible and punch-a-hole
                _imposterMaterial.color = value ? Color.black : Color.white;
            }
            _overlayEnabled = value;
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        if (UnityEngine.XR.XRSettings.isDeviceActive)
        {
            _optimalResolutionWidth = UnityEngine.XR.XRSettings.eyeTextureWidth * 2;
            _optimalResolutionHeight = UnityEngine.XR.XRSettings.eyeTextureHeight * 2;
        }
        else
        {
            _optimalResolutionWidth = Screen.width * 2;
            _optimalResolutionHeight = Screen.height * 2;
        }

        _canvas = GetComponent<Canvas>();
        _rectTransform = _canvas.GetComponent<RectTransform>();

        HideFlags hideFlags = HideFlags.DontSave | HideFlags.NotEditable | HideFlags.HideInHierarchy;

        GameObject overlayCamera = new GameObject(name + " Overlay Camera") { hideFlags = hideFlags };
        overlayCamera.transform.SetParent(transform, false);

        _camera = overlayCamera.AddComponent<Camera>();
        _camera.stereoTargetEye = StereoTargetEyeMask.None;
        _camera.transform.position = transform.position - transform.forward;
        _camera.orthographic = true;
        _camera.enabled = false;
        _camera.clearFlags = CameraClearFlags.SolidColor;
        _camera.backgroundColor = Color.clear;
        _camera.nearClipPlane = 0.99f;
        _camera.farClipPlane = 1.01f;

        GameObject imposter = new GameObject(name + " Imposter") { hideFlags = hideFlags };

        imposter.transform.SetParent(transform, false);
        imposter.AddComponent<MeshFilter>();
        _meshRenderer = imposter.AddComponent<MeshRenderer>();
        _meshGenerator = imposter.AddComponent<OVROverlayMeshGenerator>();

        GameObject overlay = new GameObject(name + " Overlay") { hideFlags = hideFlags };
        overlay.transform.SetParent(transform, false);
        _overlay = overlay.AddComponent<OVROverlay>();
        _overlay.enabled = false;
        _overlay.isDynamic = true;
        _overlay.noDepthBufferTesting = true;
        _overlay.isAlphaPremultiplied = true;
        _overlay.currentOverlayType = OVROverlay.OverlayType.Underlay;

        InitializeRenderTexture();
    }

    private void InitializeRenderTexture()
    {
        float rectWidth = _rectTransform.rect.width;
        float rectHeight = _rectTransform.rect.height;

        float aspectX = rectWidth >= rectHeight ? 1 : rectWidth / rectHeight;
        float aspectY = rectHeight >= rectWidth ? 1 : rectHeight / rectWidth;

        // if we are scaling the viewport we don't need to add a border
        int pixelBorder = _scaleViewport ? 0 : 8;
        int innerWidth = Mathf.CeilToInt(aspectX * (maxTextureSize - pixelBorder * 2));
        int innerHeight = Mathf.CeilToInt(aspectY * (maxTextureSize - pixelBorder * 2));
        int width = innerWidth + pixelBorder * 2;
        int height = innerHeight + pixelBorder * 2;

        float paddedWidth = rectWidth * (width / (float)innerWidth);
        float paddedHeight = rectHeight * (height / (float)innerHeight);

        float insetRectWidth = innerWidth / (float)width;
        float insetRectHeight = innerHeight / (float)height;

        _imposterTextureOffset = new Vector2(0.5f - 0.5f * insetRectWidth, 0.5f - 0.5f * insetRectHeight);
        _imposterTextureScale = new Vector2(insetRectWidth, insetRectHeight);

        if (_renderTexture == null || _renderTexture.width != width || _renderTexture.height != height)
        {
            if (_renderTexture != null)
            {
                DestroyImmediate(_renderTexture);
            }

            _renderTexture = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
            // if we can't scale the viewport, generate mipmaps instead
            _renderTexture.useMipMap = !_scaleViewport;
            _renderTexture.filterMode = FilterMode.Trilinear;
        }

        _camera.orthographicSize = 0.5f * paddedHeight * _rectTransform.localScale.y;
        _camera.targetTexture = _renderTexture;
        _camera.cullingMask = 1 << gameObject.layer;

        Shader shader = opacity == DrawMode.Opaque || opacity == DrawMode.OpaqueWithClip
            ? _opaqueImposterShader
            : _transparentImposterShader;

        if (_imposterMaterial == null)
        {
            _imposterMaterial = new Material(shader);
        }
        else
        {
            _imposterMaterial.shader = shader;
        }

        if (opacity == DrawMode.OpaqueWithClip)
        {
            _imposterMaterial.EnableKeyword("WITH_CLIP");
        }
        else
        {
            _imposterMaterial.DisableKeyword("WITH_CLIP");
        }

        if (opacity == DrawMode.TransparentDefaultAlpha)
        {
            _imposterMaterial.EnableKeyword("ALPHA_SQUARED");
        }
        else
        {
            _imposterMaterial.DisableKeyword("ALPHA_SQUARED");
        }

        if (expensive)
        {
            _imposterMaterial.EnableKeyword("EXPENSIVE");
        }
        else
        {
            _imposterMaterial.DisableKeyword("EXPENSIVE");
        }

        _imposterMaterial.mainTexture = _renderTexture;
        _imposterMaterial.color = Color.black;
        _imposterMaterial.mainTextureOffset = _imposterTextureOffset;
        _imposterMaterial.mainTextureScale = _imposterTextureScale;

        _meshRenderer.sharedMaterial = _imposterMaterial;
        _meshRenderer.gameObject.layer = layer;

        if (shape == CanvasShape.Flat)
        {
            _meshRenderer.transform.localPosition = _overlay.transform.localPosition = Vector3.zero;
            _meshRenderer.transform.localScale = new Vector3(rectWidth, rectHeight, 1);
            _overlay.transform.localScale = new Vector3(paddedWidth, paddedHeight, 1);
        }
        else
        {
            _meshRenderer.transform.localPosition = _overlay.transform.localPosition = new Vector3(0, 0, -curveRadius / transform.lossyScale.z);
            _meshRenderer.transform.localScale = new Vector3(rectWidth, rectHeight, curveRadius / transform.lossyScale.z);
            _overlay.transform.localScale = new Vector3(paddedWidth, paddedHeight, curveRadius / transform.lossyScale.z);
        }

        _overlay.textures[0] = _renderTexture;
        _overlay.currentOverlayShape = shape == CanvasShape.Flat
            ? OVROverlay.OverlayShape.Quad
            : OVROverlay.OverlayShape.Cylinder;

        _overlay.useExpensiveSuperSample = expensive;
        _overlay.enabled = Application.isPlaying && _overlayEnabled;

        _meshGenerator.SetOverlay(_overlay);

        if (overrideDefaultCanvasMaterial)
        {
            Canvas.GetDefaultCanvasMaterial().shader = _overrideCanvasShader;
        }
    }

    private void OnDestroy()
    {
        if (Application.isPlaying)
        {
            Destroy(_imposterMaterial);
            Destroy(_renderTexture);
        }
        else
        {
            DestroyImmediate(_imposterMaterial);
            DestroyImmediate(_renderTexture);
        }
    }

    private void OnEnable()
    {
        if (_overlay)
        {
            _meshRenderer.enabled = true;
            _overlay.enabled = Application.isPlaying && _overlayEnabled;
        }
    }

    private void OnDisable()
    {
        if (_overlay)
        {
            _overlay.enabled = false;
            _meshRenderer.enabled = false;
        }
    }

    protected virtual bool ShouldRender()
    {
        if (renderInterval > 1)
        {
            if (Time.frameCount % renderInterval != renderIntervalFrameOffset % renderInterval && _hasRenderedFirstFrame)
            {
                return false;
            }
        }

        // Always render in the editor
        if (Application.isEditor)
        {
            return true;
        }

        var mainCamera = OVRManager.FindMainCamera();
        if (mainCamera != null)
        {
            // Perform Frustum culling
            if (mainCamera.stereoEnabled)
            {
                for (int i = 0; i < 2; i++)
                {
                    var eye = (Camera.StereoscopicEye)i;
                    var mat = mainCamera.GetStereoProjectionMatrix(eye) * mainCamera.GetStereoViewMatrix(eye);
                    GeometryUtility.CalculateFrustumPlanes(mat, _FrustumPlanes);
                    if (GeometryUtility.TestPlanesAABB(_FrustumPlanes, _meshRenderer.bounds))
                    {
                        return true;
                    }
                }
            }
            else
            {
                var mat = mainCamera.projectionMatrix * mainCamera.worldToCameraMatrix;
                GeometryUtility.CalculateFrustumPlanes(mat, _FrustumPlanes);
                if (GeometryUtility.TestPlanesAABB(_FrustumPlanes, _meshRenderer.bounds))
                {
                    return true;
                }
            }

            return false;
        }

        return true;
    }

    private void Update()
    {
        if (!ShouldRender())
            return;

        ApplyViewportScale();
        _hasRenderedFirstFrame = true;
        _camera.Render();
    }

    private void LateUpdate()
    {
        // Update our impostor color to switch between visible and punch-a-hole
        _imposterMaterial.color = _overlay.enabled && _overlay.isOverlayVisible ? Color.black : Color.white;
        // Update the scale and offset each frame to avoid a bug where Unity likes to reset them for some reason
        _imposterMaterial.mainTextureScale = _imposterTextureScale;
        _imposterMaterial.mainTextureOffset = _imposterTextureOffset;
    }

    private void ApplyViewportScale()
    {
        if (!_scaleViewport)
            return;

        var mainCamera = OVRManager.FindMainCamera();
        if (mainCamera == null)
            return;

        _rectTransform.GetWorldCorners(_WorldCorners);

        // Calculate Clip Pos for our quad
        for (int i = 0; i < 4; i++)
        {
            var corner = mainCamera.WorldToScreenPoint(_WorldCorners[i]);
            corner.x *= _optimalResolutionWidth / mainCamera.pixelWidth;
            corner.y *= _optimalResolutionHeight / mainCamera.pixelHeight;
            _WorldCorners[i] = corner;
        }

        // Because our quad might be rotated, we find the raw max pixel length of each quad side
        float height = Mathf.Max((_WorldCorners[1] - _WorldCorners[0]).magnitude, (_WorldCorners[3] - _WorldCorners[2]).magnitude);
        float width = Mathf.Max((_WorldCorners[2] - _WorldCorners[1]).magnitude, (_WorldCorners[3] - _WorldCorners[0]).magnitude);

        // round to the nearest even pixel size
        float pixelHeight = Mathf.Ceil(height / 2 + 1) * 2 * (expensive ? 2 : 1);
        float pixelWidth = Mathf.Ceil(width / 2 + 1) * 2 * (expensive ? 2 : 1);

        // clamp our viewport to the texture size
        pixelHeight = Mathf.Min(pixelHeight, _renderTexture.height);
        pixelWidth = Mathf.Min(pixelWidth, _renderTexture.width);

        // Don't change texture sizes unless our image would change more than four pixels to avoid judder
        if (Math.Abs(pixelHeight - _lastPixelHeight) < 4 && Math.Abs(pixelWidth - _lastPixelWidth) < 4)
        {
            pixelWidth = _lastPixelWidth;
            pixelHeight = _lastPixelHeight;
        }
        else
        {
            _lastPixelHeight = pixelHeight;
            _lastPixelWidth = pixelWidth;
        }

        float innerPixelHeight = pixelHeight - 2;
        float innerPixelWidth = pixelWidth - 2;

        float orthoHeight = _rectTransform.rect.height * _rectTransform.localScale.y *
            pixelHeight / innerPixelHeight;
        float orthoWidth = _rectTransform.rect.width * _rectTransform.localScale.x *
            pixelWidth / innerPixelWidth;

        _camera.orthographicSize = (0.5f * orthoHeight);
        _camera.aspect = (orthoWidth / orthoHeight);

        float sizeX = pixelWidth / _renderTexture.width;
        float sizeY = pixelHeight / _renderTexture.height;

        float innerSizeX = innerPixelWidth / _renderTexture.width;
        float innerSizeY = innerPixelHeight / _renderTexture.height;

        // scale the camera rect
        _camera.rect = new Rect((1 - sizeX) / 2, ((1 - sizeY) / 2), sizeX, sizeY);

        Rect src = new Rect(0.5f - (0.5f * innerSizeX), (0.5f - (0.5f * innerSizeY)), innerSizeX, innerSizeY);
        Rect dst = new Rect(0, 0, 1, 1);

        // update the overlay to use this same size
        _overlay.overrideTextureRectMatrix = true;
        _overlay.SetSrcDestRects(src, src, dst, dst);

        // Update our material offset and scale
        _imposterTextureOffset = src.min;
        _imposterTextureScale = src.size;
    }


    public override Ray TransformRay(Ray ray)
    {
        if (shape != CanvasShape.Curved)
        {
            return ray;
        }

        // Transform from 3D space to Curved UI Space
        var localPoint = transform.InverseTransformPoint(ray.origin);
        var localDirection = transform.InverseTransformDirection(ray.direction);

        var localRadius = curveRadius / transform.lossyScale.z;
        var localCenter = new Vector3(0, 0, -localRadius);

        // find xz intersect with circle

        if (!LineCircleIntersection(new Vector2(localPoint.x, localPoint.z),
                new Vector2(localDirection.x, localDirection.z), new Vector2(localCenter.x, localCenter.z), localRadius,
                out float distance))
        {
            // the ray misses, return a ray parallel to the canvas
            return new Ray(ray.origin, transform.right);
        }

        Vector3 localIntersection = localPoint + localDirection * distance;

        // convert circle coordinates to plane coordinates by getting angle
        float angle = Mathf.Atan2(localIntersection.x, localIntersection.z + localRadius);
        float xPos = angle * localRadius;
        float yPos = localIntersection.y;

        // return a ray going directly into our calculated intersection point
        return new Ray(transform.TransformPoint(new Vector3(xPos, yPos, -1)), transform.forward);
    }

    private static bool LineCircleIntersection(Vector2 p1, Vector2 dp, Vector2 center, float radius, out float distance)
    {
        //  Find intersection with circle using quadratic equation
        var a = dp.sqrMagnitude;
        var b = 2 * Vector2.Dot(dp, p1 - center);
        var c = center.sqrMagnitude;
        c += p1.sqrMagnitude;
        c -= 2 * Vector2.Dot(center, p1);
        c -= radius * radius;
        var bb4ac = b * b - 4 * a * c;
        if (Mathf.Abs(a) < float.Epsilon || bb4ac < 0)
        {
            //  line does not intersect
            distance = default;
            return false;
        }
        var mu1 = (-b - Mathf.Sqrt(bb4ac)) / (2 * a);
        var mu2 = (-b + Mathf.Sqrt(bb4ac)) / (2 * a);

        distance = mu1 >= 0 ? mu1 : mu2;
        return true;
    }
}
