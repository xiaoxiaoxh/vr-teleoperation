/************************************************************************************
Copyright : Copyright (c) Facebook Technologies, LLC and its affiliates. All rights reserved.

Your use of this SDK or tool is subject to the Oculus SDK License Agreement, available at
https://developer.oculus.com/licenses/oculussdk/

Unless required by applicable law or agreed to in writing, the Utilities SDK distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF
ANY KIND, either express or implied. See the License for the specific language governing
permissions and limitations under the License.
************************************************************************************/

Shader "Oculus/DimmableLitWithRadialAlpha"
{
    Properties
    {
        [NoScaleOffset] _MainTex ("Albedo (RGB)", 2D) = "white" {}
        _DimColor ("Dim Color", Color) = (1,1,1,1)
        _DimBlend ("Dim Blend", Range(0,1)) = 0
        _Tiling ("Tiling", float) = 1
        _Rotate ("Rotate", Range(0,360)) = 0

        _Power ("Alpha Power", Range(0.001,1)) = 1

        _Glossiness ("Smoothness", Range(0,1)) = 0.5
        _Metallic ("Metallic", Range(0,1)) = 0.0
    }
    SubShader
    {
        Tags {"Queue"="Transparent" "IgnoreProjector"="True" "RenderType"="Transparent"}
        LOD 100

        ZWrite Off
        Blend SrcAlpha OneMinusSrcAlpha

        CGPROGRAM
        #pragma surface surf Standard fullforwardshadows alpha:blend
        #pragma target 3.0

        sampler2D _MainTex;

        struct Input
        {
            float2 uv_MainTex;
        };

        half _Glossiness;
        half _Metallic;
        fixed4 _DimColor;
        float _DimBlend;
        float _Power;
        float _Tiling;
        float _Rotate;

        float2 RotateUV(float2 UV)
        {
            float radians = _Rotate * (3.1415926538 * 2 / 360);
            float2 center = float2 (0.5, 0.5);
            UV -= center;
            float s = sin(radians);
            float c = cos(radians);

            float2x2 rMatrix = float2x2(c, -s, s, c);
            rMatrix *= 0.5;
            rMatrix += 0.5;
            rMatrix = rMatrix*2 - 1;

            UV.xy = mul(UV.xy, rMatrix);
            UV += center;
            return UV;
        }

        void surf (Input IN, inout SurfaceOutputStandard o)
        {
            float2 uv = RotateUV(IN.uv_MainTex) * _Tiling;
            fixed4 c = tex2D (_MainTex, uv);
            o.Albedo = lerp (c.rgb, _DimColor, _DimBlend);

            float fading = distance(IN.uv_MainTex, float2(0.5, 0.5));
            fading = smoothstep (0.5, 0.0, fading);
            fading = pow(fading, _Power);
            o.Alpha = fading;

            o.Metallic = _Metallic;
            o.Smoothness = _Glossiness;
        }
        ENDCG
    }
}
