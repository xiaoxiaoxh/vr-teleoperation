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

Shader "Unlit/MultiGradientUI"
{
    Properties
    {
        [PerRendererData] _MainTex ("Texture", 2D) = "white" {}
        _Color ("Tint", Color) = (1,1,1,1)

        [HideInInspector] _StencilComp		("Stencil Comparison", Float) = 0
	    [HideInInspector] _Stencil			("Stencil ID", Float) = 0
	    [HideInInspector] _StencilOp		("Stencil Operation", Float) = 0
	    [HideInInspector] _StencilWriteMask	("Stencil Write Mask", Float) = 255
	    [HideInInspector] _StencilReadMask	("Stencil Read Mask", Float) = 255

	    _ColorMask ("Color Mask", Float) = 15
        [Toggle(UNITY_UI_ALPHACLIP)] _UseUIAlphaClip ("Use Alpha Clip", Float) = 0
        
        _Color0("Radial Color 0", Color) = (1, 1, 1, 1)
        _GradientPosition0("Gradient Start - End", Vector) = (0,0,1,1)
        _Color1("Radial Color 1", Color) = (1, 1, 1, 1)
        _GradientPosition1("Gradient Start - End", Vector) = (0,0,1,1)
        _Color2("Radial Color 2", Color) = (1, 1, 1, 1)
        _GradientPosition2("Gradient Start - End", Vector) = (0,0,1,1)
    }
    SubShader
    {
        Tags
	    {
		    "Queue"="Transparent"
		    "IgnoreProjector"="True"
		    "RenderType"="Transparent"
	    }

	    Stencil
	    {
		    Ref [_Stencil]
		    Comp [_StencilComp]
		    Pass [_StencilOp]
		    ReadMask [_StencilReadMask]
		    WriteMask [_StencilWriteMask]
	    }

	    Cull Off
        Lighting Off
        ZWrite Off
        ZTest [unity_GUIZTestMode]
        Blend SrcAlpha OneMinusSrcAlpha, OneMinusDstAlpha One
        ColorMask [_ColorMask]

        Pass
        {
            Name "Default"
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 2.0
		    #pragma multi_compile __ UNITY_UI_CLIP_RECT
		    #pragma multi_compile __ UNITY_UI_ALPHACLIP
            #pragma multi_compile __ ADD_UV_NOISE

            #include "UnityCG.cginc"
            #include "UnityUI.cginc"

            struct vertexInput
            {
                float4 vertex : POSITION;
                float4 color : COLOR;
                float4 texcoord : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct fragmentInput
            {
                float4 vertex : SV_POSITION;
                fixed4 color : COLOR;
                float4 texcoord : TEXCOORD0;
                float4 worldPosition : TEXCOORD1;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            fixed4 _Color;
            fixed4 _TextureSampleAdd;
            float4 _ClipRect;

            float4 _Color0;
            float4 _GradientPosition0;
            float4 _Color1;
            float4 _GradientPosition1;
            float4 _Color2;
            float4 _GradientPosition2;

            fragmentInput vert(vertexInput input)
            {
                fragmentInput output;

                UNITY_INITIALIZE_OUTPUT(fragmentInput, output);
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);

                output.worldPosition = input.vertex;
                output.vertex = UnityObjectToClipPos(output.worldPosition);
                output.texcoord = input.texcoord;
                output.color = input.color * _Color;

                return output;
            }

            float4 gradient(float2 uv, float2 center, float radius, float4 color) {
                float dist = 1.0 - saturate(length(center - uv) / radius);
                return lerp(float4(color.rgb, 0.0), color, dist);
            }

            fixed4 frag (fragmentInput input) : SV_Target
            {
                float2 rectSize = input.texcoord.zw;
                float2 uv = input.texcoord.xy;

                half4 color = half4(0.0, 0.0, 0.0, 0.0);

                float2 uvNoise = uv;

                float4 grad0 = gradient(uvNoise, _GradientPosition0.xy, _GradientPosition0.z, _Color0);
                float4 grad1 = gradient(uvNoise, _GradientPosition1.xy, _GradientPosition1.z, _Color1);
                float4 grad2 = gradient(uvNoise, _GradientPosition2.xy, _GradientPosition2.z, _Color2);

                color.a = max(max(grad0.a, grad1.a), grad2.a) * input.color.a;
                color.rgb = grad0.rgb * grad0.a + grad1.rgb * grad1.a + grad2.rgb * grad2.a;
                #ifdef UNITY_UI_CLIP_RECT
                    color.a *= UnityGet2DClipping(input.worldPosition.xy, _ClipRect);
                #endif

                #ifdef UNITY_UI_ALPHACLIP
                    clip (color.a - 0.001);
                #endif
                return color;
            }
            ENDCG
        }
    }
}
