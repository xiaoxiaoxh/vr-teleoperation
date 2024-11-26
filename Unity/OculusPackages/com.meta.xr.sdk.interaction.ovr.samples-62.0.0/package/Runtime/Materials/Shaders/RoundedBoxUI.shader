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

Shader "Unlit/RoundedBoxUI"
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
        _BorderWidth ("Border Width", Float) = 0
        [Enum(NoBorder,0,OnlyBorder,1,Both,2)] _BorderColorType ("Border Type", Int) = 0
        [Toggle(IMAGE_SDF)] _UseImageAsSDF ("Use Image as SDF", Float) = 0
        [Enum(Off,0,On,1)]_ZWrite ("ZWrite", Float) = 1.0
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
        ZWrite [_ZWrite]
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
            #pragma multi_compile __ IMAGE_SDF

            #include "UnityCG.cginc"
            #include "UnityUI.cginc"
            #include "../../ThirdParty/Box2DSignedDistance.cginc"

            struct vertexInput
            {
                float4 vertex : POSITION;
                float4 color : COLOR;
                float4 texcoord : TEXCOORD0;
                //--- Custom
                float4 borderRadius : TEXCOORD1;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct fragmentInput
            {
                float4 vertex : SV_POSITION;
                fixed4 color : COLOR;
                float4 texcoord : TEXCOORD0;
                float4 worldPosition : TEXCOORD1;
                //--- Custom
                float4 borderRadius : TEXCOORD2;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            fixed4 _Color;
            fixed4 _TextureSampleAdd;
            float4 _ClipRect;
            float _BorderWidth;
            int _BorderColorType;

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
                output.borderRadius = input.borderRadius;

                return output;
            }

            fixed4 frag (fragmentInput input) : SV_Target
            {
                float2 rectSize = input.texcoord.zw;
                float2 uv = input.texcoord.xy * rectSize;
                uv = uv - rectSize * 0.5;

                float dist = sdRoundBox(uv, rectSize * 0.5 - (_BorderWidth).xx, input.borderRadius);
                float2 ddDist = float2(ddx(dist), ddy(dist));
                float ddDistLen = length(ddDist);

                float alpha = saturate(((dist - _BorderWidth) / ddDistLen) + 1.0);
                float borderParam = saturate((dist) / ddDistLen);

                half4 color = half4(0.0, 0.0, 0.0, 0.0);

                #ifdef IMAGE_SDF
                    float4 texSample = tex2D(_MainTex, input.texcoord) + _TextureSampleAdd;
                    float c_dist = texSample.x - 0.1;
                    float c_mask = smoothstep(0.00, 0.2, c_dist);

                    color = input.color;
                    color.a *= 1.0 - alpha;
                    color.a *= saturate(c_mask);
                #else
                    color = (tex2D(_MainTex, input.texcoord) + _TextureSampleAdd) * input.color;
                    color.a *= 1.0 - alpha;
                #endif

                if (_BorderColorType == 1) {
                    color.a *= borderParam;
                }
                //color.rgb *= 1.0 - borderParam;

                //color.a *= c_dist;
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
