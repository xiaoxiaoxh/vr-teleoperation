Shader "Unlit/ParticleAdditive"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _AlphaColor ("Alpha Color", Color) = (1.0, 1.0, 1.0, 1.0)
    }
    SubShader
    {
        Tags
	    {
		    "Queue"="Transparent"
		    "IgnoreProjector"="True"
		    "RenderType"="Transparent"
	    }

        Cull Back
        Lighting Off
        ZWrite Off
        Blend SrcAlpha One, OneMinusDstAlpha One

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct vertexInput
            {
                float4 vertex : POSITION;
                float4 color : COLOR;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct fragmentInput
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                float4 color : COLOR;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            float4 _AlphaColor;

            fragmentInput vert(vertexInput input)
            {
                fragmentInput output;

                UNITY_INITIALIZE_OUTPUT(fragmentInput, output);
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);

                output.vertex = UnityObjectToClipPos(input.vertex);
                output.uv = TRANSFORM_TEX(input.uv, _MainTex);
                output.color = input.color;
                return output;
            }

            fixed4 frag(fragmentInput input) : SV_Target
            {
                fixed4 col = tex2D(_MainTex, input.uv);
                col.xyz = lerp(_AlphaColor.xyz, col.xyz, col.w);
                col.w *= _AlphaColor.w;
                return col * input.color;
            }
            ENDCG
        }
    }
}
