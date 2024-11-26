Shader "Oculus/UnlitTransparentCubemap" {
    Properties {
        _MainTex ("Cubemap", Cube) = "black" {}
         _Tint ("Tint Color", Color) = (1,1,1,1)
        _FrontfaceAlphaMultiplier("Fontface alpha multiplier", Float) = 0.75
        _BackfaceAlphaMultiplier("Backface alpha multiplier", Float) = 0.25
    }

    SubShader {
        Tags {"Queue"="Transparent" "IgnoreProjector"="True" "RenderType"="Transparent"}
        LOD 100

        ZWrite Off
        Blend SrcAlpha OneMinusSrcAlpha
        Cull Front

        Pass {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata {
                float4 vertex : POSITION;
                float3 normal : NORMAL;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f {
                float4 pos : SV_POSITION;
                float3 worldNormal : TEXCOORD0;
                float3 worldPos : TEXCOORD1;
                float3 worldView : TEXCOORD2;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            samplerCUBE _MainTex;
            float _BackfaceAlphaMultiplier;
            fixed4 _Tint;

            v2f vert (appdata v) {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
                o.pos = UnityObjectToClipPos(v.vertex);
                o.worldNormal = UnityObjectToWorldNormal(v.normal);
                o.worldPos = mul(unity_ObjectToWorld, v.vertex).xyz;
                o.worldView = normalize(UnityWorldSpaceViewDir(o.worldPos));
                return o;
            }

            fixed4 frag (v2f i) : SV_Target {
                float3 reflection = reflect(-i.worldView * -1, i.worldNormal);
                float4 col = texCUBE(_MainTex, reflection);
                col.rgb *= _Tint.rgb;
                col.a = (col.r * _BackfaceAlphaMultiplier) + _Tint.a;
                return col;
            }
            ENDCG
        }

        Pass {
            Tags {"Queue"="Transparent" "IgnoreProjector"="True" "RenderType"="Transparent"}
            LOD 100

            ZWrite Off
            Blend SrcAlpha OneMinusSrcAlpha
            Cull Back

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata {
                float4 vertex : POSITION;
                float3 normal : NORMAL;
            };

            struct v2f {
                float4 pos : SV_POSITION;
                float3 worldNormal : TEXCOORD0;
                float3 worldPos : TEXCOORD1;
                float3 worldView : TEXCOORD2;
            };

            samplerCUBE _MainTex;
            float _FrontfaceAlphaMultiplier;
            fixed4 _Tint;

            v2f vert (appdata v) {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.worldNormal = UnityObjectToWorldNormal(v.normal);
                o.worldPos = mul(unity_ObjectToWorld, v.vertex).xyz;
                o.worldView = normalize(UnityWorldSpaceViewDir(o.worldPos));
                return o;
            }

            fixed4 frag (v2f i) : SV_Target {
                float3 reflection = reflect(-i.worldView, i.worldNormal);
                float4 col = texCUBE(_MainTex, reflection);
                col.rgb *= _Tint.rgb;
                col.a = (col.r * _FrontfaceAlphaMultiplier) + _Tint.a;
                return col;
            }
        ENDCG
        }
    }
    FallBack "Unlit/Color"
}
