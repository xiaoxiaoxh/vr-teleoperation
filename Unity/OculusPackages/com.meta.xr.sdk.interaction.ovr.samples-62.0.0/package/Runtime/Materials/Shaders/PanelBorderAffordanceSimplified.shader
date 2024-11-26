Shader "Unlit/PanelBorderAffordanceSimplified"
{
  Properties
  {
      _BaseOpacity("Base Opacity", Float) = 0.5 
      _SelectedOpacity("Selected Opacity", Float) = 1.0
      _MinFadeDistance("Min Fade Distance", Float) = 0
      _MaxFadeDistance("Max Fade Distance", Float) = 0.6
      [Toggle(ENABLE_OPACITY)] _EnableOpacity ("Enable Opacity", Float) = 0
  }
  SubShader
  {
    Tags
    {
        "Queue"="Transparent"
        "IgnoreProjector"="True"
        "RenderType" = "Transparent"
    }

    Cull Back
    Lighting Off
    ZWrite Off
    Blend SrcAlpha OneMinusSrcAlpha, OneMinusDstAlpha One

    Pass
    {
      CGPROGRAM
      #pragma vertex vert
      #pragma fragment frag

      #pragma multi_compile __ ENABLE_OPACITY

      #include "UnityCG.cginc"

      struct appdata
      {
        float4 vertex : POSITION;
        UNITY_VERTEX_INPUT_INSTANCE_ID
      };

      struct v2f {
        float4 vertex : SV_POSITION;
        float4 worldSpacePosition : COLOR0;
        UNITY_VERTEX_OUTPUT_STEREO
      };

      #define WORLD_SPACE_FADE_POINTS_COUNT 6
      float4 _WorldSpaceFadePoints[WORLD_SPACE_FADE_POINTS_COUNT];
      
      float _BaseOpacity;
      float _SelectedOpacity;
      float _MinFadeDistance;
      float _MaxFadeDistance;

      float _OpacityMultiplier;
      float _SelectedOpacityParam;
      int _UsedPointCount;

      v2f vert(appdata v) {
        v2f o;

        UNITY_INITIALIZE_OUTPUT(v2f, o);
        UNITY_SETUP_INSTANCE_ID(v);
        UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

        o.vertex = UnityObjectToClipPos(v.vertex);
        o.worldSpacePosition = mul(UNITY_MATRIX_M, v.vertex);
        return o;
      }

      fixed4 frag(v2f i) : SV_Target {
        float transparencyParam = 0.0;
        float3 worldPosition = i.worldSpacePosition.xyz;

        for (int idx = 0; idx < _UsedPointCount; ++idx) {
          float fadePointOpacity = _WorldSpaceFadePoints[idx].w;
          float3 fadePointPosition = _WorldSpaceFadePoints[idx].xyz;

          float distanceToFadePoint = distance(fadePointPosition, worldPosition);
          float fadeParam = smoothstep(_MinFadeDistance, _MaxFadeDistance, distanceToFadePoint);
          float invFadeParam = (1.0 - fadeParam);
          transparencyParam = max(transparencyParam, invFadeParam * invFadeParam * fadePointOpacity);
        }

        float fadePointsOpacity = lerp(_BaseOpacity, _SelectedOpacity, transparencyParam);
        float opacity = lerp(fadePointsOpacity, _SelectedOpacity, _SelectedOpacityParam);

#if ENABLE_OPACITY
        return fixed4(1.0, 1.0, 1.0, _OpacityMultiplier * opacity);
#else
        return fixed4(1.0, 1.0, 1.0, 1.0);
#endif
      }
      ENDCG
    }
  }
}
