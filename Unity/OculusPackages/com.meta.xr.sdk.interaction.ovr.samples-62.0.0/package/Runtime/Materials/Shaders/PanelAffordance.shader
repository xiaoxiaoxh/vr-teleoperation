Shader "Unlit/PanelAffordance"
{
  Properties
  {
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

      #define WORLD_SPACE_FADE_POINTS_COUNT 2
      float4 _WorldSpaceFadePoints[WORLD_SPACE_FADE_POINTS_COUNT];
      float _Opacity;

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
        float distanceMeters = lerp(0.1, distance(i.worldSpacePosition.xyz, _WorldSpaceFadePoints[0].xyz), _WorldSpaceFadePoints[0].w);
        for (int idx = 1; idx < WORLD_SPACE_FADE_POINTS_COUNT; ++idx) {
          distanceMeters = min(distanceMeters, lerp(0.1, distance(i.worldSpacePosition.xyz, _WorldSpaceFadePoints[idx].xyz), _WorldSpaceFadePoints[idx].w));
        }
        float distanceCentimeters = 100.0 * distanceMeters;
        return fixed4(1.0, 1.0, 1.0, _Opacity * min(1.0, max(distanceCentimeters - 2.0, 0.0) * 0.1));
      }
      ENDCG
    }
  }
}
