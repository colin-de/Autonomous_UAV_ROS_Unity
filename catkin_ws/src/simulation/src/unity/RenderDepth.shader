// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

 Shader "Custom/RenderDepth"
 {
     SubShader
     {
         Pass
         {
             CGPROGRAM
             #pragma vertex vert
             #pragma fragment frag
             #include "UnityCG.cginc"

             uniform sampler2D _CameraDepthTexture;

             struct input_t
             {
                 float4 pos : POSITION;
                 half2 uv : TEXCOORD0;
             };
 
             struct output_t
             {
                 float4 pos : SV_POSITION;
                 half2 uv : TEXCOORD0;
             };
 
 
             output_t vert(input_t i)
             {
                 output_t o;
                 o.pos = UnityObjectToClipPos(i.pos);
                 o.uv = MultiplyUV(UNITY_MATRIX_TEXTURE0, i.uv);
                 return o;
             }

            float3 UnitToColor24(in float unit) {
                
                const float3 factor = float3(1, 255, 65025);
                const float mask = 1.0 / 256.0;
                float3 color = unit * factor.rgb;
                color.gb = frac(color.gb);
                color.rg -= color.gb * mask;
                return saturate(color);
            }
             
             float4 frag(output_t o) : COLOR
             {
                float depth = UNITY_SAMPLE_DEPTH(tex2D(_CameraDepthTexture, o.uv));
                float3 packed_color = UnitToColor24(Linear01Depth(depth));
                return float4(packed_color.r, packed_color.g, packed_color.b, 0);
             }
             
             ENDCG
         }
     } 
 }
