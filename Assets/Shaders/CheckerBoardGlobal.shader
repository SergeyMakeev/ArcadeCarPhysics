// Upgrade NOTE: replaced '_Object2World' with 'unity_ObjectToWorld'

Shader "ArcadeCarPhysics/CheckerBoardGlobal" {
	Properties {

		_MainTex ("Albedo (RGB)", 2D) = "white" {}
		_Glossiness ("Smoothness", Range(0,1)) = 0.0
		_Metallic ("Metallic", Range(0,1)) = 0.0
		_UvScale("UV Scale", Range(0.1,10)) = 1.0
		_Color("Color", Color) = (1,1,1,1)
	}
	SubShader {
		Tags { "RenderType"="Opaque" }
		LOD 200

		CGPROGRAM
		// Physically based Standard lighting model, and enable shadows on all light types
		#pragma surface surf Standard fullforwardshadows vertex:vert

		// Use shader model 3.0 target, to get nicer looking lighting
		#pragma target 3.0

		sampler2D _MainTex;

		struct Input {
			float3 worldPosition;
			float3 worldNormal;
		};

		half _Glossiness;
		half _Metallic;
		half _UvScale;
		half4 _Color;

		// Add instancing support for this shader. You need to check 'Enable Instancing' on materials that use the shader.
		// See https://docs.unity3d.com/Manual/GPUInstancing.html for more information about instancing.
		// #pragma instancing_options assumeuniformscaling
		UNITY_INSTANCING_BUFFER_START(Props)
			// put more per-instance properties here
		UNITY_INSTANCING_BUFFER_END(Props)


		void vert(inout appdata_full v, out Input data)
		{
			UNITY_INITIALIZE_OUTPUT(Input, data);
			data.worldPosition = mul(unity_ObjectToWorld, float4(v.vertex.xyz, 1.0)).xyz;;
			data.worldNormal = UnityObjectToWorldNormal(v.normal.xyz);
		}

		void surf (Input IN, inout SurfaceOutputStandard o) {

			float3 bf = normalize(abs(IN.worldNormal));
			bf /= dot(bf, (float3)1);

			float2 tx = IN.worldPosition.yz * _UvScale;
			float2 ty = IN.worldPosition.zx * _UvScale;
			float2 tz = IN.worldPosition.xy * _UvScale;

			half4 cx = tex2D(_MainTex, tx) * bf.x;
			half4 cy = tex2D(_MainTex, ty) * bf.y;
			half4 cz = tex2D(_MainTex, tz) * bf.z;

			o.Albedo = (cx + cy + cz).rgb * _Color.rgb;
			o.Metallic = _Metallic;
			o.Smoothness = _Glossiness;
			o.Alpha = 1.0;
		}
		ENDCG
	}
	FallBack "Diffuse"
}
