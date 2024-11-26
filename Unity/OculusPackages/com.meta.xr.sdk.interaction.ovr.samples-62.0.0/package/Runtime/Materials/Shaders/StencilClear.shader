/************************************************************************************
Copyright : Copyright (c) Facebook Technologies, LLC and its affiliates. All rights reserved.

Your use of this SDK or tool is subject to the Oculus SDK License Agreement, available at
https://developer.oculus.com/licenses/oculussdk/

Unless required by applicable law or agreed to in writing, the Utilities SDK distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF
ANY KIND, either express or implied. See the License for the specific language governing
permissions and limitations under the License.
************************************************************************************/

Shader "StencilClear"
{
	Properties {}
	SubShader
	{
		Tags{ "RenderType" = "Opaque"  "Queue" = "Geometry+502"}
		LOD 100

		CGINCLUDE
		#pragma target 3.0
		ENDCG

		Pass
		{
			Name "StencilClear"
			ColorMask 0
			Stencil
			{
				Ref 0
				Comp Always
				Pass Replace
				ReadMask 255
				WriteMask 255
			}
		}
	}
}
