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

using Oculus.Interaction.Input;
using System.Collections.Generic;
using UnityEngine;

namespace Oculus.Interaction
{
    /// <summary>
    /// HandFingerMaskGenerator creates a finger mask from the hand model currently used (check diff that added this script)
    /// It projects the hand model into a 2d space using the object space positions of the hand model
    /// and line positions using the hand visual Joint Poses.
    /// </summary>
    public class HandFingerMaskGenerator
    {
        private static readonly int[] _fingerLinesID =
        {
            Shader.PropertyToID("_ThumbLine"),
            Shader.PropertyToID("_IndexLine"),
            Shader.PropertyToID("_MiddleLine"),
            Shader.PropertyToID("_RingLine"),
            Shader.PropertyToID("_PinkyLine")
        };

        private static readonly int[] _palmFingerLinesID =
        {
            Shader.PropertyToID("_PalmThumbLine"),
            Shader.PropertyToID("_PalmIndexLine"),
            Shader.PropertyToID("_PalmMiddleLine"),
            Shader.PropertyToID("_PalmRingLine"),
            Shader.PropertyToID("_PalmPinkyLine")
        };

        private static float HandednessMultiplier(Handedness hand)
        {
            return hand != Handedness.Right ? -1.0f : 1.0f;
        }

        private static List<Vector2> GenerateModelUV(Handedness handedness, Mesh sharedHandMesh,
            out Vector2 minPosition,
            out Vector2 maxPosition)
        {
            List<Vector3> vertices = new List<Vector3>();
            sharedHandMesh.GetVertices(vertices);
            minPosition = new Vector2(vertices[0].x, vertices[0].z);
            maxPosition = new Vector2(vertices[0].x, vertices[0].z);
            for (int i = 0; i < vertices.Count; i++)
            {
                var vertex = vertices[i] * HandednessMultiplier(handedness);
                var vertex2d = new Vector2(vertex.x, vertex.z);
                minPosition = Vector2.Min(minPosition, vertex2d);
                maxPosition = Vector2.Max(maxPosition, vertex2d);
                vertices[i] = vertex;
            }

            List<Vector2> uvs = new List<Vector2>();
            Vector2 regionSize = maxPosition - minPosition;
            float maxLength = Mathf.Max(regionSize.x, regionSize.y);
            foreach (var vertex in vertices)
            {
                var vertex2d = new Vector2(vertex.x, vertex.z);
                var vertexUV = (vertex2d - minPosition) / maxLength;
                uvs.Add(vertexUV);
            }

            return uvs;
        }

        private static Vector2 GetPositionOnRegion(HandVisual handVisual, HandJointId jointId,
            Vector2 minRegion, float sideLength)
        {
            IHand hand = handVisual.Hand;

            Pose lineStartPose = handVisual.Joints[(int)jointId].GetPose(Space.World);
            Vector3 lineStartPosition = handVisual.Root.InverseTransformPoint(lineStartPose.position);

            Vector2 point = new Vector2(lineStartPosition.x, lineStartPosition.z);
            point *= HandednessMultiplier(hand.Handedness);
            return (point - minRegion) / sideLength;
        }

        private static Vector4[] GenerateFingerLines(HandVisual handVisual, Vector2 minPosition,
            float maxLength, float[] lineScale)
        {
            Vector4 thumbLine = GenerateLineData(handVisual, HandJointId.HandThumbTip,
                HandJointId.HandThumb1, minPosition, maxLength, lineScale[0]);
            Vector4 indexLine = GenerateLineData(handVisual, HandJointId.HandIndexTip,
                HandJointId.HandIndex1, minPosition, maxLength, lineScale[1]);
            Vector4 middleLine = GenerateLineData(handVisual, HandJointId.HandMiddleTip,
                HandJointId.HandMiddle1, minPosition, maxLength, lineScale[2]);
            Vector4 ringLine = GenerateLineData(handVisual, HandJointId.HandRingTip,
                HandJointId.HandRing1, minPosition, maxLength, lineScale[3]);
            Vector4 pinkyLine = GenerateLineData(handVisual, HandJointId.HandPinkyTip,
                HandJointId.HandPinky1, minPosition, maxLength, lineScale[4]);

            return new Vector4[Constants.NUM_FINGERS] { thumbLine, indexLine, middleLine, ringLine, pinkyLine };
        }

        private static Vector4 GenerateLineData(HandVisual handVisual, HandJointId jointIdStart,
            HandJointId jointIdEnd,
            Vector2 minRegion, float sideLength, float lineScale)
        {
            Vector2 startPosition =
                GetPositionOnRegion(handVisual, jointIdStart, minRegion, sideLength);
            Vector2 endPosition =
                GetPositionOnRegion(handVisual, jointIdEnd, minRegion, sideLength);
            endPosition = Vector2.LerpUnclamped(startPosition, endPosition, lineScale);
            return new Vector4(startPosition.x, startPosition.y, endPosition.x, endPosition.y);
        }

        private static void SetGlowModelUV(SkinnedMeshRenderer handRenderer, Handedness handedness,
            out Vector2 minPosition, out Vector2 maxPosition)
        {
            Mesh sharedHandMesh = handRenderer.sharedMesh;

            List<Vector2> uvs = GenerateModelUV(handedness, sharedHandMesh,
                out minPosition,
                out maxPosition);

            sharedHandMesh.SetUVs(1, uvs);
            sharedHandMesh.UploadMeshData(false);
        }

        private static void SetFingerMaskUniforms(HandVisual handVisual, MaterialPropertyBlock materialPropertyBlock, Vector2 minPosition, Vector2 maxPosition)
        {
            Vector2 regionSize = maxPosition - minPosition;
            float maxLength = Mathf.Max(regionSize.x, regionSize.y);

            //The following numbers generate good looking lines for the current hand model
            var fingerLineScales = new float[Constants.NUM_FINGERS] { 0.9f, 0.91f, 0.9f, 0.87f, 0.87f };
            var fingerLines = GenerateFingerLines(handVisual, minPosition, maxLength, fingerLineScales);
            //The thumb line is going to be perpendicularly aligned to the wrist direction
            fingerLines[0].z = Mathf.Lerp(fingerLines[0].z, fingerLines[0].x, 0.3f);
            fingerLines[0].x = fingerLines[0].z;

            var palmFingerLineScales = new float[Constants.NUM_FINGERS] { 1.2f, 1.25f, 1.25f, 1.25f, 1.25f };
            var palmFingerLines =
                GenerateFingerLines(handVisual, minPosition, maxLength, palmFingerLineScales);
            //The thumb line is going to be perpendicularly aligned to the wrist direction
            float thumbOffset = Mathf.Abs(palmFingerLines[0].x - palmFingerLines[0].z) * 0.1f;
            palmFingerLines[0].z += thumbOffset;

            for (int i = 0; i < Constants.NUM_FINGERS; i++)
            {
                materialPropertyBlock.SetVector(_fingerLinesID[i], fingerLines[i]);
                materialPropertyBlock.SetVector(_palmFingerLinesID[i], palmFingerLines[i]);
            }
        }

        public static void GenerateFingerMask(SkinnedMeshRenderer handRenderer, HandVisual handVisual, MaterialPropertyBlock materialPropertyBlock)
        {
            SetGlowModelUV(handRenderer, handVisual.Hand.Handedness,
                out Vector2 minPosition, out Vector2 maxPosition);
            SetFingerMaskUniforms(handVisual, materialPropertyBlock, minPosition, maxPosition);
        }
    }
}
