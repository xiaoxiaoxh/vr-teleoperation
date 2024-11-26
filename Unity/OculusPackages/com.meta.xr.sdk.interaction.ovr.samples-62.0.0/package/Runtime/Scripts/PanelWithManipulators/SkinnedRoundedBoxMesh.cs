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

using Oculus.Interaction;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SkinnedRoundedBoxMesh : MonoBehaviour
{
    [SerializeField]
    private Transform _topLeft;
    [SerializeField]
    private Transform _topRight;
    [SerializeField]
    private Transform _bottomLeft;
    [SerializeField]
    private Transform _bottomRight;
    [SerializeField]
    private int _cornerSegmentCount;
    [SerializeField]
    private int _cylinderFaceCount;
    [SerializeField]
    private float _borderRadius;
    [SerializeField]
    private float _cylinderRadius;
    [SerializeField]
    private SkinnedMeshRenderer _skinnedMeshRenderer;

    public bool generateOnStart;

    private void Start()
    {
        if (generateOnStart)
        {
            GenerateMesh(_cornerSegmentCount, _borderRadius, _cylinderFaceCount, _cylinderRadius, _skinnedMeshRenderer, _topLeft, _topRight, _bottomLeft, _bottomRight);
        }
    }

    public static Vector2[] GenerateArcPath(float startAngle, float endAngle, int steps, float radius, bool closed)
    {
        var segmentAngle = (endAngle - startAngle) / steps;
        var vertices = new List<Vector2>();
        var s = closed ? steps + 1 : steps;
        for (int i = 0; i < s; i++)
        {
            var vert = new Vector2(Mathf.Cos(startAngle + segmentAngle * i), Mathf.Sin(startAngle + segmentAngle * i));
            vertices.Add(vert * radius);
        }
        return vertices.ToArray();
    }

    public static Vector3[] GenerateCylinderAroundPath(List<Vector2> path, int cylinderFaceCount, float cylinderRadius)
    {
        var ringPoints = GenerateArcPath(Mathf.Deg2Rad * 0.0f, Mathf.Deg2Rad * 360.0f, cylinderFaceCount, cylinderRadius, false);
        var rings = new List<Vector3>();

        //Generate Cylinder
        for (int i = 0; i < path.Count; i++)
        {
            var currentVert = path[i];
            var nextVert = path[(i + 1) % path.Count];

            var forward = Vector3.Normalize(nextVert - currentVert);
            var right = Vector3.Normalize(Vector3.Cross(forward, new Vector3(0.0f, 0.0f, 1.0f)));

            for (int j = 0; j < ringPoints.Length; j++)
            {
                var ringPoint = (Vector3)currentVert + (ringPoints[j].x * right) + (ringPoints[j].y * Vector3.forward);
                rings.Add(ringPoint);
            }
        }

        return rings.ToArray();
    }

    public static int[] GenerateCylinderIndices(int cornerSegmentCount, int cylinderFaceCount)
    {
        var meshIndices = new List<int>();
        var sectionCount = cornerSegmentCount * 4; /*_arcSegmentCount*/
        for (int i = 0; i < sectionCount; i++)
        {
            var currentSectionStart = cylinderFaceCount * i; /*_ringSegmentCount*/
            var nextSectionStart = cylinderFaceCount * ((i + 1) % sectionCount);

            for (int j = 0; j < cylinderFaceCount; j++)
            {
                var current = j;
                var next = (j + 1) % cylinderFaceCount;

                //Quad

                //Tri0
                meshIndices.Add(currentSectionStart + current);
                meshIndices.Add(nextSectionStart + current);
                meshIndices.Add(nextSectionStart + next);
                //Tri1
                meshIndices.Add(currentSectionStart + current);
                meshIndices.Add(nextSectionStart + next);
                meshIndices.Add(currentSectionStart + next);
            }
        }
        return meshIndices.ToArray();
    }

    private static void PushBoneWeigth(int boneIndex, List<BoneWeight> weights, int cornerSegmentCount, int cylinderFaceCount)
    {
        var segmentVertCount = cornerSegmentCount * cylinderFaceCount;
        for (int i = 0; i < segmentVertCount; i++)
        {
            var weight = new BoneWeight();
            weight.boneIndex0 = boneIndex;
            weight.weight0 = 1.0f;
            weight.boneIndex1 = 0;
            weight.boneIndex2 = 0;
            weight.boneIndex3 = 0;
            weights.Add(weight);
        }
    }

    public static void GenerateMesh(
        int cornerSegmentCount, float borderRadius, int cylinderFaceCount, float cylinderRadius,
        SkinnedMeshRenderer skinnedMeshRenderer, Transform topLeft, Transform topRight, Transform bottomLeft, Transform bottomRight)
    {
        var mesh = new Mesh();
        var pathList = new List<Vector2>();

        pathList.AddRange(GenerateArcPath(Mathf.Deg2Rad * 180.0f, Mathf.Deg2Rad * 90.0f, cornerSegmentCount, borderRadius, false));
        pathList.AddRange(GenerateArcPath(Mathf.Deg2Rad * 90.0f, Mathf.Deg2Rad * 0.0f, cornerSegmentCount, borderRadius, false));
        pathList.AddRange(GenerateArcPath(Mathf.Deg2Rad * 0.0f, Mathf.Deg2Rad * -90.0f, cornerSegmentCount, borderRadius, false));
        pathList.AddRange(GenerateArcPath(Mathf.Deg2Rad * -90.0f, Mathf.Deg2Rad * -180.0f, cornerSegmentCount, borderRadius, false));

        var meshVertices = GenerateCylinderAroundPath(pathList, cylinderFaceCount, cylinderRadius);
        var meshIndices = GenerateCylinderIndices(cornerSegmentCount, cylinderFaceCount);

        var boneWeigths = new List<BoneWeight>();
        PushBoneWeigth(0, boneWeigths, cornerSegmentCount, cylinderFaceCount);
        PushBoneWeigth(1, boneWeigths, cornerSegmentCount, cylinderFaceCount);
        PushBoneWeigth(3, boneWeigths, cornerSegmentCount, cylinderFaceCount);
        PushBoneWeigth(2, boneWeigths, cornerSegmentCount, cylinderFaceCount);

        mesh.vertices = meshVertices;
        mesh.SetIndices(meshIndices, MeshTopology.Triangles, 0);
        mesh.boneWeights = boneWeigths.ToArray();
        mesh.bindposes = new Matrix4x4[]
        {
            Matrix4x4.identity,
            Matrix4x4.identity,
            Matrix4x4.identity,
            Matrix4x4.identity
        };
        mesh.name = "SkinnedRoundedBoxMesh";

        skinnedMeshRenderer.bones = new Transform[]
        {
            topLeft,topRight,
            bottomLeft,bottomRight
        };
        skinnedMeshRenderer.rootBone = topLeft;
        skinnedMeshRenderer.sharedMesh = mesh;
        mesh.RecalculateBounds();
    }

    [ContextMenu("Generate Mesh")]
    public void GenerateMeshFromMenu()
    {
        GenerateMesh(_cornerSegmentCount, _borderRadius, _cylinderFaceCount, _cylinderRadius, _skinnedMeshRenderer, _topLeft, _topRight, _bottomLeft, _bottomRight);
    }
}
