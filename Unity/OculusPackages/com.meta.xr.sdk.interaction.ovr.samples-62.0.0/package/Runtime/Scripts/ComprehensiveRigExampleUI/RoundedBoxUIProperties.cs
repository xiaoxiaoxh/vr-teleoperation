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

using System.Collections;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class RoundedBoxUIProperties : UIBehaviour, IMeshModifier
{
    private Image _image;
    public Vector4 borderRadius;

#if UNITY_EDITOR
    protected override void OnValidate()
    {
        if (_image == null)
        {
            _image = gameObject.GetComponent<Image>();
            if (_image == null) return;
        }
        _image.SetAllDirty();
    }
#endif

    protected override void OnEnable()
    {
        _image = gameObject.GetComponent<Image>();
    }
    protected override void OnDisable()
    {
        _image = null;
    }

    protected override void Start()
    {
        StartCoroutine(DelayVertexGeneration());
    }

    IEnumerator DelayVertexGeneration()
    {
        yield return new WaitForSeconds(0.1f);
        if (_image == null)
        {
            _image = gameObject.GetComponent<Image>();
            if (_image == null) yield break;
        }
        _image.SetAllDirty();
    }

    public void ModifyMesh(Mesh mesh)
    {
    }

    public void ModifyMesh(VertexHelper verts)
    {
        if (_image == null)
        {
            _image = gameObject.GetComponent<Image>();
            if (_image == null) return;
        }

        var rectTransform = (RectTransform)transform;
        var rect = rectTransform.rect;
        var offset = new Vector4(rect.x, rect.y, Mathf.Abs(rect.width), Mathf.Abs(rect.height));
        UIVertex vert = new UIVertex();

        for (int i = 0; i < verts.currentVertCount; i++)
        {
            verts.PopulateUIVertex(ref vert, i);
            var uv0 = vert.uv0;
            uv0.z = offset.z;
            uv0.w = offset.w;
            vert.uv0 = uv0;
            vert.uv1 = borderRadius * 0.5f;
            verts.SetUIVertex(vert, i);
        }
    }
}
