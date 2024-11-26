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

using UnityEngine;

namespace Oculus.Interaction.Samples
{
    /// <summary>
    /// It is not expected that typical users of the SlateWithManipulators prefab
    /// should need to either interact with or understand this script.
    ///
    /// This script maintains scale based on the world-space relationship between a
    /// "local anchor" hierarchically parented to the transform on which this
    /// script resides and a "parent anchor" which is a hierarchical sibling. This
    /// logic is quite particular to that exact hierarchical configuration and is thus
    /// intended specifically for use within the SlateWithManipulators prefab.
    ///
    /// The specific world-space relationship maintained depends on the scaling mode.
    /// The default is a two-dimensional scaling mode which attempts to maintain a
    /// constant world-space offset between the local and parent anchors. Explicitly,
    /// with the local anchor placed at the top left corner of the "slate" and the
    /// parent anchor set to the AnchorTopLeft of the SlateWithManipulators prefab,
    /// this script will rescale the local space such that the world-space relationship
    /// between the AnchorTopLeft (and the visual affordances attached to it) and the
    /// local anchor (which logically represents the top-left corner of the "slate"
    /// content) is kept constant. As this is a two-dimensional scaling mode, the Z
    /// scale or "depth" of the slate is scaled to maintain a constant world-space
    /// depth.
    ///
    /// The spactial logic for the three-dimensional scaling mode is based on that
    /// of the two-dimensional scaling mode, but altered to maintain the original scale
    /// ratio of the content. This, as the name implies, is intended for manipulating
    /// object scale in three dimensions without squashing and stretching the object
    /// along the scaling axes. To this end, the script rescales the local space to
    /// find the largest aspect-correct scaling which can fit within the 2D rectangle
    /// described by the logic of the two-dimensional scaling mode. In other words,
    /// the three-dimensional scaling mode looks at where the slate content would have
    /// been placed by the two-dimensional scaling mode, then scales its 3D content to
    /// be as large as possible (without squashing) while still fitting within that
    /// "slate" space.
    /// </summary>
    public class AnchoredWorldSpaceDistanceScaler : MonoBehaviour
    {
        [SerializeField]
        private Transform _parentAnchor;

        [SerializeField]
        private Transform _localAnchor;

        public enum ScalingMode
        {
            TwoDimensional,
            ThreeDimensional,
        };

        [SerializeField, Tooltip("Choose whether content should be scaled as two- or three-dimensional")]
        private ScalingMode _scalingMode = ScalingMode.TwoDimensional;

        private Vector3 _parentAnchorOffset;
        private Vector3 _originalLocalScale;
        private Vector3 _originalParentLocalScale;
        private Vector3 _originalCombinedScale;

        private void Start()
        {
            _parentAnchorOffset = _parentAnchor.InverseTransformPoint(_localAnchor.position);
            _originalLocalScale = this.transform.localScale;
            _originalParentLocalScale = this.transform.parent.localScale;
            _originalCombinedScale = Vector3.Scale(_originalParentLocalScale, _originalLocalScale);
        }

        private void LateUpdate()
        {
            var worldSpace = _parentAnchor.TransformPoint(_parentAnchorOffset);
            var targetPosition = this.transform.InverseTransformPoint(worldSpace);

            // Because _localAnchor is parented to transform and targetPosition is now in transform's space,
            // the new local scale can be seeded with the transform.localScale adjusted to move _localAnchor.localPosition
            // onto targetPosition.
            var newScale = this.transform.localScale;
            newScale.Scale(new Vector3(
                Mathf.Abs(_localAnchor.localPosition.x) < Mathf.Epsilon ? 1f : targetPosition.x / _localAnchor.localPosition.x,
                Mathf.Abs(_localAnchor.localPosition.y) < Mathf.Epsilon ? 1f : targetPosition.y / _localAnchor.localPosition.y,
                Mathf.Abs(_localAnchor.localPosition.z) < Mathf.Epsilon ? 1f : targetPosition.z / _localAnchor.localPosition.z));

            if (_scalingMode == ScalingMode.ThreeDimensional)
            {
                // Three-dimensional scaling uses local scale to "rectify" aspect changes induced by the scaling of
                // transform.parent as well as transform, so their scales are combined to assess how newScale should be
                // modified. This is theoretically similar to using transform.lossyScale, but (1) it avoids depending
                // on the lossy arithmetic of lossyScale by instead depending on the structure of the prefab (on which
                // this script already depends) and (2) it limits the scaling contract only to the prefab itself, meaning
                // the behavior will not fight attempts by users to add non-uniform scaling to higher-level transforms
                // should the user choose to do so.
                var combinedScale = Vector3.Scale(this.transform.parent.localScale, newScale);

                // Depending on which axis (X or Y) is "stretched" by the baseline scale calculation for newScale,
                // set the "stretched" axis and the Z axis to be aspect-correct relative to the scale of the "unstretched"
                // axis. For example, suppose an originally cubic (1, 1, 1) object were to have a newScale calculated above
                // such that combinedScale was (1.3, 1.6, 1). The correct 3D combined scale for this scenario would be
                // (1.3, 1.3, 1.3), which is aspect-correct at the largest size which fits within the rectangle described
                // by the 2D scaling calculations. The "stretched" axis in this case would be the Y axis, falling
                // into the second case of the following conditional. The scalar would then be calculated as 1.3, which
                // is the scalar required to turn the original combined scale into the new correct combined scale; and
                // the new correct local scale is that scaled original combined scale divided by the current parent local
                // scale.
                if (combinedScale.x / combinedScale.y > _originalCombinedScale.x / _originalCombinedScale.y)
                {
                    float scalar = combinedScale.y / _originalCombinedScale.y;
                    newScale.x = _originalCombinedScale.x * scalar / this.transform.parent.localScale.x;
                    newScale.z = _originalCombinedScale.z * scalar / this.transform.parent.localScale.z;
                }
                else
                {
                    float scalar = combinedScale.x / _originalCombinedScale.x;
                    newScale.y = _originalCombinedScale.y * scalar / this.transform.parent.localScale.y;
                    newScale.z = _originalCombinedScale.z * scalar / this.transform.parent.localScale.z;
                }
            }
            else
            {
                newScale.z = _originalParentLocalScale.z * _originalLocalScale.z / this.transform.parent.localScale.z;
            }

            this.transform.localScale = newScale;
        }
    }
}
