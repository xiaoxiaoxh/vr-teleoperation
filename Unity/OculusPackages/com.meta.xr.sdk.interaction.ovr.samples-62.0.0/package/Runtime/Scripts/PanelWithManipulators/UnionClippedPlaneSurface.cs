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
using System.Collections.Generic;
using System.Linq;

namespace Oculus.Interaction.Surfaces
{
    public class UnionClippedPlaneSurface : MonoBehaviour, IClippedSurface<IBoundsClipper>
    {
        private static readonly Bounds InfiniteBounds = new Bounds(Vector3.zero,
            Vector3.one * float.PositiveInfinity);

        private static readonly Bounds PlaneBounds = new Bounds(Vector3.zero,
            new Vector3(float.PositiveInfinity, float.PositiveInfinity, Vector3.kEpsilon));

        [Tooltip("The Plane Surface to be clipped.")]
        [SerializeField]
        private PlaneSurface _planeSurface;

        [Tooltip("The clippers that will be used to clip the Plane Surface.")]
        [SerializeField, Interface(typeof(IBoundsClipper))]
        private List<UnityEngine.Object> _clippers = new List<UnityEngine.Object>();
        private List<IBoundsClipper> Clippers { get; set; }

        public ISurface BackingSurface => _planeSurface;

        public Transform Transform => _planeSurface.Transform;

        public IReadOnlyList<IBoundsClipper> GetClippers()
        {
            if (Clippers != null)
            {
                return Clippers;
            }
            else
            {
                return _clippers.ConvertAll(clipper => clipper as IBoundsClipper);
            }
        }

        protected virtual void Awake()
        {
            Clippers = _clippers.ConvertAll(clipper => clipper as IBoundsClipper);
        }

        protected virtual void Start()
        {
            this.AssertField(_planeSurface, nameof(_planeSurface));
            this.AssertCollectionItems(Clippers, nameof(Clippers));
        }

        public List<Bounds> GetLocalBounds()
        {
            var clipBounds = new List<Bounds>();

            IReadOnlyList<IBoundsClipper> clippers = GetClippers();
            for (int i = 0; i < clippers.Count; i++)
            {
                IBoundsClipper clipper = clippers[i];
                if (clipper == null ||
                    !clipper.GetLocalBounds(Transform, out Bounds clipTo))
                {
                    continue;
                }

                clipBounds.Add(clipTo);
            }

            return clipBounds;
        }

        private Vector3 ClampPoint(in Vector3 point, in Bounds bounds)
        {
            Vector3 min = bounds.min;
            Vector3 max = bounds.max;
            Vector3 localPoint = Transform.InverseTransformPoint(point);

            Vector3 clamped = new Vector3(
                Mathf.Clamp(localPoint.x, min.x, max.x),
                Mathf.Clamp(localPoint.y, min.y, max.y),
                Mathf.Clamp(localPoint.z, min.z, max.z));

            return Transform.TransformPoint(clamped);
        }

        public bool ClosestSurfacePoint(in Vector3 point, out SurfaceHit hit, float maxDistance = 0)
        {
            if (!_planeSurface.ClosestSurfacePoint(point, out hit, maxDistance)) return false;
            //Vector3.kEpsilon
            var boundsList = GetLocalBounds();
            var points = new List<(Vector3, float)>();
            foreach (var bounds in boundsList)
            {
                var bound = bounds;
                var size = bound.size;
                size.z = Vector3.kEpsilon;
                bound.size = size;

                var clampPoint = ClampPoint(hit.Point, bound);
                var distance = Vector3.Distance(point, clampPoint);

                if(maxDistance <= 0 || distance <= maxDistance)
                {
                    points.Add((clampPoint, distance));
                }
            }
            if (points.Count == 0) return false;

            var minPoint = points[0];
            for (var i = 1; i < points.Count; i++)
            {
                var p = points[i];
                if(p.Item2 < minPoint.Item2)
                {
                    minPoint = p;
                }
            }
            hit.Point = minPoint.Item1;
            hit.Distance = minPoint.Item2;
            return true;
        }

        public bool Raycast(in Ray ray, out SurfaceHit hit, float maxDistance = 0)
        {
            if (BackingSurface.Raycast(ray, out hit, maxDistance))
            {
                var bounds = GetLocalBounds();
                foreach(var bound in bounds)
                {
                    if (bound.Contains(Transform.InverseTransformPoint(hit.Point)))
                    {
                        return true;
                    }
                }
                return false;
            }
            return false;
        }

        #region Inject

        public void InjectAllClippedPlaneSurface(
            PlaneSurface planeSurface,
            IEnumerable<IBoundsClipper> clippers)
        {
            InjectPlaneSurface(planeSurface);
            InjectClippers(clippers);
        }

        public void InjectPlaneSurface(PlaneSurface planeSurface)
        {
            _planeSurface = planeSurface;
        }

        public void InjectClippers(IEnumerable<IBoundsClipper> clippers)
        {
            _clippers = new List<UnityEngine.Object>(
                clippers.Select(c => c as UnityEngine.Object));
            Clippers = clippers.ToList();
        }

        #endregion
    }
}
