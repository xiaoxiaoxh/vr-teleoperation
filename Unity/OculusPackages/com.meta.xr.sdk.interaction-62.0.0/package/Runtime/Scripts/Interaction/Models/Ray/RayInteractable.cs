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

using Oculus.Interaction.Surfaces;
using UnityEngine;

namespace Oculus.Interaction
{
    public class RayInteractable : PointerInteractable<RayInteractor, RayInteractable>
    {
        /// <summary>
        /// The mesh used as the interactive surface for the ray.
        /// </summary>
        [Tooltip("The mesh used as the interactive surface for the ray.")]
        [SerializeField, Interface(typeof(ISurface))]
        private UnityEngine.Object _surface;
        public ISurface Surface { get; private set; }

        /// <summary>
        /// Defines the boundaries of the raycast. All <cref="RayInteractable" />s must be inside this surface for the raycast to reach them.
        /// </summary>
        [Tooltip("Defines the boundaries of the raycast. All RayInteractables must be inside this surface for the raycast to reach them.")]
        [SerializeField, Optional, Interface(typeof(ISurface))]
        private UnityEngine.Object _selectSurface = null;
        private ISurface SelectSurface;

        /// <summary>
        /// An <cref="IMovementProvider" /> that determines how the interactable moves when selected.
        /// </summary>
        [Tooltip("An IMovementProvider that determines how the interactable moves when selected.")]
        [SerializeField, Optional, Interface(typeof(IMovementProvider))]
        private UnityEngine.Object _movementProvider;
        private IMovementProvider MovementProvider { get; set; }

        /// <summary>
        /// The score used when comparing two interactables to determine which one should be selected.
        /// Each interactable has its own score, and the highest scoring interactable will be selected.
        /// </summary>
        [Tooltip("The score used when comparing two interactables to determine which one should be selected. " +
        "Each interactable has its own score, and the highest scoring interactable will be selected.")]
        [SerializeField, Optional]
        private int _tiebreakerScore = 0;

        #region Properties
        public int TiebreakerScore
        {
            get
            {
                return _tiebreakerScore;
            }
            set
            {
                _tiebreakerScore = value;
            }
        }
        #endregion

        protected override void Awake()
        {
            base.Awake();
            Surface = _surface as ISurface;
            SelectSurface = _selectSurface as ISurface;
            MovementProvider = _movementProvider as IMovementProvider;
        }

        protected override void Start()
        {
            this.BeginStart(ref _started, () => base.Start());
            this.AssertField(Surface, nameof(Surface));
            if (_selectSurface != null)
            {
                this.AssertField(SelectSurface, nameof(SelectSurface));
            }
            else
            {
                SelectSurface = Surface;
                _selectSurface = SelectSurface as MonoBehaviour;
            }
            this.EndStart(ref _started);
        }

        /// <summary>
        /// Called when the interactor has a set of interactables and is calculating which one is closest.
        /// </summary>
        /// <param name="ray">The position and direction of the ray.</param>
        /// <param name="hit">The location, normal, and distance of the ray hit.</param>
        /// <param name="maxDistance">The maximum ray length.</param>
        /// <param name="selectSurface">True if the raycast has hit the selectable part of the <cref="RayInteractable" />, otherwise false.</param>
        /// <returns>Returns true if hit.</returns>
        public bool Raycast(Ray ray, out SurfaceHit hit, in float maxDistance, bool selectSurface)
        {
            hit = new SurfaceHit();
            ISurface surface = selectSurface ? SelectSurface : Surface;
            return surface.Raycast(ray, out hit, maxDistance);
        }

        /// <summary>
        /// Generates movement to move the <cref="RayInteractable" /> from its current position to the target position.
        /// </summary>
        /// <param name="to">The target position.</param>
        /// <param name="source">The current position.</param>
        /// <returns>Returns the movement that will be applied to the interactable.</returns>
        public IMovement GenerateMovement(in Pose to, in Pose source)
        {
            if (MovementProvider == null)
            {
                return null;
            }
            IMovement movement = MovementProvider.CreateMovement();
            movement.StopAndSetPose(source);
            movement.MoveTo(to);
            return movement;
        }

        #region Inject

        /// <summary>
        /// Sets all required values for a <cref="RayInteractable" /> on a dynamically instantiated GameObject.
        /// </summary>
        public void InjectAllRayInteractable(ISurface surface)
        {
            InjectSurface(surface);
        }

        /// <summary>
        /// Sets a surface for a dynamically instantiated GameObject.
        /// </summary>
        public void InjectSurface(ISurface surface)
        {
            Surface = surface;
            _surface = surface as UnityEngine.Object;
        }

        /// <summary>
        /// Sets a select surface for a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalSelectSurface(ISurface surface)
        {
            SelectSurface = surface;
            _selectSurface = surface as UnityEngine.Object;
        }

        /// <summary>
        /// Sets a movement provider for a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalMovementProvider(IMovementProvider provider)
        {
            _movementProvider = provider as UnityEngine.Object;
            MovementProvider = provider;
        }

        #endregion
    }
}
