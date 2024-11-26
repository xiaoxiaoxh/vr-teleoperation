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

using Oculus.Interaction.HandGrab;
using UnityEngine;

namespace Oculus.Interaction.Samples
{
    public class SlingshotPellet : MonoBehaviour
    {
        [SerializeField]
        private PhysicsGrabbable _physicsGrabbable;
        [SerializeField]
        private Grabbable grabbable;

        [SerializeField]
        private HandGrabInteractable[] _handGrabInteractables;

        private HandGrabInteractor _lastHandGrabInteractor;

        public HandGrabInteractor HandGrabber => _lastHandGrabInteractor;

        private UniqueIdentifier Identifier = UniqueIdentifier.Generate();

        private void OnEnable()
        {
            foreach (var handGrabInteractable in _handGrabInteractables)
            {
                handGrabInteractable.WhenSelectingInteractorAdded.Action += HandleSelectingHandGrabInteractorAdded;
            }
        }

        private void OnDisable()
        {
            foreach (var handGrabInteractable in _handGrabInteractables)
            {
                handGrabInteractable.WhenSelectingInteractorAdded.Action -= HandleSelectingHandGrabInteractorAdded;
            }
        }

        private void HandleSelectingHandGrabInteractorAdded(HandGrabInteractor interactor)
        {
            _lastHandGrabInteractor = interactor;
        }

        public void Attach()
        {
            Pose selfPose = this.transform.GetPose();
            grabbable.ProcessPointerEvent(new PointerEvent(Identifier.ID, PointerEventType.Hover, selfPose));
            grabbable.ProcessPointerEvent(new PointerEvent(Identifier.ID, PointerEventType.Select, selfPose));
            grabbable.ProcessPointerEvent(new PointerEvent(Identifier.ID, PointerEventType.Move, selfPose));
        }


        public void Move(Transform transform)
        {
            grabbable.ProcessPointerEvent(new PointerEvent(Identifier.ID, PointerEventType.Move, transform.GetPose()));
        }

        public void Eject(Vector3 force)
        {
            grabbable.ProcessPointerEvent(new PointerEvent(Identifier.ID, PointerEventType.Cancel, this.transform.GetPose()));

            _physicsGrabbable.ApplyVelocities(force, Vector3.zero);
        }

    }
}
