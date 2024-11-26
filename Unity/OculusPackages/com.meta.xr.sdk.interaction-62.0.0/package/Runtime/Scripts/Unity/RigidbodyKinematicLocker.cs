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

namespace Oculus.Interaction
{
    /// <summary>
    /// Manages the kinematic state of a Rigidbody component attached to the GameObject.
    /// This class provides methods to lock and unlock the kinematic state of the Rigidbody,
    /// ensuring controlled changes when multiple actors want to modify the kinematic state.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class RigidbodyKinematicLocker : MonoBehaviour
    {
        private Rigidbody _rigidbody;

        private int _counter = 0;
        private bool _savedIsKinematicState = false;

        private void Awake()
        {
            _rigidbody = this.GetComponent<Rigidbody>();
        }


        /// <summary>
        /// Locks to true the kinematic state of the attached Rigidbody.
        /// This method increments an internal counter with each call and sets the Rigidbody to kinematic.
        /// It also saves the original kinematic state for restoration.
        /// </summary>
        public void LockKinematic()
        {
            if (_counter == 0)
            {
                _savedIsKinematicState = _rigidbody.isKinematic;
            }
            _counter++;
            _rigidbody.isKinematic = true;
        }

        /// <summary>
        /// Unlocks the kinematic state of the attached Rigidbody.
        /// This method decrements the internal counter. When the counter reaches zero,
        /// it restores the Rigidbody to its original kinematic state.
        /// </summary>
        public void UnlockKinematic()
        {
            if (_counter == 0)
            {
                Debug.LogError($"Too many calls to {nameof(UnlockKinematic)}." +
                    $"Expected calls to {nameof(LockKinematic)} to balance the kinematic state.", this);
                return;
            }
            _counter--;
            if (_counter == 0)
            {
                _rigidbody.isKinematic = _savedIsKinematicState;
            }
        }
    }

    /// <summary>
    /// Extension methods for Rigidbody to easily manage kinematic locking through the RigidbodyKinematicLocker component.
    /// </summary>
    public static class RigidbodyKinematicLockerExtension
    {
        /// <summary>
        /// Locks to true the kinematic state of the Rigidbody this method is called on.
        /// If the Rigidbody does not have a RigidbodyKinematicLocker component, it adds one.
        /// </summary>
        /// <param name="rigidbody">The Rigidbody to lock the kinematic state on.</param>
        public static void LockKinematic(this Rigidbody rigidbody)
        {
            if (!rigidbody.TryGetComponent(out RigidbodyKinematicLocker component))
            {
                component = rigidbody.gameObject.AddComponent<RigidbodyKinematicLocker>();
            }
            component.LockKinematic();
        }

        /// <summary>
        /// Unlocks the kinematic state of the Rigidbody this method is called on.
        /// If the Rigidbody does not have a RigidbodyKinematicLocker component, it logs an error.
        /// </summary>
        /// <param name="rigidbody">The Rigidbody to unlock the kinematic state on.</param>
        public static void UnlockKinematic(this Rigidbody rigidbody)
        {
            if (!rigidbody.TryGetComponent(out RigidbodyKinematicLocker component))
            {
                Debug.LogError($"Too many calls to {nameof(UnlockKinematic)}." +
                    $"Expected calls to {nameof(LockKinematic)} to balance the kinematic state.", rigidbody);
                return;
            }
            component.UnlockKinematic();
        }
    }
}
