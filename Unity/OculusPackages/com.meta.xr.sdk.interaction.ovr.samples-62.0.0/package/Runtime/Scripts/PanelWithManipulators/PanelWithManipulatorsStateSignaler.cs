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

using System;
using UnityEngine;

namespace Oculus.Interaction.Samples
{
    /// <summary>
    /// It is not expected that typical users of the SlateWithManipulators prefab
    /// should need to either interact with or understand this script.
    ///
    /// This script centralizes state for the SlateWithManipulators prefab, allowing the
    /// various elements of the prefab to participate in statefulness without taking any
    /// direct dependencies on one another. Canonically, all interactable elements within
    /// the prefab will take references to this signaler, which they will use both to
    /// subscribe to the WhenStateChanged event and to set the state when appropriate.
    /// In this way, when one interactable element is selected, it can set the state and
    /// thereby notify other interested elements that the prefab as a whole, in a sense,
    /// has been selected; the other elements can then respond to this signal as
    /// appropriate, typically by becoming disabled for the duration of the selection.
    /// This signaler can also be used by external logic to set state for the entire
    /// prefab, for example to set the prefab to Idle when no interactors are nearby,
    /// though this is not part of the prefab's default behavior.
    /// </summary>
    public class PanelWithManipulatorsStateSignaler : MonoBehaviour
    {
        public enum State
        {
            Default,
            Selected,
            Idle
        }

        public event Action<State> WhenStateChanged = (State newState) => { };

        private State _state = State.Default;
        public State CurrentState
        {
            get
            {
                return _state;
            }
            set
            {
                if (value != _state)
                {
                    _state = value;
                    WhenStateChanged(_state);
                }
            }
        }
    }
}
