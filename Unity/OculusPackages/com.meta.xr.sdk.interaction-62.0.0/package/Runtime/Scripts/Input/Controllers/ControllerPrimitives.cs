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

namespace Oculus.Interaction.Input
{
    // Enum containing all values of Unity.XR.CommonUsage.
    [Flags]
    public enum ControllerButtonUsage
    {
        None = 0,
        PrimaryButton = 1 << 0,
        PrimaryTouch = 1 << 1,
        SecondaryButton = 1 << 2,
        SecondaryTouch = 1 << 3,
        GripButton = 1 << 4,
        TriggerButton = 1 << 5,
        MenuButton = 1 << 6,
        Primary2DAxisClick = 1 << 7,
        Primary2DAxisTouch = 1 << 8,
        Thumbrest = 1 << 9,
    }

    [Flags]
    public enum ControllerAxis1DUsage
    {
        None = 0,
        Trigger = 1 << 0,
        Grip = 1 << 1,
    }

    [Flags]
    public enum ControllerAxis2DUsage
    {
        None = 0,
        Primary2DAxis = 1 << 0,
        Secondary2DAxis = 1 << 1
    }

    public struct ControllerInput
    {
        public ControllerButtonUsage ButtonUsageMask { get; private set; }

        public bool PrimaryButton => (ButtonUsageMask & ControllerButtonUsage.PrimaryButton) != 0;
        public bool PrimaryTouch => (ButtonUsageMask & ControllerButtonUsage.PrimaryTouch) != 0;
        public bool SecondaryButton => (ButtonUsageMask & ControllerButtonUsage.SecondaryButton) != 0;
        public bool SecondaryTouch => (ButtonUsageMask & ControllerButtonUsage.SecondaryTouch) != 0;
        public bool GripButton => (ButtonUsageMask & ControllerButtonUsage.GripButton) != 0;
        public bool TriggerButton => (ButtonUsageMask & ControllerButtonUsage.TriggerButton) != 0;
        public bool MenuButton => (ButtonUsageMask & ControllerButtonUsage.MenuButton) != 0;
        public bool Primary2DAxisClick => (ButtonUsageMask & ControllerButtonUsage.Primary2DAxisClick) != 0;
        public bool Primary2DAxisTouch => (ButtonUsageMask & ControllerButtonUsage.Primary2DAxisTouch) != 0;
        public bool Thumbrest => (ButtonUsageMask & ControllerButtonUsage.Thumbrest) != 0;

        public float Trigger { get; private set; }
        public float Grip { get; private set; }

        public Vector2 Primary2DAxis { get; private set; }
        public Vector2 Secondary2DAxis { get; private set; }

        public void Clear()
        {
            ButtonUsageMask = ControllerButtonUsage.None;
            Trigger = 0f;
            Grip = 0f;
            Primary2DAxis = Vector2.zero;
            Secondary2DAxis = Vector2.zero;
        }

        public void SetButton(ControllerButtonUsage usage, bool value)
        {
            if (value)
            {
                ButtonUsageMask |= usage;
            }
            else
            {
                ButtonUsageMask &= ~usage;
            }
        }

        public void SetAxis1D(ControllerAxis1DUsage usage, float value)
        {
            switch (usage)
            {
                case ControllerAxis1DUsage.Trigger:
                    Trigger = value; break;
                case ControllerAxis1DUsage.Grip:
                    Grip = value; break;
            }
        }

        public void SetAxis2D(ControllerAxis2DUsage usage, Vector2 value)
        {
            switch (usage)
            {
                case ControllerAxis2DUsage.Primary2DAxis:
                    Primary2DAxis = value; break;
                case ControllerAxis2DUsage.Secondary2DAxis:
                    Secondary2DAxis = value; break;
            }
        }

    }
}
