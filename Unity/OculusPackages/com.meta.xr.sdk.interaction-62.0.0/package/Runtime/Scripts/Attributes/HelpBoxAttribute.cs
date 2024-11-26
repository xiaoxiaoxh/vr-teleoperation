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
using static Oculus.Interaction.ConditionalHideAttribute;

namespace Oculus.Interaction
{
    public class HelpBoxAttribute : PropertyAttribute
    {
        public enum MessageType
        {
            None = 0,
            Info = 1,
            Warning = 2,
            Error = 3
        }

        public string Message { get; private set; }
        public object Value { get; private set; }
        public MessageType Type { get; private set; }
        public DisplayMode Display { get; private set; } = DisplayMode.Always;

        public delegate bool HelpBoxCondition();


        public HelpBoxAttribute(string message)
        {
            Message = message;
            Type = MessageType.Info;
            Value = null;
            Display = DisplayMode.Always;
        }

        public HelpBoxAttribute(string message, MessageType type)
        {
            Message = message;
            Type = type;
            Value = null;
            Display = DisplayMode.Always;
        }

        public HelpBoxAttribute(string message, MessageType type, object value)
        {
            Message = message;
            Type = type;
            Value = value;
            Display = DisplayMode.ShowIfTrue;
        }

        public HelpBoxAttribute(string message, MessageType type, object value, DisplayMode display)
        {
            Message = message;
            Type = type;
            Value = value;
            Display = display;
        }
    }
}
