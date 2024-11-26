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
using System.Collections.Generic;
using UnityEngine;

namespace Meta.XR.Editor.Tags
{
    internal class TagBehavior
    {
        public static readonly Dictionary<Tag, TagBehavior> Registry =
            new Dictionary<Tag, TagBehavior>();

        private Tag _tag;

        public int Order { get; set; } = 0;
        public bool Show { get; set; } = true;
        public bool Automated { get; set; } = false;
        public Color Color { get; set; } = OVREditorUtils.HexToColor("#DDDDDD");
        public OVRGUIContent Icon { get; set; } = null;
        public bool CanFilterBy { get; set; } = true;
        public bool ShowOverlay { get; set; } = false;
        public bool ToggleableVisibility { get; set; } = false;
        public bool DefaultVisibility { get; set; } = true;
        public bool Visibility => VisibilitySetting.Value;

        private OVRProjectSetupSettingBool _visibilitySetting;
        public OVRProjectSetupSettingBool VisibilitySetting
            => _visibilitySetting ??= new OVRProjectSetupUserSettingBool($"Tag_{_tag.Name}_Visibility", DefaultVisibility, $"Show {_tag.Name} blocks");

        public static TagBehavior GetBehavior(Tag tag)
        {
            if (tag == null)
            {
                throw new ArgumentNullException(nameof(tag));
            }

            if (!Registry.TryGetValue(tag, out var tagBehavior))
            {
                tagBehavior = new TagBehavior(tag);
            }
            return tagBehavior;
        }

        public TagBehavior(Tag tag)
        {
            _tag = tag;
            Registry[tag] = this;
        }
    }
}
