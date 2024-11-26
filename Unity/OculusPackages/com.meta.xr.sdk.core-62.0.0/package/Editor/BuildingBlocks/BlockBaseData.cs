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
using System.Linq;
using Meta.XR.Editor.Tags;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEditor;

namespace Meta.XR.BuildingBlocks.Editor
{
    public abstract class BlockBaseData : ScriptableObject, ITaggable
    {
        [SerializeField, OVRReadOnly] internal string id = Guid.NewGuid().ToString();
        public string Id => id;

        [SerializeField, OVRReadOnly] internal int version = 1;
        public int Version => version;

        private static readonly OVRGUIContent DefaultThumbnailTexture = OVREditorUtils.CreateContent("bb_thumb_default.jpg",
            OVRGUIContent.Source.BuildingBlocksThumbnails);

        private static readonly OVRGUIContent DefaultInternalThumbnailTexture = OVREditorUtils.CreateContent("bb_thumb_internal.jpg",
            OVRGUIContent.Source.BuildingBlocksThumbnails);

        [SerializeField] internal string blockName;
        internal string BlockNameOverride;
        public string BlockName => string.IsNullOrEmpty(BlockNameOverride) ? blockName : BlockNameOverride;

        [SerializeField] internal string description;
        internal string DescriptionOverride;
        public string Description => string.IsNullOrEmpty(DescriptionOverride) ? description : DescriptionOverride;

        [SerializeField] internal TagArray tags;

        #region Tags
        public TagArray Tags => tags ??= new TagArray();
        public bool HasTag(Tag tag) => Tags.Contains(tag);
        public bool HasAnyTag(IEnumerable<Tag> tagArray) => Tags.Intersect(tagArray).Any();

        public void OnAwake()
        {
            ValidateTags();
        }

        public void OnValidate()
        {
            ValidateTags();
        }

        private void ValidateTags()
        {
            {
                Tags.Remove(Utils.InternalTag);
            }

            if (IsNew())
            {
                Tags.Add(Utils.NewTag);
            }
            else
            {
                Tags.Remove(Utils.NewTag);
            }

            Tags.OnValidate();
        }

        private OVRProjectSetupSettingBool _hasSeenBefore;

        private bool IsNew()
        {
            _hasSeenBefore ??= new OVRProjectSetupUserSettingBool($"HasSeenBeforeKey_{Id}", false);
            return !_hasSeenBefore.Value;
        }

        internal void MarkAsSeen()
        {
            if (_hasSeenBefore == null || _hasSeenBefore.Value)
            {
                return;
            }

            _hasSeenBefore.Value = true;
            ValidateTags();
        }

        internal void ResetSeen()
        {
            _hasSeenBefore.Value = false;
            ValidateTags();
        }
        #endregion

        [SerializeField] internal Texture2D thumbnail;

        public Texture2D Thumbnail
        {
            get
            {
                if (thumbnail != null)
                {
                    return thumbnail;
                }

                if (!Hidden)
                {
                    return DefaultThumbnailTexture.Content.image as Texture2D;
                }

                return DefaultInternalThumbnailTexture.Content.image as Texture2D;
            }
        }

        public virtual bool Hidden => HasTag(Utils.HiddenTag);

        public bool Experimental => HasTag(Utils.ExperimentalTag);


        [SerializeField] internal int order;
        public int Order => order;

        [ContextMenu("Assign ID")]
        internal void AssignId()
        {
            id = Guid.NewGuid().ToString();
        }

        [ContextMenu("Copy ID to clipboard")]
        internal void CopyIdToClipboard()
        {
            GUIUtility.systemCopyBuffer = Id;
        }

        [ContextMenu("Increment Version")]
        internal void IncrementVersion()
        {
            version++;
        }


        internal abstract bool CanBeAdded { get; }

        internal abstract void AddToProject(GameObject selectedGameObject = null, Action onInstall = null);

        internal virtual bool RequireListRefreshAfterInstall => false;

        internal virtual bool OverridesInstallRoutine => false;
    }
}
