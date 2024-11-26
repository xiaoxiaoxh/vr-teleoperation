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

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Video;

namespace Meta.XR.SharedAssets
{
    public class PlayMultipleVideos : MonoBehaviour
    {
        [SerializeField] VideoClip[] _videoClips;
        VideoPlayer videoPlayer;
        int clipNumber = -1;
        private void Start()
        {
            videoPlayer = GetComponentInChildren<VideoPlayer>();
            if (!videoPlayer)
            {
                Debug.LogWarning("This script requires a VideoPlayer. Make sure you have a properly setup video player first, in this object or in one of its children.");
            }
        }
        void Update()
        {
            if (!videoPlayer)
            {
                return;
            }
            videoPlayer.loopPointReached += EndReached;
        }
        void EndReached(UnityEngine.Video.VideoPlayer vp)
        {
            if (clipNumber == _videoClips.Length - 1)
            {
                clipNumber = 0;
            }
            else
            {
                clipNumber++;
            }
            videoPlayer.clip = _videoClips[clipNumber];
            videoPlayer.Play();
        }
    }
}
