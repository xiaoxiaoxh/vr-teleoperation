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

#if USING_XR_SDK_OPENXR
using Meta.XR;
using UnityEngine.XR.OpenXR;
using UnityEngine.XR.Management;
#endif

public partial class OVRManager
{
    public static FoveatedRenderingLevel GetFoveatedRenderingLevel()
    {
#if USING_XR_SDK_OPENXR
        if (IsOpenXRLoaderActive())
            return MetaXRFoveationFeature.foveatedRenderingLevel;
        else
#endif
            return (FoveatedRenderingLevel)OVRPlugin.foveatedRenderingLevel;
    }

    public static void SetFoveatedRenderingLevel(FoveatedRenderingLevel level)
    {
#if USING_XR_SDK_OPENXR
        if (IsOpenXRLoaderActive())
            MetaXRFoveationFeature.foveatedRenderingLevel = level;
        else
#endif
            OVRPlugin.foveatedRenderingLevel = (OVRPlugin.FoveatedRenderingLevel)level;
    }

    public static bool GetDynamicFoveatedRenderingEnabled()
    {
#if USING_XR_SDK_OPENXR
        if (IsOpenXRLoaderActive())
            return MetaXRFoveationFeature.useDynamicFoveatedRendering;
        else
#endif
            return OVRPlugin.useDynamicFoveatedRendering;
    }

    public static void SetDynamicFoveatedRenderingEnabled(bool enabled)
    {
#if USING_XR_SDK_OPENXR
        if (IsOpenXRLoaderActive())
            MetaXRFoveationFeature.useDynamicFoveatedRendering = enabled;
        else
#endif
            OVRPlugin.useDynamicFoveatedRendering = enabled;
    }

    public static bool GetEyeTrackedFoveatedRenderingSupported()
    {
#if USING_XR_SDK_OPENXR
        if (IsOpenXRLoaderActive())
            return MetaXREyeTrackedFoveationFeature.eyeTrackedFoveatedRenderingSupported;
        else
#endif
            return OVRPlugin.eyeTrackedFoveatedRenderingSupported;
    }

    public static bool GetEyeTrackedFoveatedRenderingEnabled()
    {
#if USING_XR_SDK_OPENXR
        if (IsOpenXRLoaderActive())
            return MetaXREyeTrackedFoveationFeature.eyeTrackedFoveatedRenderingEnabled;
        else
#endif
            return OVRPlugin.eyeTrackedFoveatedRenderingEnabled;
    }

    public static void SetEyeTrackedFoveatedRenderingEnabled(bool enabled)
    {
#if USING_XR_SDK_OPENXR
        if (IsOpenXRLoaderActive())
            MetaXREyeTrackedFoveationFeature.eyeTrackedFoveatedRenderingEnabled = enabled;
        else
#endif
        OVRPlugin.eyeTrackedFoveatedRenderingEnabled = enabled;
    }

    private static bool IsOpenXRLoaderActive()
    {
#if USING_XR_SDK_OPENXR
        XRLoader loader = XRGeneralSettings.Instance.Manager.activeLoader;
        OpenXRLoader openXRLoader = loader as OpenXRLoader;
        return openXRLoader != null;
#else
        return false;
#endif
    }
}
