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

namespace Oculus.Interaction
{
    /// <summary>
    /// A general interface for replacing the global time provider used
    /// in an object. Grouping classes that use this pattern under this
    /// interface allows changing multiple objects easily.
    /// </summary>
    public interface ITimeConsumer
    {
        /// <summary>
        /// Sets a function that returns the current time in seconds
        /// </summary>
        void SetTimeProvider(Func<float> timeProvider);
    }

    /// <summary>
    /// A general interface for replacing the delta time provider used
    /// in an object. Grouping classes that use this pattern under this
    /// interface allows changing multiple objects easily.
    /// </summary>
    public interface IDeltaTimeConsumer
    {
        /// <summary>
        /// Sets a function that returns the last delta time in seconds
        /// </summary>
        void SetDeltaTimeProvider(Func<float> deltaTimeProvider);
    }
}
