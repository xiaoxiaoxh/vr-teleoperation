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

namespace Oculus.Interaction
{
    /// <summary>
    /// By default, InteractorGroups are prioritized in list-order (first = highest priority). An <cref="ICandidateComparer" /> can prioritize <cref="IInteractor" />s in a different order based on their CandidateProperties.
    /// For instance, for those IInteractors whose CandidateProperties can be cast to an ICandidatePosition, a CandidatePositionComparer can prioritize IInteractors by their candidates’ position as measured by a distance to a common location.
    /// </summary>
    public interface ICandidateComparer
    {
        int Compare(object a, object b) ;
    }
}
