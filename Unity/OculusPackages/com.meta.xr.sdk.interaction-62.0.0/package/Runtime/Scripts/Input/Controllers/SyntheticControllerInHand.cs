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

namespace Oculus.Interaction.Input
{
    public class SyntheticControllerInHand : Controller
    {
        [SerializeField, Interface(typeof(IHand)), Optional]
        private UnityEngine.Object _rawHand;
        private IHand RawHand;

        [SerializeField, Interface(typeof(IHand)), Optional]
        private UnityEngine.Object _syntheticHand;
        private IHand SyntheticHand;

        private Pose _handToControllerDelta = Pose.identity;
        private Pose _rootToPointerDelta = Pose.identity;

        protected virtual void Awake()
        {
            RawHand = _rawHand as IHand;
            SyntheticHand = _syntheticHand as IHand;
        }

        protected override void Start()
        {
            this.BeginStart(ref _started, () => base.Start());
            if (_rawHand != null)
            {
                this.AssertField(RawHand, nameof(RawHand));
            }
            if (_syntheticHand != null)
            {
                this.AssertField(SyntheticHand, nameof(SyntheticHand));
            }
            this.EndStart(ref _started);
        }

        protected override void UpdateData()
        {
            if (Started)
            {
                RecalculateOffset();
            }
            base.UpdateData();
        }

        protected override void Apply(ControllerDataAsset data)
        {
            if (SyntheticHand != null
                && SyntheticHand.GetRootPose(out Pose syntheticHandPose))
            {
                Pose pose = Pose.identity;
                PoseUtils.Multiply(syntheticHandPose, _handToControllerDelta, ref pose);
                data.RootPose = pose;

                PoseUtils.Multiply(syntheticHandPose, _handToControllerDelta, ref pose);
                PoseUtils.Multiply(pose, _rootToPointerDelta, ref pose);
                data.PointerPose = pose;
            }
        }

        protected void RecalculateOffset()
        {
            if (RawHand != null
                && RawHand.GetRootPose(out Pose rawHandPose))
            {
                ControllerDataAsset baseData = ModifyDataFromSource.GetData();
                Pose rawRoot = baseData.RootPose;
                Pose rawPointer = baseData.PointerPose;

                _rootToPointerDelta = PoseUtils.Delta(rawRoot, rawPointer);
                _handToControllerDelta = PoseUtils.Delta(rawHandPose, rawRoot);
            }
        }

        #region Inject

        public void InjectAllSyntheticControllerInHand(UpdateModeFlags updateMode, IDataSource updateAfter,
            IDataSource<ControllerDataAsset> modifyDataFromSource, bool applyModifier)
        {
            base.InjectAllController(updateMode, updateAfter, modifyDataFromSource, applyModifier);
        }

        public void InjectOptionalRawHand(IHand rawHand)
        {
            _rawHand = rawHand as UnityEngine.Object;
            RawHand = rawHand;
        }

        public void InjectOptionalSyntheticHand(IHand syntheticHand)
        {
            _syntheticHand = syntheticHand as UnityEngine.Object;
            SyntheticHand = syntheticHand;
        }

        #endregion
    }
}
