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

namespace Oculus.Interaction
{
    public class TransformerUtils
    {
        [Serializable]
        public struct FloatRange
        {
            public float Min;
            public float Max;
        }

        [Serializable]
        public struct ConstrainedAxis
        {
            public bool ConstrainAxis;
            public FloatRange AxisRange;
        }

        [Serializable]
        public class PositionConstraints
        {
            public bool ConstraintsAreRelative;
            public ConstrainedAxis XAxis;
            public ConstrainedAxis YAxis;
            public ConstrainedAxis ZAxis;
        }

        [Serializable]
        public class RotationConstraints
        {
            public ConstrainedAxis XAxis;
            public ConstrainedAxis YAxis;
            public ConstrainedAxis ZAxis;
        }

        [Serializable]
        public class ScaleConstraints
        {
            public bool ConstraintsAreRelative;
            public ConstrainedAxis XAxis;
            public ConstrainedAxis YAxis;
            public ConstrainedAxis ZAxis;
        }

        public static PositionConstraints GenerateParentConstraints(PositionConstraints constraints, Vector3 initialPosition)
        {
            PositionConstraints parentConstraints;

            if (!constraints.ConstraintsAreRelative)
            {
                parentConstraints = constraints;
            }
            else
            {
                parentConstraints = new PositionConstraints();

                parentConstraints.XAxis = new ConstrainedAxis();
                parentConstraints.YAxis = new ConstrainedAxis();
                parentConstraints.ZAxis = new ConstrainedAxis();

                if (constraints.XAxis.ConstrainAxis)
                {
                    parentConstraints.XAxis.ConstrainAxis = true;
                    parentConstraints.XAxis.AxisRange.Min = constraints.XAxis.AxisRange.Min + initialPosition.x;
                    parentConstraints.XAxis.AxisRange.Max = constraints.XAxis.AxisRange.Max + initialPosition.x;
                }
                if (constraints.YAxis.ConstrainAxis)
                {
                    parentConstraints.YAxis.ConstrainAxis = true;
                    parentConstraints.YAxis.AxisRange.Min = constraints.YAxis.AxisRange.Min + initialPosition.y;
                    parentConstraints.YAxis.AxisRange.Max = constraints.YAxis.AxisRange.Max + initialPosition.y;
                }
                if (constraints.ZAxis.ConstrainAxis)
                {
                    parentConstraints.ZAxis.ConstrainAxis = true;
                    parentConstraints.ZAxis.AxisRange.Min = constraints.ZAxis.AxisRange.Min + initialPosition.z;
                    parentConstraints.ZAxis.AxisRange.Max = constraints.ZAxis.AxisRange.Max + initialPosition.z;
                }
            }

            return parentConstraints;
        }

        public static ScaleConstraints GenerateParentConstraints(ScaleConstraints constraints, Vector3 initialScale)
        {
            ScaleConstraints parentConstraints;

            if (!constraints.ConstraintsAreRelative)
            {
                parentConstraints = constraints;
            }
            else
            {
                parentConstraints = new ScaleConstraints();

                parentConstraints.XAxis = new ConstrainedAxis();
                parentConstraints.YAxis = new ConstrainedAxis();
                parentConstraints.ZAxis = new ConstrainedAxis();

                if (constraints.XAxis.ConstrainAxis)
                {
                    parentConstraints.XAxis.ConstrainAxis = true;
                    parentConstraints.XAxis.AxisRange.Min = constraints.XAxis.AxisRange.Min * initialScale.x;
                    parentConstraints.XAxis.AxisRange.Max = constraints.XAxis.AxisRange.Max * initialScale.x;
                }
                if (constraints.YAxis.ConstrainAxis)
                {
                    parentConstraints.YAxis.ConstrainAxis = true;
                    parentConstraints.YAxis.AxisRange.Min = constraints.YAxis.AxisRange.Min * initialScale.y;
                    parentConstraints.YAxis.AxisRange.Max = constraints.YAxis.AxisRange.Max * initialScale.y;
                }
                if (constraints.ZAxis.ConstrainAxis)
                {
                    parentConstraints.ZAxis.ConstrainAxis = true;
                    parentConstraints.ZAxis.AxisRange.Min = constraints.ZAxis.AxisRange.Min * initialScale.z;
                    parentConstraints.ZAxis.AxisRange.Max = constraints.ZAxis.AxisRange.Max * initialScale.z;
                }
            }

            return parentConstraints;
        }

        public static Vector3 GetConstrainedTransformPosition(Vector3 unconstrainedPosition, PositionConstraints positionConstraints, Transform relativeTransform = null)
        {
            Vector3 constrainedPosition = unconstrainedPosition;

            // the translation constraints occur in parent space
            if (relativeTransform != null)
            {
                constrainedPosition = relativeTransform.InverseTransformPoint(constrainedPosition);
            }

            if (positionConstraints.XAxis.ConstrainAxis)
            {
                constrainedPosition.x = Mathf.Clamp(constrainedPosition.x, positionConstraints.XAxis.AxisRange.Min, positionConstraints.XAxis.AxisRange.Max);
            }
            if (positionConstraints.YAxis.ConstrainAxis)
            {
                constrainedPosition.y = Mathf.Clamp(constrainedPosition.y, positionConstraints.YAxis.AxisRange.Min, positionConstraints.YAxis.AxisRange.Max);
            }
            if (positionConstraints.ZAxis.ConstrainAxis)
            {
                constrainedPosition.z = Mathf.Clamp(constrainedPosition.z, positionConstraints.ZAxis.AxisRange.Min, positionConstraints.ZAxis.AxisRange.Max);
            }

            // Convert the constrained position back to world space
            if (relativeTransform != null)
            {
                constrainedPosition = relativeTransform.TransformPoint(constrainedPosition);
            }

            return constrainedPosition;
        }

        public static Quaternion GetConstrainedTransformRotation(Quaternion unconstrainedRotation, RotationConstraints rotationConstraints)
        {
            var newX = unconstrainedRotation.eulerAngles.x;
            var newY = unconstrainedRotation.eulerAngles.y;
            var newZ = unconstrainedRotation.eulerAngles.z;

            if (rotationConstraints.XAxis.ConstrainAxis)
            {
                newX = Mathf.Clamp(unconstrainedRotation.eulerAngles.x, rotationConstraints.XAxis.AxisRange.Min, rotationConstraints.XAxis.AxisRange.Max);
            }
            if (rotationConstraints.YAxis.ConstrainAxis)
            {
                newY = Mathf.Clamp(unconstrainedRotation.eulerAngles.y, rotationConstraints.YAxis.AxisRange.Min, rotationConstraints.YAxis.AxisRange.Max);
            }
            if (rotationConstraints.ZAxis.ConstrainAxis)
            {
                newZ = Mathf.Clamp(unconstrainedRotation.eulerAngles.z, rotationConstraints.ZAxis.AxisRange.Min, rotationConstraints.ZAxis.AxisRange.Max);
            }

            return Quaternion.Euler(newX, newY, newZ);
        }

        public static Vector3 GetConstrainedTransformScale(Vector3 unconstrainedScale, ScaleConstraints scaleConstraints)
        {
            Vector3 constrainedScale = unconstrainedScale;

            if (scaleConstraints.XAxis.ConstrainAxis)
            {
                constrainedScale.x = Mathf.Clamp(constrainedScale.x, scaleConstraints.XAxis.AxisRange.Min, scaleConstraints.XAxis.AxisRange.Max);
            }
            if (scaleConstraints.YAxis.ConstrainAxis)
            {
                constrainedScale.y = Mathf.Clamp(constrainedScale.y, scaleConstraints.YAxis.AxisRange.Min, scaleConstraints.YAxis.AxisRange.Max);
            }
            if (scaleConstraints.ZAxis.ConstrainAxis)
            {
                constrainedScale.z = Mathf.Clamp(constrainedScale.z, scaleConstraints.ZAxis.AxisRange.Min, scaleConstraints.ZAxis.AxisRange.Max);
            }

            return constrainedScale;
        }
    }
}
