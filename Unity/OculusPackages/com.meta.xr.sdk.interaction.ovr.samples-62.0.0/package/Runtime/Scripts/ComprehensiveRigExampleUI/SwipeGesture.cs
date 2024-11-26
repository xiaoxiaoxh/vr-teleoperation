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
using UnityEngine.Events;
using UnityEngine.EventSystems;

/// <summary>
/// The SwipeGesture component listens to the drag events to detect if a gesture happened on a particular element in the local X or Y axis.
/// </summary>
public class SwipeGesture : UIBehaviour, IBeginDragHandler, IEndDragHandler
{
    public float gestureMaxDuration = 1;
    public float gestureMinDistanceNormalized = 0.15f;
    [Space(10)]
    public bool invertScroll;
    public enum Axis
    {
        Horizontal = 0,
        Vertical = 1
    }
    public Axis swipeAxis;

    private float startTime;
    private Vector2 startLocalPosition;
    [Space(10)]
    [SerializeField] private UnityEvent<int> swipeExecuted;

    public void OnBeginDrag(PointerEventData eventData)
    {
        startTime = Time.time;
        var viewRect = (RectTransform)transform;
        Vector2 pointerLocalPosition;
        RectTransformUtility.ScreenPointToLocalPointInRectangle(viewRect, eventData.position, eventData.pressEventCamera, out pointerLocalPosition);
        startLocalPosition = pointerLocalPosition;
    }
    public void OnEndDrag(PointerEventData eventData)
    {
        var duration = Time.time - startTime;
        Vector2 pointerLocalPosition;
        var viewRect = (RectTransform)transform;
        RectTransformUtility.ScreenPointToLocalPointInRectangle(viewRect, eventData.position, eventData.pressEventCamera, out pointerLocalPosition);

        var positionDelta = pointerLocalPosition - startLocalPosition;
        var isDragOnAxis = Mathf.Abs(positionDelta.normalized[(int)swipeAxis]) > 0.5f;

        var isValidGestureDuration = duration < gestureMaxDuration;

        var size = swipeAxis == Axis.Horizontal ? viewRect.rect.width : viewRect.rect.height;
        var movementOverAxis = Mathf.Abs(positionDelta[(int)swipeAxis]);
        var minRequiredTravelDistance = size * gestureMinDistanceNormalized;
        var isProperSwipeDistance = movementOverAxis > minRequiredTravelDistance;

        if(isDragOnAxis && isProperSwipeDistance && isValidGestureDuration) {
            var direction = (int)Mathf.Sign(positionDelta[(int)swipeAxis]);
            direction *= invertScroll ? -1 : 1;
            swipeExecuted.Invoke(direction);
        }
    }
}
