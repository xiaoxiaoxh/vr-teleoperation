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
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace Meta.XR.BuildingBlocks
{
    public class SpatialAnchorCoreBuildingBlock : MonoBehaviour
    {
        public UnityEvent<OVRSpatialAnchor> OnAnchorCreateCompleted { get => _onAnchorCreateCompleted; set => _onAnchorCreateCompleted = value; }
        public UnityEvent OnAnchorsLoadCompleted { get => _onAnchorsLoadCompleted; set => _onAnchorsLoadCompleted = value; }
        public UnityEvent OnAnchorsEraseAllCompleted { get => _onAnchorsEraseAllCompleted; set => _onAnchorsEraseAllCompleted = value; }
        public UnityEvent<Guid> OnAnchorEraseCompleted { get => _onAnchorEraseCompleted; set => _onAnchorEraseCompleted = value; }

        [Header("# Events")]
        [SerializeField] private UnityEvent<OVRSpatialAnchor> _onAnchorCreateCompleted;
        [SerializeField] private UnityEvent _onAnchorsLoadCompleted;
        [SerializeField] private UnityEvent _onAnchorsEraseAllCompleted;
        [SerializeField] private UnityEvent<Guid> _onAnchorEraseCompleted;


        /// <summary>
        /// Create an spatial anchor.
        /// </summary>
        /// <param name="prefab">A prefab to add the <see cref="OVRSpatialAnchor"/> component.</param>
        /// <param name="position">Position for the new anchor.</param>
        /// <param name="rotation">Orientation of the new anchor</param>
        public void InstantiateSpatialAnchor(GameObject prefab, Vector3 position, Quaternion rotation)
        {
            if (prefab == null)
            {
                prefab = new GameObject("Spatial Anchor");
            }

            var anchorGameObject = Instantiate(prefab, position, rotation);
            var spatialAnchor = anchorGameObject.AddComponent<OVRSpatialAnchor>();
            StartCoroutine(InitSpatialAnchor(spatialAnchor));
        }

        private IEnumerator InitSpatialAnchor(OVRSpatialAnchor anchor)
        {
            yield return WaitForInit(anchor);
            yield return SaveLocalAsync(anchor);
            OnAnchorCreateCompleted?.Invoke(anchor);
        }

        protected IEnumerator WaitForInit(OVRSpatialAnchor anchor)
        {
            while (anchor && !anchor.Created)
                yield return null;

            if (anchor == null)
            {
                Debug.LogWarning($"[{nameof(SpatialAnchorCoreBuildingBlock)}] Failed to create the spatial anchor.");
            }
        }

        protected IEnumerator SaveLocalAsync(OVRSpatialAnchor anchor)
        {
            var task = anchor.SaveAsync();
            while (!task.IsCompleted)
                yield return null;

            if(!task.GetResult())
                Debug.LogWarning($"[{nameof(SpatialAnchorCoreBuildingBlock)}] Failed to save the spatial anchor.");
        }

        /// <summary>
        /// Load and instantiate anchors from a list of uuids.
        /// </summary>
        /// <param name="prefab">Prefab for instantiating the loaded anchors.</param>
        /// <param name="uuids">List of anchor's uuid to load.</param>
        public void LoadAndInstantiateAnchors(GameObject prefab, List<Guid> uuids)
        {
            if (uuids == null)
            {
                throw new ArgumentNullException();
            }

            if (uuids.Count == 0)
            {
                Debug.Log($"[{nameof(SpatialAnchorCoreBuildingBlock)}] Uuid list is empty.");
                return;
            }

            var options = new OVRSpatialAnchor.LoadOptions
            {
                Timeout = 0,
                StorageLocation = OVRSpace.StorageLocation.Local,
                Uuids = uuids
            };

            StartCoroutine(LoadAnchorsRoutine(prefab, options));
        }

        /// <summary>
        /// Erase all instantiated anchors anchors.
        /// </summary>
        /// <remarks>It'll collect the uuid(s) of the instantiated anchor(s) and erase them.</remarks>
        public void EraseAllAnchors()
        {
            // Nothing to erase.
            if (OVRSpatialAnchor.SpatialAnchors.Count == 0)
                return;

            StartCoroutine(EraseAnchorsRoutine());
        }

        /// <summary>
        /// Erase a anchor by <see cref="Guid"/>.
        /// </summary>
        /// <param name="uuid">Anchor's uuid to erase.</param>
        public void EraseAnchorByUuid(Guid uuid)
        {
            // Nothing to erase.
            if (OVRSpatialAnchor.SpatialAnchors.Count == 0)
                return;

            if (!OVRSpatialAnchor.SpatialAnchors.TryGetValue(uuid, out var anchor))
            {
                Debug.LogWarning($"[{nameof(SpatialAnchorCoreBuildingBlock)}] Spatial anchor with uuid [{uuid}] not found.");
                return;
            }

            StartCoroutine(EraseAnchorByUuidRoutine(anchor));
        }

        protected IEnumerator LoadAnchorsRoutine(GameObject prefab, OVRSpatialAnchor.LoadOptions options)
        {
            // Load unbounded anchors
            var task = OVRSpatialAnchor.LoadUnboundAnchorsAsync(options);
            while (!task.IsCompleted)
                yield return null;

            var unboundAnchors = task.GetResult();
            if (unboundAnchors == null || unboundAnchors.Length == 0)
            {
                Debug.Log($"[{nameof(SpatialAnchorCoreBuildingBlock)}] Failed to load the anchors.");
                yield break;
            }

            // Localize the anchors
            foreach (var anchor in unboundAnchors)
            {
                if (!anchor.Localized)
                {
                    var localizeTask = anchor.LocalizeAsync();
                    while (!localizeTask.IsCompleted)
                        yield return null;

                    if (!localizeTask.GetResult())
                    {
                        Debug.Log($"[{nameof(SpatialAnchorCoreBuildingBlock)}] Failed to localize the anchor. Uuid: {anchor.Uuid}");
                        continue;
                    }
                }

                var spatialAnchorGo = Instantiate(prefab, anchor.Pose.position, anchor.Pose.rotation);
                anchor.BindTo(spatialAnchorGo.AddComponent<OVRSpatialAnchor>());
            }

            OnAnchorsLoadCompleted?.Invoke();
        }

        private IEnumerator EraseAnchorsRoutine()
        {
            var anchorsToErase = OVRObjectPool.List<OVRSpatialAnchor>();
            foreach (var value in OVRSpatialAnchor.SpatialAnchors.Values)
            {
                anchorsToErase.Add(value);
            }

            for (int i = 0; i < anchorsToErase.Count; i++)
            {
                var anchor = anchorsToErase[i];
                yield return EraseAnchorByUuidRoutine(anchor);
            }

            if(OVRSpatialAnchor.SpatialAnchors.Count == 0)
                OnAnchorsEraseAllCompleted?.Invoke();

            OVRObjectPool.Return(anchorsToErase);
        }

        private IEnumerator EraseAnchorByUuidRoutine(OVRSpatialAnchor anchor)
        {
            var task = anchor.EraseAsync();
            if (!task.IsCompleted)
                yield return null;

            Destroy(anchor.gameObject);
            if (OVRSpatialAnchor.SpatialAnchors.ContainsKey(anchor.Uuid))
                yield return null;

            OnAnchorEraseCompleted?.Invoke(anchor.Uuid);
        }

        internal static List<SpatialAnchorCoreBuildingBlock> GetBaseInstances()
        {
            var baseClassObjects = OVRObjectPool.List<SpatialAnchorCoreBuildingBlock>();
            var objects = FindObjectsByType<SpatialAnchorCoreBuildingBlock>(FindObjectsSortMode.None);

            foreach (var obj in objects)
            {
                if (obj != null && obj.GetType() == typeof(SpatialAnchorCoreBuildingBlock))
                    baseClassObjects.Add(obj);
            }

            return baseClassObjects;
        }
    }
}
