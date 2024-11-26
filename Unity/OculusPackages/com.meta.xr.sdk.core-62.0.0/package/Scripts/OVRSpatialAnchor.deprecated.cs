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
using Unity.Collections;

partial class OVRSpatialAnchor
{
    /// <summary>
    /// Initializes this component from an existing space handle and uuid, e.g., the result of a call to
    /// <see cref="OVRPlugin.QuerySpaces"/>.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. To create a new anchor, use
    /// <code><![CDATA[AddComponent<OVRSpatialAnchor>()]]></code>. To load a previously saved anchor, use
    /// <see cref="LoadUnboundAnchorsAsync"/>.
    ///
    /// This method associates the component with an existing spatial anchor, for example, the one that was saved in
    /// a previous session. Do not call this method to create a new spatial anchor.
    ///
    /// If you call this method, you must do so prior to the component's `Start` method. You cannot change the spatial
    /// anchor associated with this component after that.
    /// </remarks>
    /// <param name="space">The existing <see cref="OVRSpace"/> to associate with this spatial anchor.</param>
    /// <param name="uuid">The universally unique identifier to associate with this spatial anchor.</param>
    /// <exception cref="InvalidOperationException">Thrown if `Start` has already been called on this component.</exception>
    /// <exception cref="ArgumentException">Thrown if <paramref name="space"/> is not <see cref="OVRSpace.Valid"/>.</exception>
    [Obsolete("You should use LoadUnboundAnchorsAsync to load previously saved anchors and" +
              " AddComponent<OVRSpatialAnchor>() to create a new anchor. You should no longer need to use an OVRSpace" +
              " handle directly.")]
    public void InitializeFromExisting(OVRSpace space, Guid uuid)
    {
        if (_startCalled)
            throw new InvalidOperationException(
                $"Cannot call {nameof(InitializeFromExisting)} after {nameof(Start)}. This must be set once upon creation.");

        try
        {
            if (!space.Valid)
                throw new ArgumentException($"Invalid space {space}.", nameof(space));

            ThrowIfBound(uuid);
        }
        catch
        {
            Destroy(this);
            throw;
        }

        InitializeUnchecked(space, uuid);
    }

    /// <summary>
    /// Saves the <see cref="OVRSpatialAnchor"/> to local persistent storage.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. Use <see cref="SaveAsync()"/> instead. To continue using the
    /// <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the returned task.
    ///
    /// This method is asynchronous. Use <paramref name="onComplete"/> to be notified of completion.
    ///
    /// When saved, an <see cref="OVRSpatialAnchor"/> can be loaded by a different session. Use the
    /// <see cref="Uuid"/> to identify the same <see cref="OVRSpatialAnchor"/> at a future time.
    ///
    /// This operation fully succeeds or fails, which means, either all anchors are successfully saved,
    /// or the operation fails.
    /// </remarks>
    /// <param name="onComplete">
    /// Invoked when the save operation completes. May be null. Parameters are
    /// - <see cref="OVRSpatialAnchor"/>: The anchor being saved.
    /// - `bool`: A value indicating whether the save operation succeeded.
    /// </param>
    [Obsolete("Use SaveAsync instead.")]
    public void Save(Action<OVRSpatialAnchor, bool> onComplete = null)
    {
        Save(_defaultSaveOptions, onComplete);
    }

    /// <summary>
    /// Saves the <see cref="OVRSpatialAnchor"/> with specified <see cref="SaveOptions"/>.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. Use <see cref="SaveAsync()"/> instead. To continue using the
    /// <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the returned task.
    ///
    /// This method is asynchronous. Use <paramref name="onComplete"/> to be notified of completion.
    /// When saved, the <see cref="OVRSpatialAnchor"/> can be loaded by a different session. Use the
    /// <see cref="Uuid"/> to identify the same <see cref="OVRSpatialAnchor"/> at a future time.
    ///
    /// This operation fully succeeds or fails; that is, either all anchors are successfully saved,
    /// or the operation fails.
    /// </remarks>
    /// <param name="saveOptions">Save options, e.g., whether local or cloud.</param>
    /// <param name="onComplete">
    /// Invoked when the save operation completes. May be null. Parameters are
    /// - <see cref="OVRSpatialAnchor"/>: The anchor being saved.
    /// - `bool`: A value indicating whether the save operation succeeded.
    /// </param>
    [Obsolete("Use SaveAsync instead.")]
    public void Save(SaveOptions saveOptions, Action<OVRSpatialAnchor, bool> onComplete = null)
    {
        var task = SaveAsync(saveOptions);
        if (onComplete != null)
        {
            InvertedCapture<bool, OVRSpatialAnchor>.ContinueTaskWith(task, onComplete, this);
        }
    }

    /// <summary>
    /// The space associated with the spatial anchor.
    /// </summary>
    /// <remarks>
    /// NOTE: This property is obsolete. This class provides all spatial anchor functionality and it should not be
    /// necessary to use this low-level handle directly. See <see cref="SaveAsync()"/>,
    /// <see cref="ShareAsync(OVRSpaceUser)"/>, and <see cref="EraseAsync()"/>.
    ///
    /// The <see cref="OVRSpace"/> represents the runtime instance of the spatial anchor and will change across
    /// different sessions.
    /// </remarks>
    [Obsolete("This property exposes an internal handle that should no longer be necessary. You can Save, Erase," +
              " and Share anchors using the methods in this class.")]
    public OVRSpace Space => _anchor.Handle;

    /// <summary>
    /// Shares the anchor to an <see cref="OVRSpaceUser"/>.
    /// The specified user will be able to download, track, and share specified anchors.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. Use <see cref="ShareAsync(OVRSpaceUser)"/> instead. To continue using the
    /// <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the returned task.
    ///
    /// This method is asynchronous. Use <paramref name="onComplete"/> to be notified of completion.
    /// </remarks>
    /// <param name="user">An Oculus user to share the anchor with.</param>
    /// <param name="onComplete">
    /// Invoked when the share operation completes. May be null. Delegate parameter is
    /// - `OperationResult`: An error code that indicates whether the share operation succeeded or not.
    /// </param>
    [Obsolete("Use ShareAsync instead.")]
    public void Share(OVRSpaceUser user, Action<OperationResult> onComplete = null)
    {
        var task = ShareAsync(user);
        if (onComplete != null)
        {
            task.ContinueWith(onComplete);
        }
    }

    /// <summary>
    /// Shares the anchor with two <see cref="OVRSpaceUser"/>.
    /// Specified users will be able to download, track, and share specified anchors.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. Use <see cref="ShareAsync(OVRSpaceUser, OVRSpaceUser)"/> instead. To continue
    /// using the <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the returned task.
    ///
    /// This method is asynchronous. Use <paramref name="onComplete"/> to be notified of completion.
    /// </remarks>
    /// <param name="user1">An Oculus user to share the anchor with.</param>
    /// <param name="user2">An Oculus user to share the anchor with.</param>
    /// <param name="onComplete">
    /// Invoked when the share operation completes. May be null. Delegate parameter is
    /// - `OperationResult`: An error code that indicates whether the share operation succeeded or not.
    /// </param>
    [Obsolete("Use ShareAsync instead.")]
    public void Share(OVRSpaceUser user1, OVRSpaceUser user2, Action<OperationResult> onComplete = null)
    {
        var task = ShareAsync(user1, user2);
        if (onComplete != null)
        {
            task.ContinueWith(onComplete);
        }
    }

    /// <summary>
    /// Shares the anchor with three <see cref="OVRSpaceUser"/>.
    /// Specified users will be able to download, track, and share specified anchors.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. Use <see cref="ShareAsync(OVRSpaceUser, OVRSpaceUser, OVRSpaceUser)"/> instead.
    /// To continue using the <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the
    /// returned task.
    ///
    /// This method is asynchronous. Use <paramref name="onComplete"/> to be notified of completion.
    /// </remarks>
    /// <param name="user1">An Oculus user to share the anchor with.</param>
    /// <param name="user2">An Oculus user to share the anchor with.</param>
    /// <param name="user3">An Oculus user to share the anchor with.</param>
    /// <param name="onComplete">
    /// Invoked when the share operation completes. May be null. Delegate parameter is
    /// - `OperationResult`: An error code that indicates whether the share operation succeeded or not.
    /// </param>
    [Obsolete("Use ShareAsync instead.")]
    public void Share(OVRSpaceUser user1, OVRSpaceUser user2, OVRSpaceUser user3,
        Action<OperationResult> onComplete = null)
    {
        var task = ShareAsync(user1, user2, user3);
        if (onComplete != null)
        {
            task.ContinueWith(onComplete);
        }
    }

    /// <summary>
    /// Shares the anchor with four <see cref="OVRSpaceUser"/>.
    /// Specified users will be able to download, track, and share specified anchors.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. Use
    /// <see cref="ShareAsync(OVRSpaceUser, OVRSpaceUser, OVRSpaceUser, OVRSpaceUser)"/> instead. To continue using the
    /// <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the returned task.
    ///
    /// This method is asynchronous. Use <paramref name="onComplete"/> to be notified of completion.
    /// </remarks>
    /// <param name="user1">An Oculus user to share the anchor with.</param>
    /// <param name="user2">An Oculus user to share the anchor with.</param>
    /// <param name="user3">An Oculus user to share the anchor with.</param>
    /// <param name="user4">An Oculus user to share the anchor with.</param>
    /// <param name="onComplete">
    /// Invoked when the share operation completes. May be null. Delegate parameter is
    /// - `OperationResult`: An error code that indicates whether the share operation succeeded or not.
    /// </param>
    [Obsolete("Use ShareAsync instead.")]
    public void Share(OVRSpaceUser user1, OVRSpaceUser user2, OVRSpaceUser user3, OVRSpaceUser user4,
        Action<OperationResult> onComplete = null)
    {
        var task = ShareAsync(user1, user2, user3, user4);
        if (onComplete != null)
        {
            task.ContinueWith(onComplete);
        }
    }

    /// <summary>
    /// Shares the anchor to a collection of <see cref="OVRSpaceUser"/>.
    /// Specified users will be able to download, track, and share specified anchors.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. Use <see cref="ShareAsync(IEnumerable{OVRSpaceUser})"/>. To continue using the
    /// <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the returned task.
    ///
    /// This method is asynchronous. Use <paramref name="onComplete"/> to be notified of completion.
    /// </remarks>
    /// <param name="users">A collection of Oculus users to share the anchor with.</param>
    /// <param name="onComplete">
    /// Invoked when the share operation completes. May be null. Delegate parameter is
    /// - `OperationResult`: An error code that indicates whether the share operation succeeded or not.
    /// </param>
    [Obsolete("Use ShareAsync instead.")]
    public void Share(IEnumerable<OVRSpaceUser> users, Action<OperationResult> onComplete = null)
    {
        var task = ShareAsync(users);
        if (onComplete != null)
        {
            task.ContinueWith(onComplete);
        }
    }

    /// <summary>
    /// Erases the <see cref="OVRSpatialAnchor"/> from persistent storage.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. Use <see cref="EraseAsync()"/>. To continue using the
    /// <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the returned task.
    ///
    /// This method is asynchronous. Use <paramref name="onComplete"/> to be notified of completion.
    /// Erasing an <see cref="OVRSpatialAnchor"/> does not destroy the anchor.
    /// </remarks>
    /// <param name="onComplete">
    /// Invoked when the erase operation completes. May be null. Parameters are
    /// - <see cref="OVRSpatialAnchor"/>: The anchor being erased.
    /// - `bool`: A value indicating whether the erase operation succeeded.
    /// </param>
    [Obsolete("Use EraseAsync instead.")]
    public void Erase(Action<OVRSpatialAnchor, bool> onComplete = null)
    {
        Erase(_defaultEraseOptions, onComplete);
    }

    /// <summary>
    /// Erases the <see cref="OVRSpatialAnchor"/> from specified storage.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. Use <see cref="EraseAsync(EraseOptions)"/>. To continue using the
    /// <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the returned task.
    ///
    /// This method is asynchronous. Use <paramref name="onComplete"/> to be notified of completion.
    /// Erasing an <see cref="OVRSpatialAnchor"/> does not destroy the anchor.
    /// </remarks>
    /// <param name="eraseOptions">Options how the anchor should be erased.</param>
    /// <param name="onComplete">
    /// Invoked when the erase operation completes. May be null. Parameters are
    /// - <see cref="OVRSpatialAnchor"/>: The anchor being erased.
    /// - `bool`: A value indicating whether the erase operation succeeded.
    /// </param>
    [Obsolete("Use EraseAsync instead.")]
    public void Erase(EraseOptions eraseOptions, Action<OVRSpatialAnchor, bool> onComplete = null)
    {
        var task = EraseAsync(eraseOptions);

        if (onComplete != null)
        {
            InvertedCapture<bool, OVRSpatialAnchor>.ContinueTaskWith(task, onComplete, this);
        }
    }

    /// <summary>
    /// Performs a query for anchors with the specified <paramref name="options"/>.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. Use <see cref="LoadUnboundAnchorsAsync"/>. To continue using the
    /// <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the returned task.
    ///
    /// Use this method to find anchors that were previously persisted with
    /// <see cref="Save(Action{OVRSpatialAnchor, bool}"/>. The query is asynchronous; when the query completes,
    /// <paramref name="onComplete"/> is invoked with an array of <see cref="UnboundAnchor"/>s for which tracking
    /// may be requested.
    /// </remarks>
    /// <param name="options">Options that affect the query.</param>
    /// <param name="onComplete">A delegate invoked when the query completes. The delegate accepts one argument:
    /// - `UnboundAnchor[]`: An array of unbound anchors.
    ///
    /// If the operation fails, <paramref name="onComplete"/> is invoked with `null`.</param>
    /// <returns>Returns `true` if the operation could be initiated; otherwise `false`.</returns>
    /// <exception cref="ArgumentNullException">Thrown if <paramref name="onComplete"/> is `null`.</exception>
    /// <exception cref="InvalidOperationException">Thrown if <see cref="LoadOptions.Uuids"/> of <paramref name="options"/> is `null`.</exception>
    [Obsolete("Use LoadUnboundAnchorsAsync instead.")]
    public static bool LoadUnboundAnchors(LoadOptions options, Action<UnboundAnchor[]> onComplete)
    {
        var task = LoadUnboundAnchorsAsync(options);
        task.ContinueWith(onComplete);
        return task.IsPending;
    }

    partial struct UnboundAnchor
    {
        /// <summary>
        /// Localizes an anchor.
        /// </summary>
        /// <remarks>
        /// NOTE: This method is obsolete. Use <see cref="LocalizeAsync"/> instead. To continue using the
        /// <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the returned task.
        ///
        /// The delegate supplied to <see cref="OVRSpatialAnchor.LoadUnboundAnchors"/> receives an array of unbound
        /// spatial anchors. You can choose whether to localize each one and be notified when localization completes.
        ///
        /// The <paramref name="onComplete"/> delegate receives two arguments:
        /// - `bool`: Whether localization was successful
        /// - <see cref="UnboundAnchor"/>: The anchor to bind
        ///
        /// Upon successful localization, your delegate should instantiate an <see cref="OVRSpatialAnchor"/>, then bind
        /// the <see cref="UnboundAnchor"/> to the <see cref="OVRSpatialAnchor"/> by calling
        /// <see cref="UnboundAnchor.BindTo"/>. Once an <see cref="UnboundAnchor"/> is bound to an
        /// <see cref="OVRSpatialAnchor"/>, it cannot be used again; that is, it cannot be bound to multiple
        /// <see cref="OVRSpatialAnchor"/> components.
        /// </remarks>
        /// <param name="onComplete">A delegate invoked when localization completes (which may fail). The delegate
        /// receives two arguments:
        /// - <see cref="UnboundAnchor"/>: The anchor to bind
        /// - `bool`: Whether localization was successful
        /// </param>
        /// <param name="timeout">The timeout, in seconds, to attempt localization, or zero to indicate no timeout.</param>
        /// <exception cref="InvalidOperationException">Thrown if
        /// - The anchor does not support localization, e.g., because it is invalid.
        /// - The anchor has already been localized.
        /// - The anchor is being localized, e.g., because <see cref="Localize"/> was previously called.
        /// </exception>
        [Obsolete("Use LocalizeAsync instead.")]
        public void Localize(Action<UnboundAnchor, bool> onComplete = null, double timeout = 0)
        {
            var task = LocalizeAsync(timeout);

            if (onComplete != null)
            {
                InvertedCapture<bool, UnboundAnchor>.ContinueTaskWith(task, onComplete, this);
            }
        }
    }

    /// <summary>
    /// Shares a collection of <see cref="OVRSpatialAnchor"/> to specified users.
    /// Specified users will be able to download, track, and share specified anchors.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. Use
    /// <see cref="ShareAsync(IEnumerable{OVRSpatialAnchor},IEnumerable{OVRSpaceUser})"/> instead. To continue using the
    /// <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the returned task.
    ///
    /// This method is asynchronous. Use <paramref name="onComplete"/> to be notified of completion.
    ///
    /// This operation fully succeeds or fails, which means, either all anchors are successfully shared
    /// or the operation fails.
    /// </remarks>
    /// <param name="anchors">The collection of anchors to share.</param>
    /// <param name="users">An array of Oculus users to share these anchors with.</param>
    /// <param name="onComplete">
    /// Invoked when the share operation completes. May be null. Delegate parameter is
    /// - `ICollection&lt;OVRSpatialAnchor&gt;`: The collection of anchors being shared.
    /// - `OperationResult`: An error code that indicates whether the share operation succeeded or not.
    /// </param>
    /// <exception cref="ArgumentNullException">Thrown if <paramref name="anchors"/> is `null`.</exception>
    /// <exception cref="ArgumentNullException">Thrown if <paramref name="users"/> is `null`.</exception>
    [Obsolete("Use ShareAsync instead.")]
    public static void Share(ICollection<OVRSpatialAnchor> anchors, ICollection<OVRSpaceUser> users,
        Action<ICollection<OVRSpatialAnchor>, OperationResult> onComplete = null)
    {
        if (anchors == null)
            throw new ArgumentNullException(nameof(anchors));

        using var spaces = ToNativeArray(anchors);

        var handles = new NativeArray<ulong>(users.Count, Allocator.Temp);
        using var disposer = handles;
        int i = 0;
        foreach (var user in users)
        {
            handles[i++] = user._handle;
        }

        var shareResult = OVRPlugin.ShareSpaces(spaces, handles, out var requestId);
        if (shareResult.IsSuccess())
        {
            Development.LogRequest(requestId, $"Sharing {(uint)spaces.Length} spatial anchors...");

            MultiAnchorCompletionDelegates[requestId] = new MultiAnchorDelegatePair
            {
                Anchors = CopyAnchorListIntoListFromPool(anchors),
                Delegate = onComplete
            };
        }
        else
        {
            Development.LogError(
                $"{nameof(OVRPlugin)}.{nameof(OVRPlugin.ShareSpaces)}  failed with error {shareResult}.");
            onComplete?.Invoke(anchors, (OperationResult)shareResult);
        }
    }

    /// <summary>
    /// Saves a collection of anchors to persistent storage.
    /// </summary>
    /// <remarks>
    /// NOTE: This method is obsolete. Use
    /// <see cref="SaveAsync(IEnumerable{OVRSpatialAnchor}, SaveOptions)"/> instead. To continue using the
    /// <paramref name="onComplete"/> callback, use <see cref="OVRTask{T}.ContinueWith"/> on the returned task.
    ///
    /// This method is asynchronous. Use <paramref name="onComplete"/> to be notified of completion.
    /// When saved, an <see cref="OVRSpatialAnchor"/> can be loaded by a different session. Use the
    /// <see cref="Uuid"/> to identify the same <see cref="OVRSpatialAnchor"/> at a future time.
    /// </remarks>
    /// <param name="anchors">Collection of anchors</param>
    /// <param name="saveOptions">Save options, e.g., whether local or cloud.</param>
    /// <param name="onComplete">
    /// Invoked when the save operation completes. May be null. <paramref name="onComplete"/> receives two parameters:
    /// - `ICollection&lt;OVRSpatialAnchor&gt;`: The same collection as in <paramref name="anchors"/> parameter
    /// - `OperationResult`: An error code indicating whether the save operation succeeded or not.
    /// </param>
    /// <exception cref="ArgumentNullException">Thrown if <paramref name="anchors"/> is `null`.</exception>
    [Obsolete("Use SaveAsync instead.")]
    public static void Save(ICollection<OVRSpatialAnchor> anchors, SaveOptions saveOptions,
        Action<ICollection<OVRSpatialAnchor>, OperationResult> onComplete = null)
    {
        if (anchors == null)
            throw new ArgumentNullException(nameof(anchors));

        using var spaces = ToNativeArray(anchors);
        var saveResult =
            OVRPlugin.SaveSpaceList(spaces, saveOptions.Storage.ToSpaceStorageLocation(), out var requestId);

        if (saveResult.IsSuccess())
        {
            Development.LogRequest(requestId, $"Saving spatial anchors...");

            MultiAnchorCompletionDelegates[requestId] = new MultiAnchorDelegatePair
            {
                Anchors = CopyAnchorListIntoListFromPool(anchors),
                Delegate = onComplete
            };
        }
        else
        {
            Development.LogError(
                $"{nameof(OVRPlugin)}.{nameof(OVRPlugin.SaveSpaceList)} failed with error {saveResult}.");
            onComplete?.Invoke(anchors, (OperationResult)saveResult);
        }
    }
}
