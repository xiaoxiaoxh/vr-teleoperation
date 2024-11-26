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
using UnityEngine;

namespace Oculus.Interaction
{
    /// <summary>
    /// Interactor provides a base template for any kind of interaction.
    /// Interactions can be wholly defined by three things: the concrete Interactor,
    /// the concrete Interactable, and the logic governing their coordination.
    ///
    /// Subclasses are responsible for implementing that coordination logic via template
    /// methods that operate on the concrete interactor and interactable classes.
    /// </summary>
    public abstract class Interactor<TInteractor, TInteractable> : MonoBehaviour, IInteractor
                                    where TInteractor : Interactor<TInteractor, TInteractable>
                                    where TInteractable : Interactable<TInteractor, TInteractable>
    {
        #region Oculus Library Variables and Constants
        private const ulong DefaultNativeId = 0x494e56414c494420;
        protected ulong _nativeId = DefaultNativeId;
        #endregion Oculus Library Methods and Constants

        /// <summary>
        /// An ActiveState whose value determines if the interactor is enabled or disabled.
        /// </summary>
        [Tooltip("An ActiveState whose value determines if the interactor is enabled or disabled.")]
        [SerializeField, Interface(typeof(IActiveState)), Optional]
        private UnityEngine.Object _activeState;
        private IActiveState ActiveState = null;

        /// <summary>
        /// The interactables this interactor can or can't use. Is determined by comparing this interactor's TagSetFilter component(s) to the TagSet component on the interactables.
        /// </summary>
        [Tooltip("The interactables this interactor can or can't use. Is determined by comparing this interactor's TagSetFilter component(s) to the TagSet component on the interactables.")]
        [SerializeField, Interface(typeof(IGameObjectFilter)), Optional]
        private List<UnityEngine.Object> _interactableFilters = new List<UnityEngine.Object>();
        private List<IGameObjectFilter> InteractableFilters = null;

        /// <summary>
        /// Custom logic used to determine the best interactable candidate.
        /// </summary>
        [Tooltip("Custom logic used to determine the best interactable candidate.")]
        [SerializeField, Interface(nameof(CandidateTiebreaker)), Optional]
        private UnityEngine.Object _candidateTiebreaker;
        private IComparer<TInteractable> CandidateTiebreaker;

        private Func<TInteractable> _computeCandidateOverride;
        private bool _clearComputeCandidateOverrideOnSelect = false;
        private Func<bool> _computeShouldSelectOverride;
        private bool _clearComputeShouldSelectOverrideOnSelect = false;
        private Func<bool> _computeShouldUnselectOverride;
        private bool _clearComputeShouldUnselectOverrideOnUnselect;

        protected virtual void DoPreprocess() { }
        protected virtual void DoNormalUpdate() { }
        protected virtual void DoHoverUpdate() { }
        protected virtual void DoSelectUpdate() { }
        protected virtual void DoPostprocess() { }

        public virtual bool ShouldHover
        {
            get
            {
                if (State != InteractorState.Normal)
                {
                    return false;
                }

                return HasCandidate || ComputeShouldSelect();
            }
        }

        public virtual bool ShouldUnhover
        {
            get
            {
                if (State != InteractorState.Hover)
                {
                    return false;
                }

                return _interactable != _candidate || _candidate == null;
            }
        }

        public bool ShouldSelect
        {
            get
            {
                if (State != InteractorState.Hover)
                {
                    return false;
                }

                if (_computeShouldSelectOverride != null)
                {
                    return _computeShouldSelectOverride.Invoke();
                }

                return _candidate == _interactable && ComputeShouldSelect();
            }
        }

        public bool ShouldUnselect
        {
            get
            {
                if (State != InteractorState.Select)
                {
                    return false;
                }

                if (_computeShouldUnselectOverride != null)
                {
                    return _computeShouldUnselectOverride.Invoke();
                }

                return ComputeShouldUnselect();
            }
        }

        protected virtual bool ComputeShouldSelect()
        {
            return QueuedSelect;
        }

        protected virtual bool ComputeShouldUnselect()
        {
            return QueuedUnselect;
        }

        private InteractorState _state = InteractorState.Normal;
        public event Action<InteractorStateChangeArgs> WhenStateChanged = delegate { };
        public event Action WhenPreprocessed = delegate { };
        public event Action WhenProcessed = delegate { };
        public event Action WhenPostprocessed = delegate { };

        private ISelector _selector = null;

        /// <summary>
        /// The maximum number of state changes that can occur per frame. For example, the interactor switching from normal to hover or vice-versa counts as one state change.
        /// </summary>
        [Tooltip("The maximum number of state changes that can occur per frame. For example, the interactor switching from normal to hover or vice-versa counts as one state change.")]
        [SerializeField]
        private int _maxIterationsPerFrame = 3;
        public int MaxIterationsPerFrame
        {
            get
            {
                return _maxIterationsPerFrame;
            }
            set
            {
                _maxIterationsPerFrame = value;
            }
        }

        protected ISelector Selector
        {
            get
            {
                return _selector;
            }

            set
            {
                if (value != _selector)
                {
                    if (_selector != null && _started)
                    {
                        _selector.WhenSelected -= HandleSelected;
                        _selector.WhenUnselected -= HandleUnselected;
                    }
                }

                _selector = value;
                if (_selector != null && _started)
                {
                    _selector.WhenSelected += HandleSelected;
                    _selector.WhenUnselected += HandleUnselected;
                }
            }
        }

        private Queue<bool> _selectorQueue = new Queue<bool>();
        private bool QueuedSelect => _selectorQueue.Count > 0 && _selectorQueue.Peek();
        private bool QueuedUnselect => _selectorQueue.Count > 0 && !_selectorQueue.Peek();

        public InteractorState State
        {
            get
            {
                return _state;
            }
            private set
            {
                if (_state == value)
                {
                    return;
                }
                InteractorState previousState = _state;
                _state = value;

                WhenStateChanged(new InteractorStateChangeArgs(previousState, _state));

                // Update native component
                if (_nativeId != DefaultNativeId && _state == InteractorState.Select)
                {
                    NativeMethods.isdk_NativeComponent_Activate(_nativeId);
                }
            }
        }

        protected TInteractable _candidate;
        protected TInteractable _interactable;
        protected TInteractable _selectedInteractable;

        public virtual object CandidateProperties
        {
            get
            {
                return null;
            }
        }

        public TInteractable Candidate => _candidate;
        public TInteractable Interactable => _interactable;
        public TInteractable SelectedInteractable => _selectedInteractable;

        public bool HasCandidate => _candidate != null;
        public bool HasInteractable => _interactable != null;
        public bool HasSelectedInteractable => _selectedInteractable != null;

        private MultiAction<TInteractable> _whenInteractableSet = new MultiAction<TInteractable>();
        private MultiAction<TInteractable> _whenInteractableUnset = new MultiAction<TInteractable>();
        private MultiAction<TInteractable> _whenInteractableSelected = new MultiAction<TInteractable>();
        private MultiAction<TInteractable> _whenInteractableUnselected = new MultiAction<TInteractable>();
        public MAction<TInteractable> WhenInteractableSet => _whenInteractableSet;
        public MAction<TInteractable> WhenInteractableUnset => _whenInteractableUnset;
        public MAction<TInteractable> WhenInteractableSelected => _whenInteractableSelected;
        public MAction<TInteractable> WhenInteractableUnselected => _whenInteractableUnselected;

        protected virtual void InteractableSet(TInteractable interactable)
        {
            _whenInteractableSet.Invoke(interactable);
        }

        protected virtual void InteractableUnset(TInteractable interactable)
        {
            _whenInteractableUnset.Invoke(interactable);
        }

        protected virtual void InteractableSelected(TInteractable interactable)
        {
            _whenInteractableSelected.Invoke(interactable);
        }

        protected virtual void InteractableUnselected(TInteractable interactable)
        {
            _whenInteractableUnselected.Invoke(interactable);
        }

        private UniqueIdentifier _identifier;
        public int Identifier => _identifier.ID;

        /// <summary>
        /// Can supply additional data (ex. data from an Interactable about a given Interactor, or vice-versa), or pass data along with events like PointerEvent (ex. the associated Interactor generating the event).
        /// </summary>
        [Tooltip("Can supply additional data (ex. data from an Interactable about a given Interactor, or vice-versa), or pass data along with events like PointerEvent (ex. the associated Interactor generating the event).")]
        [SerializeField, Optional]
        private UnityEngine.Object _data = null;
        public object Data { get; protected set; } = null;

        protected bool _started;

        protected virtual void Awake()
        {
            _identifier = UniqueIdentifier.Generate();
            ActiveState = _activeState as IActiveState;
            CandidateTiebreaker = _candidateTiebreaker as IComparer<TInteractable>;
            InteractableFilters =
                _interactableFilters.ConvertAll(mono => mono as IGameObjectFilter);
        }

        protected virtual void Start()
        {
            this.BeginStart(ref _started);

            this.AssertCollectionItems(InteractableFilters, nameof(InteractableFilters));

            if (Data == null)
            {
                if (_data == null)
                {
                    _data = this;
                }
                Data = _data;
            }

            this.EndStart(ref _started);
        }

        protected virtual void OnEnable()
        {
            if (_started)
            {
                if (_selector != null)
                {
                    _selectorQueue.Clear();
                    _selector.WhenSelected += HandleSelected;
                    _selector.WhenUnselected += HandleUnselected;
                }
            }
        }

        protected virtual void OnDisable()
        {
            if (_started)
            {
                if (_selector != null)
                {
                    _selector.WhenSelected -= HandleSelected;
                    _selector.WhenUnselected -= HandleUnselected;
                }
                Disable();
            }
        }

        protected virtual void OnDestroy()
        {
            UniqueIdentifier.Release(_identifier);
        }

        /// <summary>
        /// Overrides the interactor's ComputeCandidate() method with a new method.
        /// <param name="computeCandidate">The method used instead of the interactable's existing ComputeCandidate() method.</param>
        /// <param name="shouldClearOverrideOnSelect">If true, clear the computeCandidate function once you select an interactable.</param>
        /// </summary>
        public virtual void SetComputeCandidateOverride(Func<TInteractable> computeCandidate,
            bool shouldClearOverrideOnSelect = true)
        {
            _computeCandidateOverride = computeCandidate;
            _clearComputeCandidateOverrideOnSelect = shouldClearOverrideOnSelect;
        }

        /// <summary>
        /// Clears the function provided in SetComputeCandidateOverride(). This is called when the interactor force releases an interactable.
        /// </summary>
        public virtual void ClearComputeCandidateOverride()
        {
            _computeCandidateOverride = null;
            _clearComputeCandidateOverrideOnSelect = false;
        }

        /// <summary>
        /// Overrides the interactor's ComputeShouldSelect() method with a new method.
        /// </summary>
        /// <param name="computeShouldSelect">The method used instead of the interactor's existing ComputeShouldSelect() method.</param>
        /// <param name="clearOverrideOnSelect">If true, clear the computeShouldSelect function once you select an interactable.</param>
        public virtual void SetComputeShouldSelectOverride(Func<bool> computeShouldSelect,
            bool clearOverrideOnSelect = true)
        {
            _computeShouldSelectOverride = computeShouldSelect;
            _clearComputeShouldSelectOverrideOnSelect = clearOverrideOnSelect;
        }

        /// <summary>
        /// Clears the function provided in SetComputeShouldSelectOverride(). This is called when the interactor force releases an interactable.
        /// </summary>
        public virtual void ClearComputeShouldSelectOverride()
        {
            _computeShouldSelectOverride = null;
            _clearComputeShouldSelectOverrideOnSelect = false;
        }

        /// <summary>
        /// Overrides the interactor's ComputeShouldUnselect() method with a new method.
        /// </summary>
        /// <param name="computeShouldUnselect">The method used instead of the interactor's existing ComputeShouldUnselect() method.</param>
        /// <param name="clearOverrideOnUnselect">If true, clear the computeShouldUnselect function once you unselect an interactable.</param>
        public virtual void SetComputeShouldUnselectOverride(Func<bool> computeShouldUnselect,
            bool clearOverrideOnUnselect = true)
        {
            _computeShouldUnselectOverride = computeShouldUnselect;
            _clearComputeShouldUnselectOverrideOnUnselect = clearOverrideOnUnselect;
        }

        /// <summary>
        /// Clears the function provided in SetComputeShouldUnselectOverride(). This is called when the interactor unselects an interactable.
        /// </summary>
        public virtual void ClearComputeShouldUnselectOverride()
        {
            _computeShouldUnselectOverride = null;
            _clearComputeShouldUnselectOverrideOnUnselect = false;
        }

        /// <summary>
        /// Executes any logic that should run before the interactor-specific logic. Runs before Process() and Postprocess().
        /// </summary>
        public void Preprocess()
        {
            DoPreprocess();
            if (!UpdateActiveState())
            {
                Disable();
            }
            WhenPreprocessed();
        }

        /// <summary>
        /// Runs interactor-specific logic based on the interactor's current state. Runs after Preprocess() but before Postprocess().
        /// Can be called multiple times per interaction frame.
        /// </summary>
        public void Process()
        {
            switch (State)
            {
                case InteractorState.Normal:
                    DoNormalUpdate();
                    break;
                case InteractorState.Hover:
                    DoHoverUpdate();
                    break;
                case InteractorState.Select:
                    DoSelectUpdate();
                    break;
            }
            WhenProcessed();
        }

        /// <summary>
        ///  Executes any logic that should run after the interactor-specific logic. Runs after both Process() and Preprocess().
        /// </summary>
        public void Postprocess()
        {
            _selectorQueue.Clear();
            DoPostprocess();
            WhenPostprocessed();
        }

        /// <summary>
        /// Determines what the interactable candidate should be.
        /// </summary>
        public virtual void ProcessCandidate()
        {
            _candidate = null;
            if (!UpdateActiveState())
            {
                return;
            }

            if (_computeCandidateOverride != null)
            {
                _candidate = _computeCandidateOverride.Invoke();
            }
            else
            {
                _candidate = ComputeCandidate();
            }
        }

        /// <summary>
        /// Causes the interactor to unselect or unhover an interactable. Called when an interactable is currently selected or hovered but a cancel <cref="IPointerEvent" /> occurs.
        /// </summary>
        public void InteractableChangesUpdate()
        {
            if (_selectedInteractable != null &&
                !_selectedInteractable.HasSelectingInteractor(this as TInteractor))
            {
                UnselectInteractable();
            }

            if (_interactable != null &&
                !_interactable.HasInteractor(this as TInteractor))
            {
                UnsetInteractable();
            }
        }

        /// <summary>
        /// Hovers the current candidate.
        /// </summary>
        public void Hover()
        {
            if (State != InteractorState.Normal)
            {
                return;
            }

            SetInteractable(_candidate);
            State = InteractorState.Hover;
        }

        /// <summary>
        /// Unhovers the current candidate.
        /// </summary>
        public void Unhover()
        {
            if (State != InteractorState.Hover)
            {
                return;
            }

            UnsetInteractable();
            State = InteractorState.Normal;
        }

        /// <summary>
        /// Selects the target interactable and sets the interactor's state to select.
        /// </summary>
        public virtual void Select()
        {
            if (State != InteractorState.Hover)
            {
                return;
            }

            if (_clearComputeCandidateOverrideOnSelect)
            {
                ClearComputeCandidateOverride();
            }

            if (_clearComputeShouldSelectOverrideOnSelect)
            {
                ClearComputeShouldSelectOverride();
            }

            while (QueuedSelect)
            {
                _selectorQueue.Dequeue();
            }

            if (Interactable != null)
            {
                SelectInteractable(Interactable);
            }

            State = InteractorState.Select;
        }

        /// <summary>
        /// Unselects the currently selected interactable and sets the interactor's state to hover.
        /// </summary>
        public virtual void Unselect()
        {
            if (State != InteractorState.Select)
            {
                return;
            }
            if (_clearComputeShouldUnselectOverrideOnUnselect)
            {
                ClearComputeShouldUnselectOverride();
            }
            while (QueuedUnselect)
            {
                _selectorQueue.Dequeue();
            }
            UnselectInteractable();

            State = InteractorState.Hover;
        }

        // Returns the best interactable for selection or null
        protected abstract TInteractable ComputeCandidate();

        protected virtual int ComputeCandidateTiebreaker(TInteractable a, TInteractable b)
        {
            if (CandidateTiebreaker == null)
            {
                return 0;
            }

            return CandidateTiebreaker.Compare(a, b);
        }

        /// <summary>
        /// Determines if an interactor can interact with an interactable.
        /// </summary>
        /// <param name="interactable">The interactable to check against.</param>
        /// <returns>True if the interactor can interact with the given interactable.</returns>
        public virtual bool CanSelect(TInteractable interactable)
        {
            if (InteractableFilters == null)
            {
                return true;
            }

            foreach (IGameObjectFilter interactableFilter in InteractableFilters)
            {
                if (!interactableFilter.Filter(interactable.gameObject))
                {
                    return false;
                }
            }

            return true;
        }

        private void SetInteractable(TInteractable interactable)
        {
            if (_interactable == interactable)
            {
                return;
            }
            UnsetInteractable();
            _interactable = interactable;
            interactable.AddInteractor(this as TInteractor);
            InteractableSet(interactable);
        }

        private void UnsetInteractable()
        {
            TInteractable interactable = _interactable;
            if (interactable == null)
            {
                return;
            }
            _interactable = null;
            interactable.RemoveInteractor(this as TInteractor);
            InteractableUnset(interactable);
        }

        private void SelectInteractable(TInteractable interactable)
        {
            Unselect();
            _selectedInteractable = interactable;
            interactable.AddSelectingInteractor(this as TInteractor);
            InteractableSelected(interactable);
        }

        private void UnselectInteractable()
        {
            TInteractable interactable = _selectedInteractable;

            if (interactable == null)
            {
                return;
            }

            _selectedInteractable = null;
            interactable.RemoveSelectingInteractor(this as TInteractor);
            InteractableUnselected(interactable);
        }

        /// <summary>
        /// Enables a disabled interactor.
        /// </summary>
        public void Enable()
        {
            if (!UpdateActiveState())
            {
                return;
            }

            if (State == InteractorState.Disabled)
            {
                State = InteractorState.Normal;
                HandleEnabled();
            }
        }

        /// <summary>
        /// Disables an interactor.
        /// </summary>
        public void Disable()
        {
            if (State == InteractorState.Disabled)
            {
                return;
            }

            HandleDisabled();

            if (State == InteractorState.Select)
            {
                UnselectInteractable();
                State = InteractorState.Hover;
            }

            if (State == InteractorState.Hover)
            {
                UnsetInteractable();
                State = InteractorState.Normal;
            }

            if (State == InteractorState.Normal)
            {
                State = InteractorState.Disabled;
            }
        }

        protected virtual void HandleEnabled() {}
        protected virtual void HandleDisabled() {}

        protected virtual void HandleSelected()
        {
            _selectorQueue.Enqueue(true);
        }

        protected virtual void HandleUnselected()
        {
            _selectorQueue.Enqueue(false);
        }

        private bool UpdateActiveState()
        {
            if (ActiveState != null)
            {
                return ActiveState.Active;
            }
            return this.enabled;
        }

        public bool IsRootDriver { get; set; } = true;

        protected virtual void Update()
        {
            if (!IsRootDriver)
            {
                return;
            }

            Drive();
        }

        /// <summary>
        /// Coordinates all of the interactor's interaction logic.
        /// </summary>
        public virtual void Drive()
        {
            Preprocess();

            if (!UpdateActiveState())
            {
                Disable();
                Postprocess();
                return;
            }

            Enable();

            InteractorState previousState = State;
            for (int i = 0; i < MaxIterationsPerFrame; i++)
            {
                if (State == InteractorState.Normal ||
                    (State == InteractorState.Hover && previousState != InteractorState.Normal))
                {
                    ProcessCandidate();
                }
                previousState = State;

                Process();

                if (State == InteractorState.Disabled)
                {
                    break;
                }

                if (State == InteractorState.Normal)
                {
                    if (ShouldHover)
                    {
                        Hover();
                        continue;
                    }
                    break;
                }

                if (State == InteractorState.Hover)
                {
                    if (ShouldSelect)
                    {
                        Select();
                        continue;
                    }
                    if (ShouldUnhover)
                    {
                        Unhover();
                        continue;
                    }
                    break;
                }

                if (State == InteractorState.Select)
                {
                    if (ShouldUnselect)
                    {
                        Unselect();
                        continue;
                    }
                    break;
                }
            }

            Postprocess();
        }

        #region Inject

        /// <summary>
        /// Sets an IActiveState for the interactor on a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalActiveState(IActiveState activeState)
        {
            _activeState = activeState as UnityEngine.Object;
            ActiveState = activeState;
        }

        /// <summary>
        /// Sets an set of interactable filters for the interactor on a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalInteractableFilters(List<IGameObjectFilter> interactableFilters)
        {
            InteractableFilters = interactableFilters;
            _interactableFilters = interactableFilters.ConvertAll(interactableFilter =>
                                    interactableFilter as UnityEngine.Object);
        }

        /// <summary>
        /// Sets a candidate tiebreaker for the interactor on a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalCandidateTiebreaker(IComparer<TInteractable> candidateTiebreaker)
        {
            _candidateTiebreaker = candidateTiebreaker as UnityEngine.Object;
            CandidateTiebreaker = candidateTiebreaker;
        }

        /// <summary>
        /// Sets data for the interactor on a dynamically instantiated GameObject.
        /// </summary>
        public void InjectOptionalData(object data)
        {
            _data = data as UnityEngine.Object;
            Data = data;
        }

        #endregion
    }
}
