%trackingIMM Interacting Multiple Model (IMM) filter for tracking
%   The Interacting Multiple Model filter is designed for tracking objects
%   that are highly maneuvering. You can use the filter to predict an
%   object's future location, to reduce noise in the detected location, or
%   to help associate multiple object detections with their tracks.
%
%   Interacting Multiple Model filter deals with the multiple motion models
%   in the Bayesian framework. This method resolves the target motion
%   uncertainty by using multiple models at a time for a maneuvering
%   target. The IMM algorithm processes all the models simultaneously and
%   switches between models according to their updated weights.
%
%   IMM = trackingIMM returns an Interacting Multiple Model filter object
%   with default tracking filters {trackingEKF,trackingEKF,trackingEKF}
%   with the following motion models: constant velocity, constant
%   acceleration, constant turn respectively.It also uses default model
%   conversion function (@switchimm).The default model transition
%   probability is set to 0.9 along the diagonal, which means a 0.9
%   probability of not switching a motion model, and a probability of
%   (1-0.9)/2 of switching from one motion model to a different motion
%   model.
%
%   IMM = trackingIMM(TrackingFilters) lets you specify the tracking
%   filters and uses other properties as default to generate the
%   Interacting Multiple Model filter object.
%
%   IMM = trackingIMM(TrackingFilters,ModelConversionFcn) lets you specify
%   the tracking filters, model conversion function and uses other
%   properties as default to generate the Interacting Multiple Model filter
%   object.
%
%   IMM = trackingIMM(TrackingFilters,ModelConversionFcn,TransitionProbabilities)
%   lets you specify the tracking filters, model conversion function and
%   transitionProbabilities to generate the Interacting Multiple Model
%   filter object.
%
%   IMM = trackingIMM(..., Name, Value) configures the IMM filter object
%   properties, specified as one or more name-value pair arguments.
%   Unspecified properties have default values.
%
%   trackingIMM properties:
%   State                   - (Read-only) State, (x)
%   StateCovariance         - (Read-only) State estimation error covariance, (P)
%   TrackingFilters         - (Read-only) A list of filters used for each model
%   HasMeasurementWrapping  - (read only) Flags to show if TrackingFilters use wrapping
%   ModelConversionFcn      - Coverts the State or StateCovariance
%   TransitionProbabilities - Probability of filter model transitions
%   MeasurementNoise        - Measurement noise covariance, (R)
%   ModelProbabilities      - (Read-only) Probability of each model
%   EnableSmoothing         - Enable backward smoothing for the filter
%   MaxNumSmoothingSteps    - Maximum number of backward smoothing steps
%                             allowed for smoothing
%   MaxNumOOSMSteps         - Maximum number of out-of-sequence steps
%
%   trackingIMM methods:
%   predict          - Predict the state and state estimation error covariance
%   correct          - Correct the state and state estimation error covariance
%   correctjpda      - Correct using joint probabilistic detection assignment
%   smooth           - Smooth the filter estimate using backward recursion
%   retrodict        - Predict the filter backwards in time
%   retroCorrect     - Correct the filter with an out-of-sequence measurement
%   retroCorrectJPDA - Correct the filter with multiple out-of-sequence measurements
%   distance         - Calculate the distance between measurements and the filter
%   likelihood       - Calculate the likelihood of a measurement
%   clone            - Create a copy of the object with the same property values
%   initialize       - Initialize filter state and state covariance 
%
%   Notes:
%   ------
%   * If the measurement exists, e.g., the object has been detected, you
%     can call the predict method and the correct method together. If the
%     measurement is missing, you can call the predict method but not the
%     correct method.
%   * You can use the distance method to compute distances that describe
%     how a set of measurements matches the Interacting Multiple Model
%     filter. You can thus find a measurement that best fits the filter.
%     This strategy can be used for matching object detections against
%     object tracks in a multi-object tracking problem.
%
%   % Example 1: Create a trackingIMM with three motion models
%   % ------------------------------------------------------
%   % Construct a trackingIMM filter with three motion models: constant
%   % velocity, constant acceleration and constant turn rate.
%   detection = objectDetection(0, [1;3;0], 'MeasurementNoise', [1 0.2 0; 0.2 2 0; 0 0 1]);
%   filter = {initcvekf(detection);initcaekf(detection);initctekf(detection)};
%   imm = trackingIMM(filter);
%
%   % Example 2: Create a trackingIMM with additional name-value pairs
%   % ----------------------------------------------------------------
%   % Use additional name-value pairs to define additional properties
%   % like the model conversion function or the transition probabilities.
%   detection = objectDetection(0, [1;3;0]);
%   filter = {initcvekf(detection);initctekf(detection)};
%   modelConv = @switchimm;
%   transProb = 0.9;
%   imm = trackingIMM('State',[1;2;0;4;5;0],'StateCovariance',eye(6),...
%         'TrackingFilters',filter,'TransitionProbabilities',transProb,...
%         'MeasurementNoise', [1 0.2 0; 0.2 2 0; 0 0 1],'ModelConversionFcn',modelConv);
%
%   % Example 3: Running the filter
%   % -----------------------------
%   %   After constructing a filter, use the following steps to call
%   %   predict and correct. You may call predict and correct in any order
%   %   and as many times you would like to call each.
%   detection = objectDetection(0, [1;1;0], 'MeasurementNoise', [1 0.2 0; 0.2 2 0; 0 0 1]);
%   filter = {initctekf(detection);initcvekf(detection)};
%   modelConv = @switchimm;
%   transProb = [0.9,0.9];
%   imm = trackingIMM('State',[1;1;3;1;5;1;1],'StateCovariance',eye(7),...
%        'TransitionProbabilities',transProb,'TrackingFilters',filter,...
%       'ModelConversionFcn',modelConv);
%   meas = [1;1;0];
%   [x_pred, P_pred] = predict(imm, 0.5)    % Predict over 0.5 seconds
%   [x_corr, P_corr] = correct(imm, meas)   % Correct using the measurement
%   [x_pred, P_pred] = predict(imm)         % Predict over 1 second
%   [x_pred, P_pred] = predict(imm, 2)      % Predict over 2 seconds
%
%   % Example 4: Using the distance method
%   % ------------------------------------
%   %   You can measure the distance between the filter and a set of
%   %   measurements, z_matrix, by calling the distance method.
%   detection = objectDetection(0, [1;1;0], 'MeasurementNoise', [1 0.2 0; 0.2 2 0; 0 0 1]);
%   filter = {initcvekf(detection);initctekf(detection)};
%   modelConv = @switchimm;
%   transProb = [0.9 0.1; 0.1 0.9];
%   imm = trackingIMM(filter,modelConv,transProb);
%   [x_pred, P_pred] = predict(imm);
%   z_matrix = [0, 0, 0; 1, 1, 0; 2, 2, 0];
%   d = distance(imm, z_matrix)
%
%  See also: initekfimm, switchimm, trackingKF, trackingEKF, trackingUKF,
%  trackingCKF, trackingPF, trackingGSF, constvel, constacc, constturn.

%   References:
%
%   [1] Samuel Blackman and Robert Popoli, "Design and Analysis of Modern
%       Tracking Systems", Artech House, 1999.
%   [2] Yaakov Bar-Shalom and Huimin Chen, "IMM Estimator with
%       Out-of-Sequence Measurements", IEEE Transactions on Aerospace and 
%       Electronic Systems, pp 90--98, Vol. 41, No. 1, January 2005.

%   Copyright 2018-2021 The MathWorks, Inc.

%#codegen

classdef trackingIMM < matlabshared.tracking.internal.AbstractTrackingFilter...
        & matlabshared.tracking.internal.fusion.CustomDisplay ...
        & matlabshared.tracking.internal.AbstractJPDAFilter ...
        & fusion.internal.IMMSmoother ...
        & matlabshared.tracking.internal.RetrodictionFilter
    
    properties(Dependent, SetAccess=private, GetAccess=public)
        %TrackingFilters
        %   Specify an L-element cell array of filter objects. This
        %   property is read only and can only be set on construction.
        %
        %   Default: Cell array of three trackingEKF filter objects with
        %   the following motion models: constant velocity, constant
        %   acceleration and constant turn respectively.
        TrackingFilters;
    end
    
    properties(Access=public)
        %ModelConversionFcn convert State or StateCovariance
        %   Specify a function handle to the Model conversion
        %   function. Which converts from (i-th) filter to (j-th) filter.
        %
        %   Default: @switchimm
        ModelConversionFcn;
    end
    
    properties(Dependent)
        %TransitionProbabilities Probability of filter model transitions
        %   Specify a real scalar, a L-by-1 real vector, or an L-by-L real
        %   matrix of transition probabilities, per unit time, dt = 1.
        %
        %   When specified as a scalar, defines the probability of staying
        %   in each filter model. The remaining transition probability
        %   (1-P) is distributed evenly across the L-1 motion models and
        %   defines the probability of transitioning out of a filter model
        %   into one of the L-1 other filter models.
        %
        %   When specified as an L-by-1 vector, each element (j) defines
        %   the probability of staying in the j-th filter model. The
        %   remaining transition probability (1-P) is distributed evenly
        %   across the L-1 motion models and defines the probability of
        %   transitioning out of the j-th filter model.
        %
        %   When specified as an L-by-L matrix, the element (j, k) defines
        %   the probability of transition from the j-th filter model to the
        %   k-th filter model. All elements must lie on the interval [0,1]
        %   and each row must sum to 1.
        %
        %   Default: 0.9
        TransitionProbabilities;
    end
    
    properties(Dependent, SetAccess=private, GetAccess=public)
        %ModelProbabilities Probability of each tracking filter
        %   Specify a L-by-1 real vector of model Probabilities. Initially,
        %   all models are equiprobable. This property is read only.
        %
        %   Default: 1/L
        ModelProbabilities;
    end
    
    properties(Dependent, SetAccess=private, GetAccess=public)
        %State The state (x)
        %   Specify the state of size M. Where M is the size of first
        %   tracking filter state. This property is read only and can only
        %   be set on construction.
        %
        %   Default: zeros(M)(Vector)
        State;
        %StateCovariance State estimation error covariance (P)
        %   Specify the covariance of the state estimation error of size
        %   M-by-M. Where M-by-M is the size of first tracking filter state
        %   covariance. This property is read only and can only be set on
        %   construction.
        %
        %   Default: eye(M,'like',State)
        StateCovariance;
    end
    
    properties(Dependent, SetAccess=public)
        %MeasurementNoise Measurement noise covariance (R)
        %   Specify the covariance of measurement noise as a scalar or an
        %   N-by-N matrix, where N is the number of measurements. If you
        %   specify it as a scalar it will be extended to an N-by-N diagonal
        %   matrix.
        %
        %   Default: 1
        MeasurementNoise
    end

    properties(SetAccess=protected,Dependent)
        %HasMeasurementWrapping Flags to show if TrackingFilters use wrapping
        % A 1-by-N logical array. The i-th element of the array is true if
        % the i-th filter in TrackingFilters supports wrapping measurement.
        HasMeasurementWrapping 
    end
    
    properties(Access=protected)
        pState
        pStateCovariance
        pTrackingFilters
        pTransitionProbabilities
        pMeasurementNoise
        pModelProbabilities
        pNumModels    %Number of motion model in use
        pIsLinearKalmanFilter
        pLastRetrodictionDT
    end
    
    properties(Constant, Access=private)
        defaultTransitionProbabilities = 0.9;
    end
    
    methods
        function IMM = trackingIMM(varargin)
            %-------------------------------------------------------------
            % Constructor
            %-------------------------------------------------------------
            [smoothArgs,idx1,idx2] = trackingIMM.parseSmoothingNVPairs(varargin{:});
            vArgsNoSmoothing = {varargin{1:idx1-1},varargin{idx1 + 2:idx2 - 1},varargin{idx2 + 2:end}};
            [oosmArgs,idx1] = trackingEKF.parseOOSMNVPairs(vArgsNoSmoothing{:});
            filterArgs = {vArgsNoSmoothing{1:idx1-1},vArgsNoSmoothing{idx1 + 2:end}};
            IMM@fusion.internal.IMMSmoother(smoothArgs{:});
            % Parse constructor inputs
            [state,stateCov,trackingFilters, transitionProbabilities, modelConversion,...
                modelProb,measNoise] = parseInputs(IMM, filterArgs{:});

            numModels = numel(trackingFilters);
            IMM.pNumModels = numModels;
            IMM.TrackingFilters         = trackingFilters;
            IMM.TransitionProbabilities = transitionProbabilities;
            IMM.ModelConversionFcn      = modelConversion;
            if ~isempty(measNoise)
                IMM.MeasurementNoise = measNoise;
            end
            classToUse = class(IMM.pTrackingFilters{1}.State);
            if isempty(modelProb)
                IMM.pModelProbabilities = ones(numModels, 1, 'like', IMM.TransitionProbabilities)/numModels;
            else
                % Validate user-provided model probabilities by setting
                % public property
                IMM.ModelProbabilities = cast(modelProb, classToUse);
            end
            m = numel(IMM.pTrackingFilters{1}.State);
            if(isempty(state)&&isempty(stateCov))
                % Calculate initial State and StateCovariance if State and
                % StateCovariance are not given in input.
                combineModels(IMM);
            else
                if(isempty(state) && ~isempty(stateCov))
                    % When only State is given in input parameters.
                    newState = cast(matlabshared.tracking.internal.expandScalarValue(0,[m,1]),classToUse);
                    newStateCov = cast(matlabshared.tracking.internal.expandScalarValue(stateCov,[m,m]),classToUse);
                elseif(~isempty(state) && isempty(stateCov))
                    % When only StateCovariance is given in input
                    % parameters.
                    newState = cast(matlabshared.tracking.internal.expandScalarValue(state,[m,1]),classToUse);
                    newStateCov = cast(matlabshared.tracking.internal.expandScalarValue(1,[m,m]),classToUse);
                else
                    % When State and StateCovariance is given in input
                    % parameters.
                    newState = cast(matlabshared.tracking.internal.expandScalarValue(state,[m,1]),classToUse);
                    newStateCov = cast(matlabshared.tracking.internal.expandScalarValue(stateCov,[m,m]),classToUse);
                end
                IMM.State = newState;
                IMM.StateCovariance = newStateCov;
                % initializes the underlying filters according to the State
                % and StateCovariance given in input parameters.
                initialize(IMM,newState,newStateCov);
            end

            % Setup the OOSM memory after all the construction is done
            if ~isempty(oosmArgs)
                if oosmArgs{2} > 0
                    IMM.MaxNumOOSMSteps = oosmArgs{2};
                end
            end
        end
        
        %-----------------------------------------------------------------
        function set.State(IMM, value)
            % Validating that the state
            validateattributes(value, ...
                {'double','single'}, {'real', 'finite', 'nonsparse', 'vector'},...
                'trackingIMM', 'State');
            % Validate State dimensions
            % Number of elements in trackingIMM State must always be equal
            % to number of elements in it's first underlying filter state.
            if coder.internal.is_defined(IMM.pTrackingFilters{1}.State)
                % Number of element in first underlying filter state
                m = numel(IMM.pTrackingFilters{1}.State);
                classToUse = class(IMM.pTrackingFilters{1}.State);
                value = cast(value,classToUse);
                coder.internal.assert(numel(value) == m, ...
                    'fusion:trackingIMM:invalidStateDims', 'State', m);
            end
            IMM.pState = value(:);
        end
        
        %-----------------------------------------------------------------
        function value = get.State(IMM)
            value = IMM.pState;
        end
        
        %-----------------------------------------------------------------
        function set.StateCovariance(IMM, value)
            % Validate state covariance
            validateattributes(value, ...
                {'double','single'}, ...
                {'real', 'finite', 'nonsparse', '2d', 'nonempty', 'square'},...
                'trackingIMM', 'StateCovariance');
            % Validate StateCovariance dimensions
            if coder.internal.is_defined(IMM.pTrackingFilters{1}.State)
                % Number of element in first underlying filter state
                m = numel(IMM.pTrackingFilters{1}.State);
                classToUse = class(IMM.pTrackingFilters{1}.State);
                value = cast(value,classToUse);
                coder.internal.assert(size(value,1) == m, ...
                    'fusion:trackingIMM:invalidStateCovDims', 'StateCovariance', m);
            end
            matlabshared.tracking.internal.isSymmetricPositiveSemiDefinite...
                ('StateCovariance', value);
            IMM.pStateCovariance = value;
        end
        
        %-----------------------------------------------------------------
        function value = get.StateCovariance(IMM)
            value = IMM.pStateCovariance;
        end

        %-----------------------------------------------------------------
        function set.TrackingFilters(IMM,value)
            validateattributes(value,{'cell'}, ...
                {'nonempty'},'trackingIMM','TrackingFilters');
            % check if the input is not a multi model
            coder.internal.errorIf(IMM.pNumModels <= 1, ...
                'fusion:trackingIMM:invalidMultiModel','TrackingFilters');
            IMM.pTrackingFilters = cell(IMM.pNumModels,1);
            isLinearFilter = false(1,IMM.pNumModels);
            for k = coder.unroll(1:IMM.pNumModels)
                % check if the input is a valid tracking filter for
                % trackingIMM
                isMMCompatible = fusion.internal.isMultiModelCompatible(value{k});
                coder.internal.errorIf(~isMMCompatible(2), ...
                    'fusion:trackingIMM:invalidFilterType',class(value{k}));
                cond = ~isequal(class(value{1}.State),class(value{k}.State));
                % check if type of state in all the TrackingFilters are same.
                coder.internal.errorIf(cond, ...
                    'fusion:trackingIMM:invalidStateType','TrackingFilters',class(value{k}.State));
                IMM.pTrackingFilters{k} = clone(value{k});
                isLinearFilter(k) = matlabshared.tracking.internal.isLinearKalmanFilter(IMM.pTrackingFilters{k});
            end
            IMM.pIsLinearKalmanFilter = isLinearFilter;
        end
        
        %-----------------------------------------------------------------
        function val = get.TrackingFilters(IMM)
            val = IMM.pTrackingFilters;
        end
        
        %-----------------------------------------------------------------
        function set.TransitionProbabilities(IMM,value)
            validateattributes(value,{'double','single'}, ...
                {'real', 'finite', 'nonsparse','nonempty','nonnegative','<=',1},'trackingIMM',...
                'TransitionProbabilities');
            classToUse = class(IMM.pTrackingFilters{1}.State);
            numModels = IMM.pNumModels;
            if isscalar(value)
                offDiag = cast((1 - value)/(numModels -1),classToUse);
                transProb = offDiag*ones(numModels,classToUse);
                idx = 1:(numModels + 1):numModels*numModels;
                transProb(idx) = value;
            elseif isvector(value)
                % validating the TransitionProbabilities vector size
                coder.internal.errorIf(~isequal(numel(value),IMM.pNumModels), ...
                    'fusion:trackingIMM:invalidNumProbVector','TransitionProbabilities',IMM.pNumModels);
                transProb = zeros(numModels,classToUse);
                for j = 1:numModels
                    transProb(j,:) = (1-value(j))/(numModels-1);
                    transProb(j,j) = value(j);
                end
            else
                validateattributes(value,{'double','single'}, ...
                    {'2d','square'},'trackingIMM', 'TransitionProbabilities');
                
                % validating the TransitionProbabilities matrix size
                coder.internal.errorIf(~isequal(size(value,1),IMM.pNumModels), ...
                    'fusion:trackingIMM:invalidNumProbMatrix','TransitionProbabilities',IMM.pNumModels);
                % validating that the TransitionProbabilities matrix row sum should not be greater then 1
                cond = ~(sum(abs(sum(value,2)-ones(IMM.pNumModels,1,classToUse)))<cast(1e-5,classToUse));
                coder.internal.errorIf(cond, ...
                    'fusion:trackingIMM:invalidTransProbMatrix','TransitionProbabilities');
                transProb = value;
            end
            IMM.pTransitionProbabilities = cast(transProb,classToUse);
        end
        
        %-----------------------------------------------------------------
        function val = get.TransitionProbabilities(IMM)
            val = IMM.pTransitionProbabilities;
        end
        
        %-----------------------------------------------------------------
        function set.ModelConversionFcn(IMM, value)
            validateattributes(value, {'function_handle'},...
                {'nonempty'}, 'trackingIMM', 'ModelconversionFcn');
            IMM.ModelConversionFcn = value;
        end
        
        %-----------------------------------------------------------------
        function val = get.ModelConversionFcn(IMM)
            val = IMM.ModelConversionFcn;
        end
        
        %-----------------------------------------------------------------
        function set.ModelProbabilities(IMM,value)
            validateattributes(value, ...
                {'double','single'}, ...
                {'real', 'finite', 'nonsparse','nonempty','vector','numel',IMM.pNumModels},...
                'trackingIMM','ModelProbabilities');
            value = value/sum(value);
            IMM.pModelProbabilities = value;
        end
        
        %-----------------------------------------------------------------
        function val = get.ModelProbabilities(IMM)
            val = IMM.pModelProbabilities;
        end
        
        function set.pModelProbabilities(IMM, val)
            % When setting the pModelProbabilities, the IMM filter respects
            % the original orientation of the pModelProbabilities vector. 
            if coder.internal.is_defined(IMM.pModelProbabilities)
                % All subsequent assignments respect the orientation and
                % data type of the vector
                IMM.pModelProbabilities(:) = cast(val,'like',IMM.pModelProbabilities);
            else
                % The first assignment defines the orientation of the
                % vector.
                IMM.pModelProbabilities = val;
            end
        end
        
        %-----------------------------------------------------------------
        function set.MeasurementNoise(IMM, value)
            validateattributes(value, ...
                {'single','double'}, ...
                {'real', 'finite', 'nonsparse', '2d', 'nonempty', 'square'},...
                'trackingIMM', 'MeasurementNoise');
            classToUse = class(IMM.pTrackingFilters{1}.State);
            matlabshared.tracking.internal.isSymmetricPositiveSemiDefinite...
                ('MeasurementNoise', value);
            for i = coder.unroll(1:IMM.pNumModels)
                IMM.pTrackingFilters{i}.MeasurementNoise = cast(value,classToUse);
            end
        end
        
        %-----------------------------------------------------------------
        function value = get.MeasurementNoise(IMM)
            value = IMM.pTrackingFilters{1}.MeasurementNoise;
        end

        %-----------------------------------------------------------------
        function value = get.HasMeasurementWrapping(IMM)
            value = zeros(1,IMM.pNumModels,'logical');
            for i = coder.unroll(1:IMM.pNumModels)
                if isprop(IMM.pTrackingFilters{i}, 'HasMeasurementWrapping')
                    value(i) = IMM.pTrackingFilters{i}.HasMeasurementWrapping;
                end
            end
        end
        
        %-----------------------------------------------------------------
        % initialize method
        %-----------------------------------------------------------------
        function initialize(IMM, state, stateCov, varargin)
            %INITIALIZE  Initialize filter state and state covariance
            %   INITIALIZE(IMM, state, stateCov) allows you to initialize
            %   the IMM filter to a certain state and state covariance.
            %   Each model will be initialized to the corresponding state
            %   and state covariance using the appropriate model conversion
            %   function.
            %
            %   INITIALIZE(IMM, state, stateCov, Name, value) allows you to
            %   initialize other properties of the IMM filter using
            %   Name,value pair arguments. The supported properties are
            %   ModelProbabilities, TransitionProbabilities and
            %   MeasurementNoise.
            
            % We assume that the state and state covariance are given in
            % terms of the first model. Then convert to each of the other
            % models and initialize those
            
            % initialize must be called with a single object
            coder.internal.errorIf(numel(IMM) > 1, ...
                'fusion:trackingIMM:NonScalarFilter', ...
                'trackingIMM', 'initialize');
            
            stateSize = numel(IMM.pTrackingFilters{1}.State);
            index = (1:stateSize);
            s = state(index');
            sc = stateCov(1:stateSize,1:stateSize);
            
            givenModel = 1;
            if(isa(IMM.TrackingFilters{givenModel},'trackingPF'))
                initialize(IMM.TrackingFilters{givenModel}, IMM.TrackingFilters{givenModel}.NumParticles, s, sc);
            else
                IMM.TrackingFilters{givenModel}.State = s;
                IMM.TrackingFilters{givenModel}.StateCovariance = sc;
            end
            
            modelNames = cell(IMM.pNumModels,1);
            for i = coder.unroll(1:IMM.pNumModels)
                modelNames{i} = IMM.TrackingFilters{i}.modelName;
            end
            
            for mdlOut = coder.unroll(2:IMM.pNumModels)
                Xzeros = zeros(size(IMM.TrackingFilters{mdlOut}.State),'like',IMM.TrackingFilters{mdlOut}.State);
                Pzeros = zeros(size(IMM.TrackingFilters{mdlOut}.StateCovariance),'like',IMM.TrackingFilters{mdlOut}.StateCovariance);
                Xk = IMM.ModelConversionFcn(modelNames{givenModel}, s, modelNames{mdlOut}, Xzeros);
                Pk = IMM.ModelConversionFcn(modelNames{givenModel}, sc, modelNames{mdlOut}, Pzeros);
                if(isa(IMM.TrackingFilters{mdlOut},'trackingPF'))
                    initialize(IMM.TrackingFilters{mdlOut}, IMM.TrackingFilters{mdlOut}.NumParticles, Xk, Pk);
                else
                    IMM.TrackingFilters{mdlOut}.State = Xk;
                    IMM.TrackingFilters{mdlOut}.StateCovariance = Pk;
                end
            end
            combineModels(IMM);
            
            % Set N/V pairs provided by trackingIMM.
            coder.internal.assert(mod(numel(varargin),2) == 0,'MATLAB:system:invalidPVPairs');
            for i = 1:2:numel(varargin)
                param = validatestring(varargin{i},...
                    {'ModelProbabilities','TransitionProbabilities','MeasurementNoise'},mfilename);
                IMM.(param) = varargin{i+1};
            end
        end
        
        %-----------------------------------------------------------------
        % Predict method
        %-----------------------------------------------------------------
        function [x_pred, P_pred] = predict(IMM, varargin)
            % predict Predicts the state and state estimation error covariance
            %   [x_pred,P_pred] = predict(IMM) returns x_pred, the
            %   prediction of the state, and P_pred, the prediction of the
            %   state estimation error covariance at the next time step.
            %   The internal state and covariance of IMM filter and the
            %   filter used in it are overwritten by the prediction results.
            %
            %   [x_pred, P_pred] = predict(IMM, varargin) additionally,
            %   lets you specify additional parameters that will be used by
            %   StateTransitionFcn of the tracking filter under use by the
            %   IMM. The first element of varargin must be dt.
            
            % predict must be called with a single object
            coder.internal.errorIf(numel(IMM) > 1, ...
                'fusion:trackingIMM:NonScalarFilter', ...
                'trackingIMM', 'predict');
            
            if nargin<2
                dt = ones(1,'like',IMM.pState);
            else
                validateattributes(varargin{1},{'double','single'},...
                    {'real','finite','nonsparse','nonnegative','scalar'}, ...
                    'predict', 'dt');
                dt = varargin{1};
            end
            % Call setup on smoother, the smoother knows when to return
            % safely without doing setup multiple times.
            setupInitialDistributions(IMM);
            % mixing and interactions of filters to compute the initial
            % state.
            mixModels(IMM, dt);
            % Predict each of the tracking filter models forward by using
            % their mixed state and state covariance
            for kMdl = coder.unroll(1:IMM.pNumModels)
                filterk = IMM.pTrackingFilters{kMdl};
                predict(filterk, varargin{:} );
            end
            
            % Output the combined state from all of the filter models
            [x_pred, P_pred] = combineModels(IMM);
           
            % Update prediction data
            IMM.LastStepTime = cast(dt,'like',IMM.LastStepTime);
            updatePredictionData(IMM);

            % Update the data for the OOSM filter
            updateRetrodictionStateAfterPrediction(IMM, dt);
        end
        
        %-----------------------------------------------------------------
        % Correct method
        %-----------------------------------------------------------------
        function [x_corr, P_corr, mdl_probs] = correct(IMM, z, varargin)
            % correct Corrects the state and state estimation error
            %   covariance of the IMM filter and the filter used in it.
            %   [x_corr,P_corr] = correct(IMM, z) returns x_corr, the
            %   correction of the state, and P_corr, the correction of the
            %   state estimation error covariance based on the current
            %   measurement z, an N-element vector. The internal state and
            %   covariance of the IMM filter and the filter used in it are
            %   overwritten by the corrected values.
            %
            %   [x_corr, P_corr] = correct(IMM, z, varargin) additionally
            %   allows the definition of parameters used by the
            %   MeasurementFcn in addition to IMM.State. For example, the
            %   sensor's location.

            narginchk(2,inf);
            
            % correct must be called with a single object
            coder.internal.errorIf(numel(IMM) > 1, ...
                'fusion:trackingIMM:NonScalarFilter', ...
                'trackingIMM', 'correct');
            
            likelihoods = zeros(IMM.pNumModels,1,'like',IMM.pModelProbabilities);
            for kMdl = coder.unroll(1:IMM.pNumModels)
                filterk = IMM.pTrackingFilters{kMdl};
                
                % Compute model likelihood (i.e. how "statistically" close
                % was the prediction to the measurement.)
                if IMM.pIsLinearKalmanFilter(kMdl)
                    likelihoods(kMdl) = likelihood(filterk, z);
                else
                    likelihoods(kMdl) = likelihood(filterk, z, varargin);
                end
                
                correct(filterk, z,varargin{:});
            end
            
            % Update model probabilities
            cbar = IMM.pModelProbabilities;
            mdl_probs = cbar(:).*likelihoods(:);
            mdl_probs = mdl_probs/sum(mdl_probs);
            IMM.pModelProbabilities = mdl_probs;
            
            % Output the combined state from all of the filter models
            [x_corr, P_corr] = combineModels(IMM);
            
            % Update correction data
            updateCorrectionData(IMM);

            % Update the OOSM filter.
            updateHistoryAfterCorrection(IMM);
        end
        
        function [x_corr, P_corr, mdl_probs] = correctjpda(IMM, z, jpda, varargin)
            %CORRECTJPDA Correct using joint probabilistic detection assignment
            %   Corrects the state and state error covariance with
            %   a set of measurements and their probabilistic data association
            %   coefficients.
            %
            %   [x_corr, P_corr] = correctjpda(filter, z, jpda) returns the
            %   correction of state, x_corr, and state estimation error
            %   covariance, P_corr, based on the current set of measurements
            %   z and their joint probabilistic data association
            %   coefficients jpda.
            %
            %   Inputs:
            %           - filter   filter of the class trackingIMM
            %
            %           - z        measurements matrix of size m-M  where m
            %                      is the dimension of a measurement and M
            %                      is the number of measurements.
            %
            %           - jpda     M+1 vector of joint probabilities.
            %                      For i=1:M, jpda(i) is the the joint
            %                      probability of measurement i to be
            %                      associated with the filter. jpda(M+1) is
            %                      the probability that no measurement is
            %                      associated to the filter. correctjpda
            %                      expects sum(jpda) to be equal to 1.
            %
            %   [x_corr, P_corr] = correct(filter, z, jpda, varargin) additionally
            %   allows the definition of parameters used by the
            %   MeasurementFcn in addition to obj.State. For example, the
            %   sensor's location.

            narginchk(3,inf)
            
            validateCorrectJPDAInputs(IMM, z, jpda, 'correctjpda', varargin{:});

            M=size(z,2);
            likelihoods = zeros(IMM.pNumModels,M,'like',IMM.pModelProbabilities);
            for kMdl = coder.unroll(1:IMM.pNumModels)
                filterk = IMM.pTrackingFilters{kMdl};
                
                for meas = 1:M
                    
                    % Compute model likelihood with each measurement
                    if IMM.pIsLinearKalmanFilter(kMdl)
                        likelihoods(kMdl,meas) = likelihood(filterk, z(:,meas));
                    else
                        likelihoods(kMdl,meas) = likelihood(filterk, z(:,meas), varargin);
                    end

                end
                
                % Probabilistic average of likelihoods:

                if IMM.pIsLinearKalmanFilter(kMdl)
                    correctjpda(filterk,z,jpda);
                else
                    correctjpda(filterk, z, jpda, varargin{:});
                end
            end
            
            % Update model probabilities
            mdl_probs = updateModelProbJPDA(IMM, likelihoods, jpda, M);
            
            % Output the combined state from all of the filter models
            [x_corr, P_corr] = combineModels(IMM);
            
            % Update correction data
            updateCorrectionData(IMM);

            % Update the OOSM filter.
            updateHistoryAfterCorrection(IMM);
        end
        
        
        %-----------------------------------------------------------------
        % distance method
        %-----------------------------------------------------------------
        function d = distance(IMM, zMatrix, measParams)
            % distance Computes distances between measurements and the
            %   Interacting Multiple Model object.
            %   d = distance(filter,z_matrix) computes a distance between
            %   one or more measurements supplied by the zMatrix and the
            %   measurement predicted by the Interacting Multiple Model object.
            %   This computation takes into account the covariance of the
            %   predicted state and the measurement noise. Each row of the input
            %   zMatrix must contain a measurement vector of length N.
            %
            %   d = distance(filter, zMatrix, measurementParams) allows to
            %   define additional parameters that will be used by the
            %   Measurement functions of filters under use by the IMM
            %   filter. It should be specified as a cell array,e.g., {1, [2;3]}.
            %   If unspecified, it will be assumed to be an empty cell array.
            %
            %   The distance method returns a row vector where each element
            %   is a distance associated with the corresponding measurement
            %   input.
            
            % zMatrix    - measurement vector.
            % measParams - measurementParams are the detection metadata apart from the
            % measurement values provided by an object detection.
            % d is the distance matrix.
            
            if nargin<3
                measParams = {};
            end
            
            cond = (numel(IMM) > 1);
            coder.internal.errorIf(cond, ...
                'fusion:trackingIMM:NonScalarFilter', ...
                'trackingIMM', 'distance');
            
            modelProb = IMM.pModelProbabilities;
            
            % First model
            filterk = IMM.pTrackingFilters{1};
            wk = modelProb(1);
            
            % Linear filters cannot use measurement parameters
            if IMM.pIsLinearKalmanFilter(1)
                d = wk*distance(filterk, zMatrix);
            else
                d = wk*distance(filterk, zMatrix, measParams);
            end
            
            % Remaining models
            for kMdl = coder.unroll(2:IMM.pNumModels)
                filterk = IMM.pTrackingFilters{kMdl};
                wk = modelProb(kMdl);
                
                % Linear filters cannot use measurement parameters
                if IMM.pIsLinearKalmanFilter(kMdl)
                    d = d + wk*distance(filterk, zMatrix);
                else
                    d = d + wk*distance(filterk, zMatrix, measParams);
                end
            end
        end
        
        %------------------------------------------------------------------
        % likelihood method
        %------------------------------------------------------------------
        function lhood = likelihood(IMM,z,measParams)
            %LIKELIHOOD Calculate the likelihood of a measurement
            %  l = likelihood(IMM, Z) calculates the likelihood of a
            %  measurement, Z, given the object, IMM.
            %
            %  l = likelihood(IMM, Z, measurementParams) allows you to
            %  define additional parameters that will be used by the
            %  filters. It should be specified as a cell array, e.g., {1,
            %  [2;3]}. If unspecified, it will be assumed to be an empty
            %  cell array.
            coder.internal.errorIf(numel(IMM) > 1, ...
                'fusion:trackingIMM:NonScalarFilter','trackingIMM', 'likelihood');
            
            if nargin == 2
                measParams = {};
            end
            
            % Validate z-size
            validateattributes(z, {'numeric'}, ...
                {'real', 'finite', 'nonsparse', 'vector'},...
                'likelihood', 'z')
            
            indLhood = zeros(IMM.pNumModels,1,'like',IMM.State);
            coder.unroll();
            for kMdl = 1:IMM.pNumModels
                filterk = IMM.pTrackingFilters{kMdl};
                if IMM.pIsLinearKalmanFilter(kMdl)
                    indLhood(kMdl) = likelihood(filterk,z);
                else
                    indLhood(kMdl) = likelihood(filterk,z,measParams);
                end
            end
            cbar = IMM.pModelProbabilities;
            lhood = sum(cbar(:).*indLhood);
        end
        
        %-----------------------------------------------------------------
        % clone method
        %-----------------------------------------------------------------
        function newIMM = clone(IMM)
            % clone Create a copy of the filter
            %
            % objClone = clone(IMM)
            
            coder.inline('never');
            
            % clone must be called with a single object
            coder.internal.errorIf(numel(IMM) > 1, ...
                'fusion:trackingIMM:NonScalarFilter', ...
                'trackingIMM', 'clone');
            % Use str2func to get the correct object type. When called from
            % a subclass, the resulting object will be of the same subclass
            obj = str2func(coder.const(class(IMM)));
            newFilters = cell(IMM.pNumModels,1);
            for i = coder.unroll(1:IMM.pNumModels)
                newFilters{i} = clone(IMM.pTrackingFilters{i});
            end
            
            newIMM = obj('TrackingFilters',newFilters,...
                'ModelConversionFcn',IMM.ModelConversionFcn,...
                'TransitionProbabilities',IMM.TransitionProbabilities);
            % Copy the rest of the properties
            %
            % ppProperties holds the list of all properties that may not be
            % set during construction
            ppProperties = coder.const({'pState','pStateCovariance',...
                'pMeasurementNoise','pModelProbabilities',...
                'pNumModels','pLastRetrodictionDT'});
            for kk = coder.unroll(1:numel(ppProperties))
                % Copy only if the prop was set in the source obj
                if coder.internal.is_defined(IMM.(ppProperties{kk}))
                    newIMM.(ppProperties{kk}) = IMM.(ppProperties{kk});
                end
            end
            copySmootherProperties(IMM,newIMM);
            cloneRetroFilter(newIMM, IMM);
        end
        
        %------------------------------------------------------------------
        % Retrodiction methods
        %------------------------------------------------------------------
        function [retroState, retroCov, success] = retrodict(obj,dt)
            % RETRODICT Retrodict the filter to a previous time
            % [retroState, retroCov] = RETRODICT(obj,dt) retrodicts
            % (predicts backward in time) the filter by dt seconds. dt must
            % be a nonpositive time difference from the current filter time
            % to the time at which an out-of-sequence measurement (OOSM) was
            % taken.
            %
            % retroState is the retrodicted state to OOSM time, xhat(kappa|k)
            % retroCov is the retrodicted state covariance to OOSM time, P(kappa|k)
            %
            % If the filter cannot be retrodicted to the time of the OOSM,
            % because its history does not extend that far back, the
            % returned values are the same as the current state and state
            % covariance. You can check the status of the retrodiction using
            % [..., success] = ... where success is true if the filter is
            % retrodicted.
            
            % Retrodict must be called with a single object
            coder.internal.errorIf(numel(obj) > 1, ...
                'fusion:trackingIMM:NonScalarFilter', ...
                'trackingIMM', 'retrodict');
            
            validateRetrodict(obj, dt);
            [distk,tk, tkappa, success, distl,tl] = getRetrodictQuantities(obj, dt);

            if ~success
                retroState = obj.State;
                retroCov = obj.StateCovariance;
                obj.pLastRetrodictionDT(1) = 0;
                return
            end
              
            [retroState, retroCov] = retrodictionImpl(obj,dt,tk,tkappa,distk,tl,distl);
        end
        
        function [x_retroCorr, P_retroCorr] = retroCorrect(obj,z,varargin)
            %retroCorrect Correct the filter with an out-of-sequence measurement
            % [retroCorrState,retroCorrCov] = retroCorrect(filter,z)
            % corrects the filter with the OOSM measurement z and returns
            % the corrected state and state covariance. The function
            % changes the State and StateCovariance properties of the
            % filter object to retroCorrState and retroCorrCov,
            % respectively. You must first call the retrodict method to
            % predict the filter backwards to the time of the measurement.
            %
            % ... = retroCorrect(...,measparams) specifies the measurement
            % parameters for the measurement z. The measurement parameters
            % are used by the measurement function specified in the
            % MeasurementFcn property of the filter.
            
            % retroCorrect must be called with a single object
            coder.internal.errorIf(numel(obj) > 1, ...
                'fusion:trackingIMM:NonScalarFilter', ...
                'trackingIMM', 'retroCorrect');
            
            coder.internal.assert(obj.pWasRetrodicted,...
                'shared_tracking:OOSMFilter:MustCallThisMethodBeforeThatMethod',...
                'retrodict','retroCorrect');
            
            [~,distk] = fetchDistributionByTime(obj, max(obj.pCorrectionTimestamps));
            
            retroCorrectImpl(obj,z,distk,varargin{:});

            if obj.pPredictionDeltaTFromLastCorrection > 0
                dtfp = obj.pPredictionDeltaTFromLastCorrection;
                predict(obj, dtfp);
                obj.pPredictionDeltaTFromLastCorrection = dtfp;
            end
            
            x_retroCorr = obj.pState;
            P_retroCorr = obj.pStateCovariance;
        end
        
        function [x_retroCorr, P_retroCorr] = retroCorrectJPDA(obj,z, jpda, varargin)
           %retroCorrectJPDA Correct the filter with out-of-sequence measurements
           % [x_corr,P_corr] = retroCorrectJPDA(filter, z, jpda) 
           % returns the correction of state, x_corr, and state estimation
           % error covariance, P_corr, based on the current set of
           % out-of-sequence measurements z and their joint probabilistic
           % data association coefficients jpda. The function changes the
           % State and StateCovariance properties of the filter object to
           % retroCorrState and retroCorrCov, respectively. You must first
           % call the retrodict method to predict the filter backwards to
           % the time of the measurement.
           %
           % ... = retroCorrectJPDA(...,measparams) specifies the measurement
           % parameters for the measurements z. The measurement parameters
           % are used by the measurement function specified in the
           % MeasurementFcn property of the filter.
                      
           coder.internal.assert(obj.pWasRetrodicted,...
               'shared_tracking:OOSMFilter:MustCallThisMethodBeforeThatMethod',...
               'retrodict','retroCorrectJPDA');
           
           
           % Validate correct jpda inputs
           validateCorrectJPDAInputs(obj, z, jpda, 'retroCorrectJPDA',varargin{:});
           
           [~,distk] = fetchDistributionByTime(obj, max(obj.pCorrectionTimestamps));

           retroCorrectJPDAImpl(obj,z,jpda,distk,varargin{:});

           if obj.pPredictionDeltaTFromLastCorrection > 0
               dtfp = obj.pPredictionDeltaTFromLastCorrection;
               predict(obj, dtfp);
               obj.pPredictionDeltaTFromLastCorrection = dtfp;
           end
           
           x_retroCorr = obj.State;
           P_retroCorr = obj.StateCovariance;
       end

    end

    methods(Access = ...
            {?matlabshared.tracking.internal.RetrodictionFilter, ...
            ?matlabshared.tracking.internal.AbstractContainsFilters, ...
            ?matlab.unittest.TestCase})
        function [retroState, retroCov] = retrodictionImpl(obj,dt,tk,tkappa,distk,tl,distl)
            % Retrodict each filter
            coder.unroll();
            for i = 1:obj.pNumModels
                filterDistK = distk.FilterDistributions{i};
                filterDistL = distl.FilterDistributions{i};
                retrodictionImpl(obj.pTrackingFilters{i},...
                    dt,tk,tkappa,filterDistK,tl,filterDistL);
            end

            % Compute mixing probabilities
            modelProb = distk.ModelProbabilities;
            transProb = max(0,real((obj.TransitionProbabilities)^complex(abs(tk-tkappa))));
            c0 = bsxfun(@times, modelProb(:), transProb);
            cbar = sum(c0, 1); % (11.6.6-8)
            obj.pModelProbabilities = cbar;

            % Combine the models for the final state
            [retroState, retroCov] = combineModels(obj);
            
            obj.pWasRetrodicted = true;
            obj.pLastRetrodictionDT(1) = tk-tkappa;
        end

        function retroCorrectImpl(obj,z,distk,varargin)
            likelihoods = zeros(obj.pNumModels,1,'like',obj.pModelProbabilities);
            coder.unroll();
            for kMdl = 1:obj.pNumModels
                filterK = obj.pTrackingFilters{kMdl};
                filterKDistribution = distk.FilterDistributions{kMdl};

                % Compute model likelihood (i.e. how "statistically" close
                % was the prediction to the measurement.)
                if obj.pIsLinearKalmanFilter(kMdl)
                    likelihoods(kMdl) = likelihood(filterK, z);
                    retroCorrectImpl(filterK, z, filterKDistribution);
                else
                    likelihoods(kMdl) = likelihood(filterK, z, varargin);
                    retroCorrectImpl(filterK, z, filterKDistribution, varargin{:});
                end
            end
            
            % Update model probabilities
            cbar = distk.ModelProbabilities;
            transProb = max(0,real((obj.TransitionProbabilities)^complex(abs(obj.pLastRetrodictionDT))));
            mdl_probs = (likelihoods' * transProb)' .* cbar;
            mdl_probs = mdl_probs/sum(mdl_probs);
            obj.pModelProbabilities = mdl_probs;
            
            % Output the combined state from all of the filter models
            combineModels(obj);
            
            obj.pWasRetrodicted = false;
            obj.pLastRetrodictionDT(1) = 0;
        end
        
        function [x_corr, P_corr] = retroCorrectJPDAImpl(IMM, z, jpda, distk, varargin)

            M=size(z,2);
            likelihoods = zeros(IMM.pNumModels,M,'like',IMM.pModelProbabilities);
            for kMdl = coder.unroll(1:IMM.pNumModels)
                filterk = IMM.pTrackingFilters{kMdl};
                filterKDistribution = distk.FilterDistributions{kMdl};

                for meas = 1:M
                    % Compute model likelihood with each measurement
                    if IMM.pIsLinearKalmanFilter(kMdl)
                        likelihoods(kMdl,meas) = likelihood(filterk, z(:,meas));
                    else
                        likelihoods(kMdl,meas) = likelihood(filterk, z(:,meas), varargin);
                    end
                end

                if IMM.pIsLinearKalmanFilter(kMdl)
                    retroCorrectJPDAImpl(filterk,z,jpda, filterKDistribution);
                else
                    retroCorrectJPDAImpl(filterk, z, jpda, filterKDistribution, varargin{:});
                end

            end

            % Update model probabilities
            updateModelProbJPDA(IMM, likelihoods, jpda, M);
            
            % Output the combined state from all of the filter models
            [x_corr, P_corr] = combineModels(IMM);

            IMM.pWasRetrodicted = false;
            IMM.pLastRetrodictionDT(1) = 0;
        
        end
    end
    
    methods (Access = ...
            {?matlabshared.tracking.internal.AbstractTrackingFilter, ...
            ?matlabshared.tracking.internal.AbstractContainsFilters, ...
            ?matlab.unittest.TestCase})
        %-----------------------------------------------------------------
        % sync method: Sync the filter with the filter provided as second
        % input.
        %-----------------------------------------------------------------
        function sync(IMM, IMM2)
            % sync(IMM1, IMM2) synchronizes the first trackingIMM filter,
            %   IMM1, with the second trackingIMM filter, IMM2.
            %   In order to sync IMM and IMM2 both must have equal number of
            %   underlying filters of same class.
            %   sync: Synchronizes the IMM with IMM2 to make sure that:
            %   1. State is same
            %   2. StateCovariance is same.
            %   3. Measurement Noise and Model Probabilities are same.
            %   4. Number of tracking filters are same.
            
            %   % Example: Synchronizing IMM1 and IMM2
            %   % ------------------------------------------------------
            %   detection = objectDetection(0, [1;3;0], 'MeasurementNoise', [1 0.2 0; 0.2 2 0; 0 0 1]);
            %   detection2 = objectDetection(0, [1;1;1], 'MeasurementNoise', [5 0.2 0; 0.2 4 0; 0 0 5]);
            %   filter  = {initcvekf(detection);initcaekf(detection);initctekf(detection)};
            %   filter2 = {initcvekf(detection2);initcaekf(detection2);initctekf(detection2)};
            %   IMM  = trackingIMM(filter);
            %   IMM2 = trackingIMM(filter2);
            %   sync(IMM,IMM2)
            %   cond = isequal(IMM.State,IMM2.State)
            %   cond is equal to 1 means both the trackingIMM filter
            %   objects states are synced according to IMM2.State.
            
            % Validate filter input
            classToExpect = 'trackingIMM';
            validateattributes(IMM2,{classToExpect},{'scalar'},'trackingIMM');
            % Synchronizing the underlying filters
            for i = coder.unroll(1:IMM.pNumModels)
                sync(IMM.pTrackingFilters{i},IMM2.pTrackingFilters{i});
            end
            syncRetroFilter(IMM,IMM2);
            IMM.ModelConversionFcn      = IMM2.ModelConversionFcn;
            IMM.pModelProbabilities     = IMM2.pModelProbabilities;
            IMM.TransitionProbabilities = IMM2.TransitionProbabilities;
            IMM.MeasurementNoise        = IMM2.MeasurementNoise;
            combineModels(IMM);
        end
        
        %-----------------------------------------------------------------
        % nullify method
        %-----------------------------------------------------------------
        function nullify(IMM)
            % nullify(IMM) sets the State and StateCovariance to zeros
            
            % nullify must be called with a single object
            coder.internal.errorIf(numel(IMM) > 1, ...
                'fusion:trackingIMM:NonScalarFilter', ...
                'trackingIMM', 'nullify');
            for kMdl = coder.unroll(1:IMM.pNumModels)
                filt = IMM.pTrackingFilters{kMdl};
                nullify(filt);
            end
            createHistory(IMM);
            combineModels(IMM);
        end
        
        %-----------------------------------------------------------------
        % modelName method
        %-----------------------------------------------------------------
        function names = modelName(IMM)
            %modelName Return the names of the motion models
            %   names = modelName(IMM) returns a list of motion model
            %   names.
            
            modelNames = cell(IMM.pNumModels,1);
            for i = coder.unroll(1:IMM.pNumModels)
                modelNames{i} = IMM.pTrackingFilters{i}.modelName;
            end
            names = modelNames;
        end
        
        %-----------------------------------------------------------------
        % models method
        %-----------------------------------------------------------------
        function [stm,mm] = models(IMM, dt)
            %MODELS Return the state transition and measurement model
            %   [stm, mm] = MODELS(IMM) returns the state transition and
            %   measurement model
            
            [stm,mm] = models(IMM.pTrackingFilters{1}, dt);
        end
    end
    
    methods(Access=protected)
        function filter = defaultTrackingFilters(~)
             filter = {initcvekf(objectDetection(0, [0;0;0]));...
                initcaekf(objectDetection(0, [0;0;0]));initctekf(objectDetection(0, [0;0;0]))};
        end
        
        function value = defaultModelConversionFcn(~)
            value = @switchimm;
        end

        function validateCorrectJPDAInputs(filter, z, jpda, fcname, varargin)
            % correct must be called with a single object
            coder.internal.errorIf(numel(filter) > 1, ...
                'fusion:trackingIMM:NonScalarFilter', ...
                'trackingIMM', fcname);

            % validate some basic attributes of the measurement matrix
            validateattributes(z,{'double', 'single'}, ...
                {'real', 'finite', 'nonsparse','2d'},...
                fcname, 'z');

            % jpda should have length of number of measurements + 1
            ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();
            numMeas = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntIndex(size(z,2));
            expectedNumel = numMeas + ONE;
            validateattributes(jpda ,{'double','single'}, ...
                {'real', 'finite', 'nonsparse','nonnegative', 'vector', 'numel',...
                expectedNumel ,'<=',1},fcname, 'jpda');

            % jpda coefficients must sum to 1
            classToUse = class(filter.pTrackingFilters{1}.State);
            coder.internal.errorIf(abs(sum(jpda)-1)>sqrt(eps(classToUse)),...
                'shared_tracking:ExtendedKalmanFilter:InvalidJPDAInput'...
                ,'jpda')
        end
    
        function mdl_probs = updateModelProbJPDA(IMM, likelihoods, jpda, M)
            cbar = IMM.pModelProbabilities;
            jpda_col = jpda(:);
            beta = jpda_col(1:M);
            beta_not = jpda_col(M+1);
            jpdaLikelihoods = bsxfun(@plus,likelihoods*beta,beta_not);
            
            mdl_probs = cbar(:).*jpdaLikelihoods;
            mdl_probs = mdl_probs/sum(mdl_probs);
            IMM.pModelProbabilities = mdl_probs;
        end
    end
    
    methods(Access = {?matlabshared.tracking.internal.OOSMFilter, ...
            ?matlab.unittest.TestCase})
        function createHistory(obj)
            if obj.MaxNumOOSMSteps > 0
                dist = getDistribution(obj);
                classToUse = class(dist.FilterDistributions{1}.State);
                obj.pPredictionDeltaTFromLastCorrection = zeros(1,1,classToUse);
                obj.pCorrectionTimestamps = zeros(1,obj.MaxNumOOSMSteps,classToUse);
                obj.pCorrectionDistributions = repmat(dist,1,obj.MaxNumOOSMSteps);
                obj.pLastRetrodictionDT = zeros(1,1,classToUse);
            end
        end
    end
    
    methods(Access=private)
        %-----------------------------------------------------------------
        % mixModels method
        %-----------------------------------------------------------------
        function mixModels(IMM, dt)
            % When mixing models, the missing components of lower
            % dimension states (which are present in higher dimension
            % states) are augmented by the extra components from the higher
            % dimension state. This yields unbiased mixing in the IMM.
            
            % Compute mixing probabilities
            modelProb = IMM.pModelProbabilities;
            transProb = max(0,real((IMM.TransitionProbabilities)^complex(dt)));
            c0 = bsxfun(@times, modelProb(:), transProb);
            cbar = sum(c0, 1); % (11.6.6-8)
            mixProb = bsxfun(@rdivide, c0, cbar); % (11.6.6-7)
            
            % Update model probabilities (These are predicted model
            % probabilities)
            IMM.pModelProbabilities(:) = cbar;
            
            % Handle case when probability for a model becomes 0
            iFnd = find(modelProb == 0);
            for p = 1:numel(iFnd)
                mixProb(:,iFnd(p)) = 0;
                mixProb(iFnd(p),iFnd(p)) = 1;
            end
            
            IMMDistribution = mixDistribution(IMM,mixProb);
            % IMMDIstribution is a struct with Filter Distributions
            % (Storing State and State Covariances)
            
            for jMdl = coder.unroll(1:IMM.pNumModels)
                filterj = IMM.pTrackingFilters{jMdl};
                if(isa(filterj,'trackingPF'))
                    initialize(filterj, filterj.NumParticles,...
                        IMMDistribution.FilterDistributions{jMdl}.State,...
                        IMMDistribution.FilterDistributions{jMdl}.StateCovariance);
                else
                    filterj.State = IMMDistribution.FilterDistributions{jMdl}.State;
                    filterj.StateCovariance = IMMDistribution.FilterDistributions{jMdl}.StateCovariance;
                end
            end
        end
        
        %-----------------------------------------------------------------
        %  combineModels method
        %-----------------------------------------------------------------
        function [Xout, Pout] = combineModels(IMM)
            % Combine models for output. The first model's state definition
            % will be used for the combined output. When combining models
            % for output, the missing components of lower dimension states
            % (which present in the higher dimension states) are assumed to
            % be zero.
            
            coder.internal.errorIf(numel(IMM) > 1, ...
                'fusion:trackingIMM:NonScalarFilter', ...
                'trackingIMM', 'clone');
            
            [Xout, Pout] = combineDist(IMM);
            
            % Note: Xout has already been transposed for Kalman filter case,
            % see combineDist method for more details
            IMM.pState = Xout;
            IMM.pStateCovariance = Pout;
        end
        
        function [state,stateCov,trackingFilters,...
                transitionProbabilities,modelConversion,...
                modelProb,measNoise] = parseInputs(IMM, varargin)
            
            % Find first name-value index for the following possible
            % constructions:
            %   IMM = trackingIMM
            %   IMM = trackingIMM(TrackingFilters)
            %   IMM = trackingIMM(TrackingFilters,ModelConversionFcn)
            %   IMM = trackingIMM(TrackingFilters,ModelConversionFcn,TransitionProbabilities)
            %   IMM = trackingIMM(Name, Value,...)
            firstNVIndex = matlabshared.tracking.internal.findFirstNVPair(varargin{:});
            
            coder.internal.errorIf(firstNVIndex>4, ...
                'fusion:trackingIMM:invalidInputsToConstructor', firstNVIndex-1);
            
            switch firstNVIndex
                case 1
                    parsingParams = struct(...
                        'State',[],...
                        'StateCovariance',[],...
                        'TrackingFilters', [], ...
                        'ModelConversionFcn', IMM.defaultModelConversionFcn,...
                        'TransitionProbabilities', [], ...
                        'ModelProbabilities', [],...
                        'MeasurementNoise', []);
                case 2
                    parsingParams = struct(...
                        'State',[],...
                        'StateCovariance',[],...
                        'TrackingFilters', {varargin{1}}, ...
                        'ModelConversionFcn', IMM.defaultModelConversionFcn,...
                        'TransitionProbabilities', [], ...
                        'ModelProbabilities', [],...
                        'MeasurementNoise', []);
                case 3
                    parsingParams = struct(...
                        'State',[],...
                        'StateCovariance',[],...
                        'TrackingFilters', {varargin{1}}, ...
                        'ModelConversionFcn', varargin{2},...
                        'TransitionProbabilities',[], ...
                        'ModelProbabilities', [],...
                        'MeasurementNoise', []);
                case 4
                    parsingParams = struct(...
                        'State',[],...
                        'StateCovariance',[],...
                        'TrackingFilters', {varargin{1}}, ...
                        'ModelConversionFcn', varargin{2},...
                        'TransitionProbabilities', varargin{3}, ...
                        'ModelProbabilities', [],...
                        'MeasurementNoise', []);
            end
            
            if coder.target('MATLAB')
                [state,stateCov,trackingFiltersIn,transitionProbabilitiesIn,modelConversion,modelProb,measNoise]...
                    = parseInputsSimulation(parsingParams,varargin{firstNVIndex:end});
            else % Code generation
                [state,stateCov,trackingFiltersIn,transitionProbabilitiesIn,modelConversion,modelProb,measNoise]...
                    = parseInputsCodegen(parsingParams,varargin{firstNVIndex:end});
            end
            
            if isempty(trackingFiltersIn)
                trackingFilters = IMM.defaultTrackingFilters;
            else
                trackingFilters = trackingFiltersIn;
            end
            
            if isempty(transitionProbabilitiesIn)
                transitionProbabilities = cast(IMM.defaultTransitionProbabilities,'like',trackingFilters{1}.State);
            else
                transitionProbabilities = transitionProbabilitiesIn;
            end
        end
    end
    
    %---------------------------------------------------------------------
    % Overrides for Custom Display
    %---------------------------------------------------------------------
    methods (Access='protected')
        function propGroups = getPropertyGroups(obj)
            
            propGroups = [ matlab.mixin.util.PropertyGroup( {'State', 'StateCovariance'},''); ...
                matlab.mixin.util.PropertyGroup( {'TrackingFilters', 'HasMeasurementWrapping', 'MeasurementNoise'},''); ...
                matlab.mixin.util.PropertyGroup( {'ModelConversionFcn', 'TransitionProbabilities', 'ModelProbabilities'},'')];
            oosmGroup = getPropertyGroups@matlabshared.tracking.internal.RetrodictionFilter(obj);
            smoothGroups = getPropertyGroups@fusion.internal.IMMSmoother(obj);
            propGroups = [propGroups;oosmGroup;smoothGroups];
        end
    end
    
    methods (Access = protected)
        %-----------------------------------------------------------------
        % saveobj to make sure that the filter is saved correctly.
        %-----------------------------------------------------------------
        function [sobj] = saveobj(IMM)
            sIMM = struct(...
                'State',                   IMM.State,...
                'StateCovariance',         IMM.StateCovariance,...
                'ModelConversionFcn',      IMM.ModelConversionFcn,...
                'TransitionProbabilities', IMM.TransitionProbabilities,...
                'MeasurementNoise',        IMM.MeasurementNoise,...
                'ModelProbabilities',      IMM.ModelProbabilities,...
                'pState',                  IMM.pState,...
                'pStateCovariance',        IMM.pStateCovariance,...
                'pTransitionProbabilities',IMM.pTransitionProbabilities,...
                'pMeasurementNoise',       IMM.pMeasurementNoise,...
                'pModelProbabilities',     IMM.pModelProbabilities,...
                'pNumModels',              IMM.pNumModels,...
                'TrackingFilters',         {IMM.pTrackingFilters},...
                'pLastRetrodictionDT',     IMM.pLastRetrodictionDT);
            sSmooth = saveSmootherProperties(IMM,sIMM);
            sobj = saveRetroProperties(IMM,sSmooth);
        end
        
        function loadPrivateProtectedProperties(IMM,sobj)
            % Helper for loadobj. This allows subclasses to avoid
            % copy/pasting the whole loadobj for private/protected properties
            
            % Subclasses must:
            % * Construct object of the correct type
            
            % Protected/private properties
            IMM.pState                     = sobj.pState;
            IMM.pStateCovariance           = sobj.pStateCovariance;
            IMM.ModelConversionFcn         = sobj.ModelConversionFcn;
            IMM.pTransitionProbabilities   = sobj.pTransitionProbabilities;
            IMM.pMeasurementNoise          = sobj.pMeasurementNoise;
            IMM.pModelProbabilities        = sobj.pModelProbabilities;
            IMM.pNumModels                 = sobj.pNumModels;

            if isfield(sobj,'pLastRetrodictionDT')
                IMM.pLastRetrodictionDT = sobj.pLastRetrodictionDT;
            else
                IMM.pLastRetrodictionDT = zeros(1,1,'like',sobj.pState);
            end
        end
        
        function dist = getDistribution(obj)
            dist = distribution(obj);
        end

        function setMaxNumOOSMSteps(IMM,value)
            if value>0
                isRetrodictionFilter = zeros(1,numel(IMM.pTrackingFilters),'logical');
                coder.unroll();
                for i = 1:numel(IMM.pTrackingFilters)
                    isRetrodictionFilter(i) = isa(IMM.pTrackingFilters{i},...
                        'matlabshared.tracking.internal.RetrodictionFilter');
                end
                coder.internal.assert(all(isRetrodictionFilter),'fusion:trackingIMM:expectedRetrodictionFilters');
            end
            setMaxNumOOSMSteps@matlabshared.tracking.internal.RetrodictionFilter(IMM,value);
        end
    end
    
    methods (Static = true)
        function retIMM = loadobj(sobj)
            if isscalar(sobj)
                retIMM = trackingIMM(sobj.TrackingFilters, ...
                    sobj.ModelConversionFcn,sobj.TransitionProbabilities);
                loadPrivateProtectedProperties(retIMM,sobj);
            else
                % Assign non-dependent public properties
                retIMM = trackingIMM({sobj.TrackingFilters}, ...
                    sobj(1).ModelConversionFcn,sobj(1).TransitionProbabilities);
                % Load protected/private properties
                loadPrivateProtectedProperties(retIMM,sobj(1));
            end
            loadSmootherProperties(retIMM,sobj);
            loadRetroProperties(retIMM,sobj);
        end
    end
    
    methods(Hidden)
        function setMeasurementSizes(obj, measurementSize, measurementNoiseSize)
            % Sets the sizes of the measurement (pN) and measurement noise
            % (pV). Both have to be a real, positive, integer, scalar. This
            % only works on nonlinear filters.
            validateattributes(measurementSize, {'numeric'}, {'real', ...
                'positive', 'integer', 'scalar'}, 'trackingIMM');
            validateattributes(measurementNoiseSize, {'numeric'}, {'real', ...
                'positive', 'integer', 'scalar'}, 'trackingIMM');
            for i = coder.unroll(1:obj.pNumModels)
                if ~matlabshared.tracking.internal.isLinearKalmanFilter(obj.pTrackingFilters{i})
                    setMeasurementSizes(obj.pTrackingFilters{i}, measurementSize, measurementNoiseSize)
                end
            end
        end
    end
    
    methods (Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
            smootherProps = fusion.internal.IMMSmoother.matlabCodegenNontunableProperties;
            props = {smootherProps{:},'pIsLinearKalmanFilter'};
        end
    end
end
%-------------------------------------------------------------------------
% Parse inputs for simulation
%-------------------------------------------------------------------------
function [state,stateCov,trackingFilters,transitionProbabilities,...
    modelConversion,modelProb,measNoise] = parseInputsSimulation(defaultParams,varargin)
parser = inputParser;

parser.addParameter('State',                  defaultParams.State);
parser.addParameter('StateCovariance',        defaultParams.StateCovariance);
parser.addParameter('TrackingFilters',        defaultParams.TrackingFilters);
parser.addParameter('ModelConversionFcn',     defaultParams.ModelConversionFcn);
parser.addParameter('TransitionProbabilities',defaultParams.TransitionProbabilities);
parser.addParameter('ModelProbabilities',     defaultParams.ModelProbabilities);
parser.addParameter('MeasurementNoise',       defaultParams.MeasurementNoise);

% Parse and check optional parameters
parser.parse(varargin{:});
args = parser.Results;

trackingFilters         = args.TrackingFilters;
transitionProbabilities = args.TransitionProbabilities;
modelConversion         = args.ModelConversionFcn;
modelProb               = args.ModelProbabilities;
measNoise               = args.MeasurementNoise;
state                   = args.State;
stateCov                = args.StateCovariance;
end

%-------------------------------------------------------------------------
% Parse inputs for code generation
%-------------------------------------------------------------------------
function [state,stateCov,trackingFilters,transitionProbabilities,...
    modelConversion,modelProb,measNoise] = parseInputsCodegen(defaultParams,varargin)
parms = struct( ...
    'State',                    uint32(0), ...
    'StateCovariance',          uint32(0), ...
    'TrackingFilters',          uint32(0), ...
    'ModelConversionFcn',       uint32(0), ...
    'TransitionProbabilities',  uint32(0), ...
    'ModelProbabilities',       uint32(0), ...
    'MeasurementNoise',         uint32(0));

popt = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand', true, ...
    'PartialMatching', false);

optarg = eml_parse_parameter_inputs(parms, popt, varargin{:});
trackingFilters = eml_get_parameter_value(optarg.TrackingFilters,...
    defaultParams.TrackingFilters, varargin{:});
transitionProbabilities = eml_get_parameter_value(optarg.TransitionProbabilities,...
    defaultParams.TransitionProbabilities, varargin{:});
modelConversion = eml_get_parameter_value(optarg.ModelConversionFcn,...
    defaultParams.ModelConversionFcn, varargin{:});
modelProb = eml_get_parameter_value(optarg.ModelProbabilities,...
    defaultParams.ModelProbabilities, varargin{:});
measNoise = eml_get_parameter_value(optarg.MeasurementNoise,...
    defaultParams.MeasurementNoise, varargin{:});
state = eml_get_parameter_value(optarg.State,...
    defaultParams.State, varargin{:});
stateCov = eml_get_parameter_value(optarg.StateCovariance,...
    defaultParams.StateCovariance, varargin{:});
end