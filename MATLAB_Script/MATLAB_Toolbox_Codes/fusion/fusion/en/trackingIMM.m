classdef trackingIMM< matlabshared.tracking.internal.AbstractTrackingFilter & matlabshared.tracking.internal.fusion.CustomDisplay & matlabshared.tracking.internal.AbstractJPDAFilter & fusion.internal.IMMSmoother & matlabshared.tracking.internal.RetrodictionFilter
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

 
%   Copyright 2018-2021 The MathWorks, Inc.

    methods
        function out=trackingIMM
            %-------------------------------------------------------------
            % Constructor
            %-------------------------------------------------------------
        end

        function out=clone(~) %#ok<STOUT>
            % clone Create a copy of the filter
            %
            % objClone = clone(IMM)
        end

        function out=correct(~) %#ok<STOUT>
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
        end

        function out=correctjpda(~) %#ok<STOUT>
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
        end

        function out=defaultModelConversionFcn(~) %#ok<STOUT>
        end

        function out=defaultTrackingFilters(~) %#ok<STOUT>
        end

        function out=distance(~) %#ok<STOUT>
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
        end

        function out=getDistribution(~) %#ok<STOUT>
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

        function out=initialize(~) %#ok<STOUT>
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
        end

        function out=likelihood(~) %#ok<STOUT>
            %LIKELIHOOD Calculate the likelihood of a measurement
            %  l = likelihood(IMM, Z) calculates the likelihood of a
            %  measurement, Z, given the object, IMM.
            %
            %  l = likelihood(IMM, Z, measurementParams) allows you to
            %  define additional parameters that will be used by the
            %  filters. It should be specified as a cell array, e.g., {1,
            %  [2;3]}. If unspecified, it will be assumed to be an empty
            %  cell array.
        end

        function out=loadPrivateProtectedProperties(~) %#ok<STOUT>
            % Helper for loadobj. This allows subclasses to avoid
            % copy/pasting the whole loadobj for private/protected properties
        end

        function out=loadobj(~) %#ok<STOUT>
        end

        function out=predict(~) %#ok<STOUT>
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
        end

        function out=retroCorrect(~) %#ok<STOUT>
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
        end

        function out=retroCorrectJPDA(~) %#ok<STOUT>
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
        end

        function out=retrodict(~) %#ok<STOUT>
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
        end

        function out=saveobj(~) %#ok<STOUT>
        end

        function out=setMaxNumOOSMSteps(~) %#ok<STOUT>
        end

        function out=updateModelProbJPDA(~) %#ok<STOUT>
        end

        function out=validateCorrectJPDAInputs(~) %#ok<STOUT>
            % correct must be called with a single object
        end

    end
    properties
        %HasMeasurementWrapping Flags to show if TrackingFilters use wrapping
        % A 1-by-N logical array. The i-th element of the array is true if
        % the i-th filter in TrackingFilters supports wrapping measurement.
        HasMeasurementWrapping;

        %MeasurementNoise Measurement noise covariance (R)
        %   Specify the covariance of measurement noise as a scalar or an
        %   N-by-N matrix, where N is the number of measurements. If you
        %   specify it as a scalar it will be extended to an N-by-N diagonal
        %   matrix.
        %
        %   Default: 1
        MeasurementNoise;

        %ModelConversionFcn convert State or StateCovariance
        %   Specify a function handle to the Model conversion
        %   function. Which converts from (i-th) filter to (j-th) filter.
        %
        %   Default: @switchimm
        ModelConversionFcn;

        %ModelProbabilities Probability of each tracking filter
        %   Specify a L-by-1 real vector of model Probabilities. Initially,
        %   all models are equiprobable. This property is read only.
        %
        %   Default: 1/L
        ModelProbabilities;

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

        %TrackingFilters
        %   Specify an L-element cell array of filter objects. This
        %   property is read only and can only be set on construction.
        %
        %   Default: Cell array of three trackingEKF filter objects with
        %   the following motion models: constant velocity, constant
        %   acceleration and constant turn respectively.
        TrackingFilters;

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

        pIsLinearKalmanFilter;

        pLastRetrodictionDT;

        pMeasurementNoise;

        pModelProbabilities;

        %pNumModels - Number of motion model in use
        pNumModels;

        pState;

        pStateCovariance;

        pTrackingFilters;

        pTransitionProbabilities;

    end
end
