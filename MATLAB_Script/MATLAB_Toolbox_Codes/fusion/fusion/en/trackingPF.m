classdef trackingPF< matlabshared.tracking.internal.ParticleFilter & matlabshared.tracking.internal.AbstractTrackingFilter & matlabshared.tracking.internal.AbstractJPDAFilter
%trackingPF   A particle filter for object tracking
%   The particle filter is designed for tracking objects that follow a
%   nonlinear motion model or that are measured by a nonlinear measurement
%   model. The filter uses a set of discrete particles to approximate the
%   posterior distribution of the state. The particle filter can be applied
%   to arbitrary non-linear system models and process and measurement noise
%   can follow arbitrary non-Gaussian distribution.
%
%   The particles are generated using the various Resampling Methods.
%   The different Resampling Methods currently available are Multinomial,
%   Stratified, Residual and Systematic
%
%   PF = trackingPF returns an trackingPF object with state transition
%   function (@constvel), measurement function (@cvmeas) and distribution
%   of particles around state ([0;0;0;0]) with unit covariance in each
%   dimension. It assumes an additive Gaussian process noise model and
%   Gaussian likelihood calculation.
% 
%   PF = trackingPF(StateTransitionFcn, MeasurementFcn, State)
%   lets you specify the state transition function, f, and the measurement
%   function, h. Both must be specified as function_handles. In addition,
%   the object lets you specify an estimated value for the state. It
%   assumes a unit covariance around the state.
%
% 
%   PF = trackingPF(..., Name, Value) specifies additional name-value pair
%   arguments to define the properties described below.
%
%   trackingPF properties:
%   State                       - (Read-only) Current state estimate
%   StateCovariance             - (Read-only) Current state estimation error covariance
%   IsStateVariableCircular     - (Read-only) Indicator if state variables have a circular distribution
%   StateTransitionFcn          - Propagates the particles to next time step
%   ProcessNoiseSamplingFcn     - Provides a noise sample from ProcessNoise
%   ProcessNoise                - Process noise covariance
%   HasAdditiveProcessNoise     - True if process noise is additive
%   MeasurementFcn              - Projection of particles into measurement space
%   MeasurementLikelihoodFcn    - Likelihood of particles given current measurement
%   MeasurementNoise            - Measurement noise covariance if Gaussian
%   Particles                   - State of each particle
%   Weights                     - Weight/importance of each particle
%   NumStateVariables           - (Read-only) Number of state variables for the particle filter
%   NumParticles                - (Read-only) Number of particles used in the filter
%   ResamplingPolicy            - Policy settings that determine when to trigger resampling
%   ResamplingMethod            - Method used for particle resampling
%   StateEstimationMethod       - Method used for state estimation
%
%   trackingPF methods: 
%   predict    - Predict the state and state estimation error covariance
%   correct    - Correct the state and state estimation error covariance
%   correctjpda - Correct using joint probabilistic detection assignment
%   distance   - Calculate the distance between measurements and the filter
%   likelihood - Calculate the likelihood of a measurement
%   clone      - Create a copy of the object with the same property values
%   initialize - Initialize filter properties
%
%   Notes:
%   ------
%   * If the measurement exists, e.g., the object has been detected, you
%     can call the predict method and the correct method together. If the
%     measurement is missing, you can call the predict method but not the
%     correct method.
%   * You can use the distance method to compute distances that describe
%     how a set of measurements matches the particle filter. You can thus
%     find a measurement that best fits the filter. This strategy can be
%     used for matching object detections against object tracks in a
%     multi-object tracking problem.
%
%   % Example 1: Create a trackingPF for a 2D constant velocity model
%   % ---------------------------------------------------------------
%   %   Use the pre-packaged constant velocity model, constvel, and the
%   %   associated measurement model, cvmeas. These models assume that the 
%   %   state is [x;vx;y;vy] and a position measurement is [x;y;z].
%   state = [0;0;0;0]
%   PF = trackingPF(@constvel,@cvmeas,state);
%
%   % Example 2: Create a trackingPF with known number of particles and
%   % additional name-value pairs
%   % -----------------------------------------------------------------
%   %   Use the additional name-value pairs to defined NumParticles,
%   %   HasAdditiveProcessNoise and StateOrientation
%   state = [0;0;0;0];
%   stateCov = 10*eye(4);
%   PF = trackingPF(@constvel,@cvmeas,state,'StateCovariance',stateCov,...
%       'NumParticles',2500,'HasAdditiveProcessNoise',false);
%
%   % Example 3: Create a trackingPF with multi-modal state description
%   % ----------------------------------------------------------------
%   %   One of the abilities of a particle filter is to carry a multi-modal
%   %   state hypothesis. This can be achieved by modifying the Particles
%   %   property of the trackingPF after initialization.
%   state1 = [0;0;0;0];
%   stateCov1 = 10*eye(4);
%   state2 = [100;0;100;0];
%   stateCov2 = 5*eye(4);
%   % Initialize with first state to create correct sizes of particles
%   PF = trackingPF(@constvel,@cvmeas,state1,'NumParticles',2000);
%   % Modify the Particles Property
%   PF.Particles(:,1:1000) = (state1 + chol(stateCov1)*randn(4,1000));
%   PF.Particles(:,1001:2000) = (state2 + chol(stateCov2)*randn(4,1000));
%
%   % Example 4: Running the trackingPF
%   % ----------------------------------------------------------------
%   %   After constructing the filter, use the following steps to call
%   %   predict and correct. You may call predict and correct in any order
%   %   and as many times as you would like to call each.
%   state = [0;0;0;0];
%   PF = trackingPF(@constvel,@cvmeas,state);
%   meas = [1;1;0];
%   [x_Pred,P_Pred] = predict(PF,0.5); % Predict over 0.5 seconds.
%   [x_corr,P_corr] = correct(PF,meas); % Correct using the measurement
%   % You can call predict and correct without expected outputs to avoid
%   % getStateEstimate calls.
%   predict(PF); %Predict over 1 second (default constvel time)
%
%  See also: trackingKF, trackingEKF, trackingUKF, trackingCKF, constvel, cvmeas

 
%   Copyright 2018 The MathWorks, Inc.

    methods
        function out=trackingPF
        end

        function out=clone(~) %#ok<STOUT>
        end

        function out=correct(~) %#ok<STOUT>
            %CORRECT Adjust state estimate based on sensor measurement
            %   [stateCorr, stateCov] = CORRECT(PF, MEASUREMENT) calculates
            %   the corrected system state stateCorr and its associated
            %   uncertainty covariance stateCov based on a sensor
            %   MEASUREMENT at the current time step.
            %   CORRECT uses the measurement likelihood model specified in
            %   MeasurementLikelihoodFcn to calculate the likelihood for
            %   the sensor measurement for each particle. It then extracts
            %   the best state estimate and covariance based on the
            %   setting in StateEstimationMethod.
            %
            %   [stateCorr, stateCov] = CORRECT(PF, MEASUREMENT, VARARGIN)
            %   passes all additional arguments supplied in VARARGIN to the
            %   underlying MeasurementLikelihoodFcn. The first two inputs to
            %   MeasurementLikelihoodFcn are the set of particles from the
            %   current time step and the MEASUREMENT, followed by all arguments
            %   in VARARGIN
            %
            %
            %   Example:
            %
            %       % Create a particle filter with 5000 particles
            %       pf = trackingPF(@constvel,@cvmeas,[4 1 9 5],'NumParticles',5000);
            %
            %       % Run one prediction step
            %       predict(pf)
            %
            %       % Assume we have a measurement [-1;0;1]. Run the correction step.
            %       [stateCorrected, stateCov] = CORRECT(pf, [-1;0;1])
        end

        function out=correctjpda(~) %#ok<STOUT>
            %CORRECTJPDA Adjust state estimate based on several sensor
            %   measurements.
            %   [stateCorr, stateCov] = CORRECT(PF, MEASUREMENT, BETA) calculates
            %   the corrected system state stateCorr and its associated
            %   uncertainty covariance stateCov based on a scan of sensor
            %   MEASUREMENT at the current time step and their associated
            %   Joint Probabilistic Data Association coefficients BETA.
            %   CORRECTJPDA uses the measurement likelihood model specified in
            %   MeasurementLikelihoodFcn to calculate the likelihood for
            %   each sensor measurement and each particle. It then extracts
            %   the best state estimate and covariance based on the
            %   setting in StateEstimationMethod.
            %
            %   [stateCorr, stateCov] = CORRECT(PF, MEASUREMENT, VARARGIN)
            %   passes all additional arguments supplied in VARARGIN to the
            %   underlying MeasurementLikelihoodFcn. The first two inputs to
            %   MeasurementLikelihoodFcn are the set of particles from the
            %   current time step and the MEASUREMENT, followed by all arguments
            %   in VARARGIN
            %
            %
            %   Example:
            %
            %       % Create a particle filter with 5000 particles
            %       pf = trackingPF(@constvel,@cvmeas,[4 1 9 5],'NumParticles',5000);
            %
            %       % Run one prediction step
            %       predict(pf)
            %
            %       % Assume we have two measurements [-1;0;1] and [-0.5;0;1] with equal probability, i.e. beta = [0.5 0.5 0]. Run the jpda correction step.
            %       [stateCorrected, stateCov] = CORRECTJPDA(pf, [-1 -0.5;0 0;1 1],[0.5 0.5 0])
        end

        function out=displayScalarObject(~) %#ok<STOUT>
        end

        function out=distance(~) %#ok<STOUT>
            % distance Computes distances between measurements and the
            % particle filter object.
            %  d = distance(PF,z_matrix) computes a distance between one or
            %  more measurements supplied by the z_matrix and the
            %  measurement predicted by the particle filter object. To
            %  compute distance, the particles are first projected to
            %  measurement space using the MeasurementFcn and then their
            %  likelihood is calculated. The likelihood of each particle is 
            %  combined using weighted mean to compute the joint likelihood.
            %  Using the joint likelihood of the particles, a negative 
            %  log-likelihood ratio is computed as the distance value.
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

        function out=initialize(~) %#ok<STOUT>
            % initialize the particle filter 
            % initialize(PF, numParticles, state, stateCovariance);
            % initialize(PF, ____, Name, Value)
            %
            % Supported Name, Value pairs:
            % Name - 'CircularVariables' - Use this Name, value pair to set
            % the 'IsStateVariableCircular' property.
            % 
            % Name - 'StateOrientation' - Use this Name, value pair to set
            % the orientation of the state. The default functions and
            % motion/measurement models only support column orientation.
        end

        function out=invokeMeasurementFcn(~) %#ok<STOUT>
        end

        function out=invokeMeasurementLikelihoodFcn(~) %#ok<STOUT>
            % Invoke the MeasurementLikelihoodFcn with predMeasurement
        end

        function out=invokeStateTransitionFcn(~) %#ok<STOUT>
        end

        function out=likelihood(~) %#ok<STOUT>
            % likelihood Computes the joint likelihood of a measurement
            % lhood = likelihood(PF,z) computes the joint likelihood of a
            % measurement assuming empty measurement parameters.
            %
            % lhood = likelihood(PF, z, measurementParameters) allows
            % passing additional parameters that will be used by the
            % PF.MeasurementFcn. measurementParameters should be specified
            % as a cell array, e.g., {1,[2;3]}. If unspecified, it will be
            % assumed an empty cell array.
        end

        function out=predict(~) %#ok<STOUT>
            %PREDICT Calculate the predicted state in the next time step
            %   [statePred, stateCov] = PREDICT(PF) calculates the
            %   predicted system state STATEPRED and its associated
            %   uncertainty covariance STATECOV.
            %   PREDICT uses the system model specified in
            %   StateTransitionFcn to evolve the state of all particles and
            %   then extract the best state estimate and covariance based on the
            %   setting in StateEstimationMethod.
            %
            %   [statePred, stateCov] = PREDICT(PF, VARARGIN) passes
            %   all additional arguments supplied in VARARGIN to the
            %   underlying StateTransitionFcn. The first input to
            %   StateTransitionFcn is the set of particles from the
            %   previous time step, followed by all arguments in VARARGIN.
            %
            %
            %   Example:
            %
            %       % Create a particle filter with 5000 particles
            %       pf = trackingPF(@constvel,@cvmeas,[4 1 9 5],'NumParticles',5000);
            %
            %       % Run one prediction step
            %       [statePredicted, stateCov] = PREDICT(pf)
        end

        function out=validateMeasurementRelatedProperties(~) %#ok<STOUT>
        end

        function out=validateSampleSize(~) %#ok<STOUT>
        end

    end
    properties
        %HasAdditiveProcessNoise A Boolean flag that defines whether the
        %noise affecting the state transition is additive (true) or
        %non-additive (false).
        HasAdditiveProcessNoise;

        %MeasurementFcn A function to calculate the measurements given
        %current particles state.
        % zParticles = MeasurementFcn(particles,varargin)
        MeasurementFcn;

        %MeasurementNoise Measurement noise covariance
        % If MeasurementLikelihoodFcn is specified as gaussianLikelihood,
        % this property is used to specify the Gaussian noise covariance of
        % the measurement.
        MeasurementNoise;

        %ProcessNoise Process noise covariance.
        %   If ProcessNoiseSamplingFcn is specified as gaussianSampler, this
        %   property defines the Gaussian noise covariance of the process. 
        %   If HasAdditiveProcessNoise is true: specify the covariance of
        %   process noise as a scalar or an M-by-M matrix. If you specify
        %   it as a scalar it will be extended to an M-by-M diagonal
        %   matrix, with scalar value along the diagonal.
        %
        %   If HasAdditiveProcessNoise is false: specify the covariance of
        %   process noise as a W-by-W matrix, where W is the number of the
        %   process noise terms. In this case, ProcessNoise must be
        %   specified before the first call to the predict method. After
        %   the first assignment, you can specify it also as a scalar which
        %   will be extended to a W-by-W matrix, with scalar value along
        %   the diagonal.
        %
        %   Default: 1
        ProcessNoise;

        % ProcessNoiseSamplingFcn A function to generate a noise sample for
        % each particle
        % noiseSample = ProcessNoiseSamplingFcn(pf);
        % When HasAdditiveProcessNoise is false, this function outputs a
        % noise sample, which is a W-by-N matrix, where W is the number of
        % process noise terms and N is the number of particles.
        % When HasAdditiveProcessNoise is true, this function outputs a
        % noise sample, which is a M-by-N matrix, where M is the number of
        % state variables and N is the number of particles.
        % Set this property to a function_handle to generate a sample from
        % a non-Gaussian distribution. 
        ProcessNoiseSamplingFcn;

        pDataType;

        pIsCostOfAssignmentAvailable;

        pIsValidMeasurementFcn;

        %pM -  Length of state
        pM;

        pMeasurementNoise;

        pMeasurementNoiseScalar;

        %pN -  Length of measurement
        pN;

        pProcessNoise;

        pProcessNoiseScalar;

        %pV -  Length of measurement noise
        pV;

        %pW -  Length of process noise
        pW;

    end
end
