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

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>
%#ok<*MCSUP>

classdef trackingPF < matlabshared.tracking.internal.ParticleFilter &...
        matlabshared.tracking.internal.AbstractTrackingFilter & ...
        matlabshared.tracking.internal.AbstractJPDAFilter
 
%% Properties
    properties
        %MeasurementNoise Measurement noise covariance
        % If MeasurementLikelihoodFcn is specified as gaussianLikelihood,
        % this property is used to specify the Gaussian noise covariance of
        % the measurement.
        MeasurementNoise

        %MeasurementFcn A function to calculate the measurements given
        %current particles state.
        % zParticles = MeasurementFcn(particles,varargin)
        MeasurementFcn

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
        ProcessNoise

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
        ProcessNoiseSamplingFcn

        %HasAdditiveProcessNoise A Boolean flag that defines whether the
        %noise affecting the state transition is additive (true) or
        %non-additive (false).
        HasAdditiveProcessNoise = true;

    end

    properties(Hidden, Constant=true, GetAccess= {?trackingPF, ?matlab.unittest.TestCase}) 
        constProcessNoise                   = 1;
        constMeasurementNoise               = 1;
        constHasGaussianProcessNoise        = true;
        constHasAdditiveProcessNoise        = true;
        constDataType                       = 'double';
        constStateCovariance                = 1;
    end

    properties(Access=protected)
        pM; % Length of state
        pN; % Length of measurement
        pW; % Length of process noise
        pV; % Length of measurement noise
        pDataType;
        pMeasurementNoise;
        pProcessNoise;
        pProcessNoiseScalar;
        pMeasurementNoiseScalar;
        pIsValidMeasurementFcn = false;
        pIsCostOfAssignmentAvailable
    end



%% AbstractTrackingFilter methods
    methods
        %------------------------------------------------------------------
        % Constructor method: The construction of trackingPF is different
        % from superclass as it allows one-step construction i.e.
        % initialization and property settings.
        %------------------------------------------------------------------
        function PF = trackingPF(varargin)
            PF@matlabshared.tracking.internal.ParticleFilter;
            setNVProperties(PF,varargin{:});
            if nargin == 0
                PF.StateTransitionFcn = @constvel;
                PF.MeasurementFcn =  @cvmeas;
                PF.MeasurementLikelihoodFcn = PF.defaultLikelihoodFcn;
                PF.pIsCostOfAssignmentAvailable = false;
                PF.ProcessNoiseSamplingFcn = PF.defaultSamplingFcn;
                state = zeros(4,1);
                stateCovariance = eye(4);
                PF.ProcessNoise = eye(4);
                initialize(PF,PF.defaultNumParticles,state,stateCovariance);
            end
        end

        %------------------------------------------------------------------
        % Predict method
        %------------------------------------------------------------------
        function [x_Pred,P_Pred] = predict(PF,varargin)
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
            %
            ensureSamplingFcnIsDefined(PF);
            switch nargout
                case 0
                    predict@matlabshared.tracking.internal.ParticleFilter(PF,varargin{:});
                case 1
                    x_Pred = predict@matlabshared.tracking.internal.ParticleFilter(PF,varargin{:});
                case 2
                    [x_Pred,P_Pred] = predict@matlabshared.tracking.internal.ParticleFilter(PF,varargin{:});
            end
        end
        %------------------------------------------------------------------
        % Correct method: Ensures measurement size, noise and MeasurementLikelihoodFcn
        % is defined before calling correct from superclass method
        %------------------------------------------------------------------
        function [x_corr, P_corr] = correct(PF, z, varargin)
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
            %
            
            % PF should be a scalar filter
            cond = (numel(PF) > 1);
            coder.internal.errorIf(cond, ...
                'fusion:trackingPF:NonScalarFilter', ...
                'trackingPF', 'correct');
            
            validateMeasurementRelatedProperties(PF,z,'correct',varargin{:});
            % The super-class method invokes differently based on number of
            % requested outputs to avoid getStateEstimate calls.
            switch nargout
                case 0
                    correct@matlabshared.tracking.internal.ParticleFilter(PF,z,varargin{:});
                case 1
                    x_corr = correct@matlabshared.tracking.internal.ParticleFilter(PF,z,varargin{:});
                case 2
                    [x_corr,P_corr] = correct@matlabshared.tracking.internal.ParticleFilter(PF,z,varargin{:});
            end
        end

        %------------------------------------------------------------------
        % Correct method for JPDA
        %------------------------------------------------------------------
        function [x_corr, P_corr] = correctjpda(PF, z, jpda, varargin)
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
            %
            
            % number of inputs should be at least 3, ouputs are 0, 1, or 2
            narginchk(3,inf);
            nargoutchk(0,2);
            
            % PF should be a scalar filter
            cond = (numel(PF) > 1);
            coder.internal.errorIf(cond, ...
                'fusion:trackingPF:NonScalarFilter', ...
                'trackingPF', 'correctjpda');
            
            % validate some basic attributes of the measurement matrix
            validateattributes(z,{'numeric'}, ...
                {'real', 'finite', 'nonsparse','2d'},...
                'correctjpda', 'z');
            M=size(z,2);
            coder.internal.errorIf(M<1,'fusion:trackingPF:invalidJPDAMeasurement');
            validateMeasurementRelatedProperties(PF,z(:,1),'correctjpda',varargin{:});
            
            % jpda should have length of number of measurements + 1
            validateattributes(jpda ,{'numeric'}, ...
                {'real', 'finite', 'nonsparse','nonnegative', 'vector', 'numel',...
                M+1 ,'<=',1},'correctjpda', 'jpda');
            
            % jpda coefficients must sum to 1
            coder.internal.errorIf(abs(sum(jpda)-1)>sqrt(eps(PF.pDataType)),...
                'fusion:trackingPF:invalidJPDASum'...
                ,'jpda')
            
            % initialize() must have been called at least once before this operation
            PF.assertInitializeIsCalledAtLeastOnce(); 
            
            % step 1: compute likelihood of all measurements:
            lhoods = zeros(PF.InternalNumParticles,M);
            
            for meas = 1:M
                lhood = invokeMeasurementLikelihoodFcn(PF,z(:,meas),varargin{:});
                coder.internal.errorIf(length(lhood) ~= PF.NumParticles, ...
                'shared_tracking:particle:LikelihoodWrongSize', ...
                feval('sprintf','%g ', PF.NumParticles),...
                feval('sprintf','%g ', length(lhood)));
                lhoods(:,meas) = lhood;                  
            end
            
            % weight likelihood by jpda:
            beta_col = jpda(1:M);
            beta_not = jpda(M+1);
            lhood = bsxfun(@plus,lhoods*beta_col(:),beta_not);
            
            %step 2: Multiply weights by likelihood
            
            % Always add some small fraction to the likelihoods to avoid
            % a singularity when the weights are normalized.
            %
            % Ensure the result follows the orientation of weight
            % regardless of the orientation of likelihood vector
            %
            % Already verified numel of lhood, assign IntervalWeights
            PF.InternalWeights = PF.ParticleManager.weightTimesLikelihood(PF.InternalWeights, lhood + 1e-99);
            
            % Normalize weights, so they sum up to 1
            PF.InternalWeights = PF.InternalWeights / sum(PF.InternalWeights);
            
            % The super-class method invokes differently based on number of
            % requested outputs to avoid getStateEstimate calls.
            % Only run state estimation if user requested outputs
            if nargout == 1
                % Return only state estimate
                x_corr = PF.getStateEstimate;
            elseif nargout == 2
                % Also return state covariance
                [x_corr, P_corr] = PF.getStateEstimate;
            end
            
           %step 3: resample:
           PF.resample;
            
        end
        
        %------------------------------------------------------------------
        % Distance method: cannot be inherited from superclass, because the
        % superclass doesn't have this method
        %------------------------------------------------------------------
        function d = distance(PF, z_matrix ,measurementParameters)
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

            % d = distance(PF, z_matrix, measurementParameters) allows you
            % to define additional parameters that will be used by the
            % PF.MeasurementFcn and PF.MeasurementLikelihoodFcn.
            % measurementParameters should be specified as a cell array,
            % e.g. {1,[2;3]}. If unspecified, it will be assumed as empty
            % cell array.
            %
            % The distance method returns a row vector where each element
            % is a distance associated with corresponding measurement
            % input.

            cond = (numel(PF) > 1);
            coder.internal.errorIf(cond, ...
                'fusion:trackingPF:NonScalarFilter', ...
                'trackingPF', 'distance');

            validateattributes(z_matrix,{'numeric'},...
                {'real','finite','nonsparse','2d'},...
                'trackingPF','z_matrix');

            if nargin == 2
                measurementParameters = cell(0,0);
            end
            
            if isvector(z_matrix)
                zTest = z_matrix(:);
            else
                zTest = z_matrix(1,:).';
            end
            
            validateMeasurementRelatedProperties(PF,zTest,'distance',measurementParameters{:});
            
            % Check expected measurement orientations
            [NxIn,NyIn] = size(z_matrix);

            if NxIn == PF.pN
                isNColumnVector = true;
                numMeas = size(z_matrix,2);            
            elseif NyIn == PF.pN
                isNColumnVector = false;
                numMeas = size(z_matrix,1);
            else
                error(message('fusion:trackingPF:invalidZSize',PF.pN));
            end

            % Initialize distance variable for number of measurements
            d = Inf(1,numMeas);
  
            % Calculate distance from each measurement
            for i = 1:numMeas
                if isNColumnVector
                    zIn = z_matrix(:,i);
                else
                    zIn = z_matrix(i,:);
                end
                if PF.pIsCostOfAssignmentAvailable
                    [~,d(i)] = PF.invokeMeasurementLikelihoodFcn(zIn(:),measurementParameters{:});
                else % Convert likelihood to cost of assignment
                    lhood = PF.invokeMeasurementLikelihoodFcn(zIn(:),measurementParameters{:});
                    weights = PF.Weights;
                    jointLikelihood = sum(weights(:).*lhood(:));
                    d(i) = -2*log(jointLikelihood) - PF.pN*1.837877066409345; % log(2*pi) = 1.837877066409345
                end
            end
        end
        
        %------------------------------------------------------------------
        % likelihood method: cannot be inherited from superclass, because 
        % the superclass doesn't have this method
        %------------------------------------------------------------------
        function lhood = likelihood(PF,z,measParams)
            % likelihood Computes the joint likelihood of a measurement
            % lhood = likelihood(PF,z) computes the joint likelihood of a
            % measurement assuming empty measurement parameters.
            %
            % lhood = likelihood(PF, z, measurementParameters) allows
            % passing additional parameters that will be used by the
            % PF.MeasurementFcn. measurementParameters should be specified
            % as a cell array, e.g., {1,[2;3]}. If unspecified, it will be
            % assumed an empty cell array.
            
            validateattributes(z,{'numeric'},...
                {'real','finite','nonsparse','vector'},...
                'likelihood','z');
            
            if nargin == 2
                measParams = cell(0,0);
            end
            validateMeasurementRelatedProperties(PF,z,'likelihood',measParams{:});
            lhood = PF.invokeMeasurementLikelihoodFcn(z(:),measParams{:});
            weights = PF.Weights;
            lhood = sum(weights(:).*lhood(:));
        end
        
        %------------------------------------------------------------------
        % clone method: Create a copy of the filter
        %------------------------------------------------------------------
        function newPF = clone(PF)
            
            coder.inline('never')
            
            cond = numel(PF) > 1;
            coder.internal.errorIf(cond, ...
                'fusion:trackingPF:NonScalarFilter', ...
                'trackingPF','clone');
            
            objName = str2func(class(PF));
            
            newPF = objName('StateTransitionFcn',PF.StateTransitionFcn,'MeasurementFcn',PF.MeasurementFcn,'HasAdditiveProcessNoise',PF.HasAdditiveProcessNoise);
            copyProperties(PF,newPF);
            
        end

        %------------------------------------------------------------------
        % initialize method: The method for trackingPF works differently 
        % than the one defined in superclass as the initialization is
        % performed in the constructor and dimensions of state,
        % state covariance etc. are already fixed.
        %------------------------------------------------------------------
        function initialize(PF,varargin)
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
            
            numParticles = varargin{1};
            state = varargin{2};
            validateState(PF,state);
            stateCovariance = varargin{3};
            validateStateCovariance(PF,stateCovariance);
            initialize@matlabshared.tracking.internal.ParticleFilter(PF,numParticles,state,stateCovariance,varargin{4:end});
        end
    end
    
%--------------------------------------------------------------------------
    % Methods accessed by the tracker and multi-model filters
    methods (Access = ...
            {?matlabshared.tracking.internal.AbstractTrackingFilter, ...
            ?matlabshared.tracking.internal.AbstractContainsFilters, ...
            ?matlab.unittest.TestCase})
        %------------------------------------------------------------------
        % nullify method: Set the state to zero and state covariance to
        % identity for the particle filter
        %------------------------------------------------------------------
        function nullify(PF)
            % nullify Nullifies the particle filter.
            
            % Create a sampler for generating particles around zeros with 
            % unit covariance.
            pSampler = matlabshared.tracking.internal.NormalDistribution(PF.NumStateVariables); % defaults are zero mean and unit covariance.
            particleSample = pSampler.sample(PF.NumParticles,PF.StateOrientation);
            PF.Particles = particleSample;
            % Assign uniform weight. The set method can control
            % orientation.
            PF.Weights = 1/PF.NumParticles*ones(PF.NumParticles,1);
        end
        
        %-------------------------------------------------------------------
        % modelName method: cannot be inherited from the superclass, because
        % the superclass does not have this method
        %-------------------------------------------------------------------
        function name = modelName(PF)
            % modelName Return the name of the motion model
            %    name = modelName(filter) returns the motion model name,
            %    name.
            coder.internal.assert(coder.internal.is_defined(PF.StateTransitionFcn),...
                'fusion:trackingPF:undefinedStateTransitionFcn')
            name =  func2str(PF.StateTransitionFcn);
        end

        %------------------------------------------------------------------
        % sync method: Synchronizes the filter with another trackingPF
        % object
        %------------------------------------------------------------------
        function sync(PF,PF2)
            % sync: Synchronizes the PF with PF2 such that:
            % 1. Particle locations are same.
            % 2. Particle weights are same.
            % 3. Measurement and ProcessNoise are same.
            % Validate scalar 
            classToExpect = 'trackingPF';
            validateattributes(PF2,{classToExpect},{'scalar'},'trackingPF')
 
            % Validation is performed by set method of each property.
            PF.Particles = PF2.Particles;
            PF.Weights = PF2.Weights;
            PF.ProcessNoise = PF2.ProcessNoise;
            PF.MeasurementNoise = PF2.MeasurementNoise;
        end
        
        %-------------------------------------------------------------------
        % models method: Returns the state transition and measurement models
        %-------------------------------------------------------------------
        function [stm, mm] = models(filter,~)
            % MODELS Return the state transition and measurement models
            %   [stm, mm] = MODELS(filter) returns stm, the state transition
            %   model, and mm, the measurement model. Both are returned as
            %   function handles.
            if filter.HasAdditiveProcessNoise
                stm = filter.StateTransitionFcn;
                mm  = filter.MeasurementFcn;
            else
                v = 0; % Only double precision is supported
                stm = @(state,dt) filter.StateTransitionFcn(state,v,dt);
                mm  = filter.MeasurementFcn;
            end
        end

    end
%--------------------------------------------------------------------------

%% Invoke methods
    methods (Access=protected)
        
        %------------------------------------------------------------------
        % Invoke Measurement Likelihood function
        % The superclass method uses a different signature to call
        % MeasurementLikelihoodFcn.
        %------------------------------------------------------------------
        function [lhood, d] = invokeMeasurementLikelihoodFcn(PF, measurement, varargin)
            % Invoke the MeasurementLikelihoodFcn with predMeasurement
            predMeasurement = PF.MeasurementFcn(PF.Particles,varargin{:});
            if PF.pIsCostOfAssignmentAvailable
                [lhood,d] = PF.MeasurementLikelihoodFcn(PF,predMeasurement,measurement,varargin{:});
            else
                lhood = PF.MeasurementLikelihoodFcn(PF,predMeasurement,measurement,varargin{:});
            end
        end

        %------------------------------------------------------------------
        % Invoke StateTransitionFcn.
        % The superclass method does not have HasAdditiveProcessNoise
        % property to add guassian noise to all particles.
        %------------------------------------------------------------------
        function predictParticles = invokeStateTransitionFcn(PF, varargin)
            ensureProcessNoiseIsDefined(PF);
            noise = PF.ProcessNoiseSamplingFcn(PF);
            if PF.HasAdditiveProcessNoise
                predictParticles = PF.StateTransitionFcn(PF.Particles,varargin{:});
                validateSampleSize(PF,predictParticles);
                predictParticles = predictParticles + noise;
            else
                predictParticles = PF.StateTransitionFcn(PF.Particles,noise,varargin{:});
            end
        end

        function validateSampleSize(PF,predictParticles)
            coder.internal.errorIf(any(size(predictParticles) ~= size(PF.InternalParticles)), ...
                'shared_tracking:particle:PredParticlesWrongSize', ...
                feval('sprintf','(%g-by-%g)', size(PF.InternalParticles,1), size(PF.InternalParticles,2)), ...
                feval('sprintf','(%g-by-%g)', size(predictParticles,1), size(predictParticles,2)));
        end

        %------------------------------------------------------------------
        % Invoke MeasurementFcn
        % The superclass does not have a MeasurementFcn.
        %------------------------------------------------------------------
        function predMeasurement = invokeMeasurementFcn(PF,varargin)
            predMeasurement = PF.MeasurementFcn(PF.Particles,varargin{:});
        end
        
        function validateMeasurementRelatedProperties(PF,z,funcName,varargin)
            ensureLikelihoodFcnIsDefined(PF);
            n = 1 + numel(varargin);
            validateattributes(z,{PF.pDataType},{'real','finite','nonsparse'},funcName,'Measurement');
            validateMeasurementFcn(PF,z,n,funcName,varargin{:});
            if ~coder.internal.is_defined(PF.pN)
                PF.pN = numel(z);
                ensureMeasurementNoiseIsDefined(PF);
            else
                % Can check size here
                validateattributes(z,{PF.pDataType},{'numel',PF.pN},funcName,'Measurement');
            end
        end
    end

%--------------------------------------------------------------------------
    methods (Access = {?matlabshared.tracking.internal.ParticleFilter, ?matlab.unittest.TestCase})

        function stateOrientation = defaultStateOrientation(~)
            stateOrientation = 'column';
        end
        function numParticles = defaultNumParticles(~)
            numParticles = 1000;
        end
        function likelihoodFcn = defaultLikelihoodFcn(~)
            likelihoodFcn = @gaussianLikelihood;
        end
        function resamplingPolicy = defaultResamplingPolicy(~)
            resamplingPolicy = trackingResamplingPolicy;
        end
        function samplingFcn = defaultSamplingFcn(~)
            samplingFcn = @gaussianSampler;
        end
    end
%--------------------------------------------------------------------------
    methods(Hidden)
        function setMeasurementSizes(PF, measurementSize, measurementNoiseSize)
            % Sets the sizes of the measurement (pN) and measurement noise
            % (pV). Both have to be a real, positive, integer, scalar.
            validateattributes(measurementSize, {'numeric'}, {'real', ...
                'positive', 'integer', 'scalar'}, 'trackingPF');
            validateattributes(measurementNoiseSize, {'numeric'}, {'real', ...
                'positive', 'integer', 'scalar'}, 'trackingPF');
            PF.pN = measurementSize;
            PF.pV = measurementNoiseSize;
        end

        function setStateSizes(PF, stateSize, processNoiseSize)
            % Sets the sizes of the state (pM) and process noise
            % (pW). Both have to be a real, positive, integer, scalar.
            validateattributes(stateSize, {'numeric'}, {'real', ...
                'positive', 'integer', 'scalar'}, 'trackingPF');
            validateattributes(processNoiseSize, {'numeric'}, {'real', ...
                'positive', 'integer', 'scalar'}, 'trackingPF');
            PF.pM = stateSize;
            PF.pW = processNoiseSize;
        end
        
        % Hide copy method.
        function newPF = copy(PF)
            cond = numel(PF) > 1;
            coder.internal.errorIf(cond, ...
                'shared_tracking:particle:PolicyCopyNotScalar', ...
                'trackingPF');
            newPF = clone(PF);
        end

        % Hide getStateEstimate function
        function [stateEst,stateCov] = getStateEstimate(PF)
            nargoutchk(0,2);
            
            % Return a fixed-size state. stateEstimate from base-class is a
            % variable-sized state.
            if strcmpi(PF.StateOrientation,'row')
                stateEst = zeros(1,PF.pM,PF.pDataType);
            else
                stateEst = zeros(PF.pM, 1, PF.pDataType);
            end
            if nargout <= 1
                stateEst(:) = getStateEstimate@matlabshared.tracking.internal.ParticleFilter(PF); % use colon assignment to preserve size and data type 
            elseif nargout == 2
                % State covariance size depends on the 'State extraction'
                % technique. Don't fix the size.
                [stateEst(:),stateCov] = getStateEstimate@matlabshared.tracking.internal.ParticleFilter(PF); % use colon assignment to preserve size and data type 
            end
        end
    end

%% Setters/getters
%----------------------------------------------------------------------

    methods (Access = private)
        function validateState(PF, value)
            validateattributes(value, ...
                {PF.pDataType}, {'real', 'finite', 'nonsparse', 'vector'},...
                'trackingPF', 'State');
            % Validate dimensions only when it is known, otherwise set
            % dimensions.
            if coder.internal.is_defined(PF.pM)
                coder.internal.assert(isscalar(value) || numel(value)==PF.pM, ...
                    'fusion:trackingPF:invalidStateDims', PF.pM);
            else
                PF.pM = numel(value);
            end

        end
        %----------------------------------------------------------------------
        function validateStateCovariance(PF, value)
            % Validating that the new state covariance has the correct
            % attributes and dimensions
            validateattributes(value, ...
                {PF.pDataType}, ...
                {'real', 'finite', 'nonsparse', '2d', 'nonempty', 'square'},...
                'trackingPF', 'StateCovariance');
            % Check dimensions only if # of states is known, otherwise set
            % dimensions.
            if coder.internal.is_defined(PF.pM)
                matlabshared.tracking.internal.validateDataDims...
                    ('StateCovariance', value, [PF.pM, PF.pM]);
            end
            matlabshared.tracking.internal.isSymmetricPositiveSemiDefinite...
                ('StateCovariance', value);
            if ~coder.internal.is_defined(PF.pM)
                PF.pM = size(value,1);
            end
        end
    end

    methods
      %----------------------------------------------------------------------
        function set.ProcessNoise(obj, value)
            validateattributes(value, ...
                {obj.pDataType}, ...
                {'real', 'finite', 'nonsparse', '2d', 'nonempty','square'},...
                'trackingPF', 'ProcessNoise');
            if coder.internal.is_defined(obj.pW)
                matlabshared.tracking.internal.validateDataDims('ProcessNoise', value, [obj.pW obj.pW]);
            end
            matlabshared.tracking.internal.isSymmetricPositiveSemiDefinite('ProcessNoise', value);

            % The assignment sets the process noise dimensions directly if:
            % * There is non-additive process noise, and this is the first
            % assignment of Q
            % * The provided Q is not scalar
            if (~obj.HasAdditiveProcessNoise && ~coder.internal.is_defined(obj.pW)) || ~isscalar(value)
                obj.pProcessNoise = value;
                obj.pW = size(value,1);
            else
                % Scalar. Expand only if we know the # of proc noise terms
                if coder.internal.is_defined(obj.pW)
                    obj.pProcessNoise = zeros(obj.pW,obj.pW,obj.pDataType);
                    for idx = 1:obj.pW
                        obj.pProcessNoise(idx, idx) = value(1);
                    end
                else
                    % Unsure about the dimensions. Store the scalar. The
                    % first call to predict must set pProcessNoise
                    obj.pProcessNoiseScalar = value(1);
                end
            end
        end

        %----------------------------------------------------------------------
        function value = get.ProcessNoise(obj)
            % If doing codegen, pProcessNoise must be set before user
            % tries to get its value.
            %
            % In MATLAB we can display pProcessNoiseScalar, which is either
            % the default value in constProcessNoise, or a user assigned
            % scalar value
            coder.internal.assert(isempty(coder.target) || coder.internal.is_defined(obj.pProcessNoise),...
                'fusion:trackingPF:getUndefinedNoise','ProcessNoise');

            if coder.internal.is_defined(obj.pProcessNoise)
                % In codegen this is the only reachable branch. This is to
                % ensure fcn returns value of the same dims in all branches
                value = obj.pProcessNoise;
            else
                value = obj.pProcessNoiseScalar;
            end
        end


        %----------------------------------------------------------------------
        function set.pM(PF,value)
            % Set the number of states in the state transition model
            if ~coder.internal.is_defined(PF.pM)
                PF.pM = value;
            end
            % If the process model has additive noise, the number of
            % process noise terms pW is equal to pM
            if PF.HasAdditiveProcessNoise
                if ~coder.internal.is_defined(PF.pW)
                    PF.pW = value;
                end
            end
        end

        function set.pN(PF,value)
            % Set the number of measurements in the measurement model
            if ~coder.internal.is_defined(PF.pN)
                PF.pN = value;
            end
            % Number of measurement noise is same as measurement
            if ~coder.internal.is_defined(PF.pV)
                PF.pV = value;
            end
        end

        %----------------------------------------------------------------------
        function set.HasAdditiveProcessNoise(PF, value)
            validateattributes(value, {'numeric', 'logical'},...
                {'scalar','binary'},...
                'trackingPF', 'HasAdditiveProcessNoise');
            if value
                PF.HasAdditiveProcessNoise = true;
            else
                PF.HasAdditiveProcessNoise = false;
            end
        end

        %----------------------------------------------------------------------
        function set.MeasurementFcn(PF, value)
            validateattributes(value, {'function_handle'}, ...
                {'nonempty'}, 'trackingPF', 'MeasurementFcn');
            PF.MeasurementFcn = value;
        end

        function set.ProcessNoiseSamplingFcn(PF,value)
            validateattributes(value, {'function_handle'}, ...
                {'nonempty'}, 'trackingPF', 'MeasurementFcn');
            PF.ProcessNoiseSamplingFcn = value;
        end

        function value = get.MeasurementNoise(obj)
            % If doing codegen, pMeasurementNoise must be set before user
            % tries to get its value.
            %
            % In MATLAB we can display pMeasurementNoiseScalar, which is
            % either the default value in constMeasurementNoise, or a user
            % assigned scalar value
            coder.internal.assert(isempty(coder.target) || coder.internal.is_defined(obj.pMeasurementNoise),...
                'fusion:trackingPF:getUndefinedNoise','MeasurementNoise');
            if coder.internal.is_defined(obj.pMeasurementNoise)
                % In codegen this is the only reachable branch. This is to
                % ensure fcn returns value of the same dims in all branches
                value = obj.pMeasurementNoise;
            else
                value = obj.pMeasurementNoiseScalar;
            end
        end

        function set.MeasurementNoise(obj, value)
            validateattributes(value, ...
                {obj.pDataType}, ...
                {'real', 'finite', 'nonsparse', '2d', 'nonempty', 'square'},...
                'trackingPF', 'MeasurementNoise');
            % Every time the measurement function changes, this size may change so no size checking
            if obj.pIsValidMeasurementFcn && coder.internal.is_defined(obj.pV)
                % Skipping this check when meas. fcn is not valid allows
                % users to change the dimensions of R when they change the
                % meas. fcn (not supported in codegen)
                matlabshared.tracking.internal.validateDataDims('MeasurementNoise', value, [obj.pV obj.pV]);
            end
            matlabshared.tracking.internal.isSymmetricPositiveSemiDefinite('MeasurementNoise', value);

            % The assignment sets the measurement dimensions directly if:
            % * There is non-additive measurement noise, and this is the
            % first assignment of R
            % * The provided R is not scalar
            if (~coder.internal.is_defined(obj.pV)) || ~isscalar(value)
                obj.pMeasurementNoise = value;
                obj.pV = size(value,1);
            else
                % Scalar. Expand only if we know the # of meas noise terms
                if coder.internal.is_defined(obj.pV)
                    obj.pMeasurementNoise = zeros(obj.pV,obj.pV,obj.pDataType);
                    for idx = 1:obj.pV
                        obj.pMeasurementNoise(idx, idx) = value(1);
                    end
                else
                    % Unsure about the dimensions. Store the scalar. The
                    % first call to correct/distance must set pMeasurementNoise
                    obj.pMeasurementNoiseScalar = value(1);
                end
            end
        end
    end

%% Private methods: Copying PF, validation of Functions,Properties
    methods(Access = private) 
        
        function copyProperties(PF,newPF)
            % This method is required to copy properties of superclass. The
            % copy method of the superclass creates a trackingPF with empty
            % arguments, which conflicts with the constructor of
            % trackingPF.

            ppPropertiesInternal = {
                'ParticleManager',... (1)
                'StateTransitionFcn','MeasurementLikelihoodFcn',...
                'InternalNumStateVariables','InternalNumParticles'...
                'InternalIsStateVariableCircular',...
                'InternalResamplingMethod',...
                'InternalStateEstimationMethod',...
                'InternalParticles','InternalWeights',...
                'ResamplingMethod','StateEstimationMethod'};
            for kk = coder.unroll(1:numel(ppPropertiesInternal))
                % Copy only if the prop was set in the source obj
                if coder.internal.is_defined(PF.(ppPropertiesInternal{kk}))
                    newPF.(ppPropertiesInternal{kk}) = PF.(ppPropertiesInternal{kk});
                end
            end
            deepCopyProperties = {...
                'UniformDistribution','NormalDistribution',...
                'WrappedUniformDistribution','WrappedNormalDistribution',...
                'ResamplingPolicy'};
            for kk = coder.unroll(1:numel(deepCopyProperties))
                % Copy only if the prop was set in the source obj
                if coder.internal.is_defined(PF.(deepCopyProperties{kk}))
                    newPF.(deepCopyProperties{kk}) = PF.(deepCopyProperties{kk}).copy;
                end
            end

            ppProperties2 = coder.const({'pM','pN','pW','pV',...
                'pMeasurementNoise','MeasurementFcn','pProcessNoise',...
                'pIsValidMeasurementFcn','pIsCostOfAssignmentAvailable',...
                'ProcessNoiseSamplingFcn'});     
            for kk = coder.unroll(1:numel(ppProperties2))
                % Copy only if the prop was set in the source obj
                if coder.internal.is_defined(PF.(ppProperties2{kk}))
                    newPF.(ppProperties2{kk}) = PF.(ppProperties2{kk});
                end
            end
        end

        function ensureMeasurementNoiseIsDefined(obj)
            % Ensure that measurement noise covariance dimension pV,
            % measurement noise covariance R are defined before we perform
            % a correct or distance operation.
            %
            % Perform scalar expansion if pV is defined, but R is not
            % assigned

            coder.internal.assert(coder.internal.is_defined(obj.pV),...
                'fusion:trackingPF:UnknownNumberOfNoiseInputs','MeasurementNoise');
            % The default, or a user provided scalar value, is stored in
            % obj.pMeasurementNoiseScalar. Scalar expand
            if ~coder.internal.is_defined(obj.pMeasurementNoise)
                obj.pMeasurementNoise = matlabshared.tracking.internal.expandScalarValue...
                    (obj.pMeasurementNoiseScalar, [obj.pV, obj.pV]);
            end
        end

        function ensureProcessNoiseIsDefined(obj)
            % Ensure that process noise covariance dimension pW,
            % measurement noise covariance Q are defined before we perform
            % a predict operation.
            %
            % Perform scalar expansion if pW is defined, but Q is not
            % assigned

            coder.internal.assert(coder.internal.is_defined(obj.pW),...
                'fusion:trackingPF:UnknownNumberOfNoiseInputs','ProcessNoise');
            
            % Scalar expand P: The default, or a user provided scalar,
            % is stored in obj.pStateCovarianceScalar
            if ~coder.internal.is_defined(obj.pProcessNoise)
                obj.pProcessNoise = matlabshared.tracking.internal.expandScalarValue...
                    (obj.pProcessNoiseScalar, [obj.pW, obj.pW]);
            end
        end

        function validateMeasurementFcn(PF,z,narginExpected,funcName,varargin)
            % Validate MeasurementFcn
            % 1) Must be defined
            % 2) Number of inputs must match the expected value
            % 3) Must return data of state's class, and dimensions of
            % measurements is as expected
            %
            % Inputs:
            %    narginExpected: Expected number of args in MeasurementFcn
            %    funcName: Name of caller function
            %    varargin: Extra input arguments to MeasurementFcn
            %
            %
            % 1)
            if ~PF.pIsValidMeasurementFcn
                coder.internal.errorIf(isempty(PF.MeasurementFcn),...
                    'fusion:trackingPF:undefinedMeasurementFcn');
            end
            % 2)
            narginActual = nargin(PF.MeasurementFcn);
            coder.internal.errorIf(narginActual ~= narginExpected && ...
                narginActual >= 0, ... %negative if varies
                'fusion:trackingPF:invalidNumOptionalInputs',...
                'MeasurementFcn',funcName);
            % 3)
            if ~PF.pIsValidMeasurementFcn
                validateMeasurementSize(PF,z,funcName,varargin{:});
                PF.pIsValidMeasurementFcn = true;
            end
        end

        function validateMeasurementSize(PF,z,funcName,varargin)
            h = PF.MeasurementFcn;
            x = PF.Particles;
            n = PF.NumParticles;
            measSize = numel(z);
            ztry = h(x, varargin{:});
            validateattributes(ztry,{PF.pDataType},{},funcName,'Measurement');
            measSizeError = ~(size(ztry,1) == measSize);
            numParticlesError = ~(size(ztry,2) == n);
            coder.internal.errorIf(measSizeError,...
                'fusion:trackingPF:MeasurementNonMatchingSize',measSize);
            coder.internal.errorIf(numParticlesError,...
                'fusion:trackingPF:MeasurementNonMatchingParticles',n);
        end

        function ensureLikelihoodFcnIsDefined(PF)
            if ~coder.internal.is_defined(PF.MeasurementLikelihoodFcn)
                PF.MeasurementLikelihoodFcn = PF.defaultLikelihoodFcn;
                PF.pIsCostOfAssignmentAvailable = false;
            elseif ~isequal(PF.MeasurementLikelihoodFcn,PF.defaultLikelihoodFcn) && ...
                    ~coder.internal.is_defined(PF.pIsCostOfAssignmentAvailable)
                if nargout(PF.MeasurementLikelihoodFcn) == 2
                    PF.pIsCostOfAssignmentAvailable = true;
                else
                    PF.pIsCostOfAssignmentAvailable = false;
                end
            else
                PF.pIsCostOfAssignmentAvailable = false;
            end
        end

        function ensureSamplingFcnIsDefined(PF)
            if ~coder.internal.is_defined(PF.ProcessNoiseSamplingFcn)
                PF.ProcessNoiseSamplingFcn = PF.defaultSamplingFcn;
            end
        end
    end


%% ------------------------------------------------------------------------

    % ---------------------------------------------------------------------
    % Property Setters and Parsing Functions
    % ---------------------------------------------------------------------
    methods (Access = private)
        function setNVProperties(PF,varargin)
            % Parsing for simulation
            if coder.target('MATLAB')  
                [stateTransitionFcn, measurementFcn, state, stateCovariance, numParticles, stateOrientation,...
                likelihoodFcn, CircularVar, ResamplingMethod, StateEstimationMethod,processNoise,...
                measurementNoise, hasAdditiveProcessNoise, samplingFcn]...
                    = parseInputsSimulation(PF, varargin{:});
 
            % Parsing for code generation
            else
                [stateTransitionFcn, measurementFcn, state, stateCovariance, numParticles, stateOrientation,...
                likelihoodFcn, CircularVar, ResamplingMethod, StateEstimationMethod,processNoise,...
                measurementNoise, hasAdditiveProcessNoise, samplingFcn]...
                    = parseInputsCodegen(PF,varargin{:});
            end
            % Set the HasAdditiveProcessNoise property before all. These
            % are non-tunable, and needed for correct setting of the
            % remaining of the properties.
            %
            % For codegen support, these must be set during construction.
            % They cannot be changed later.
            PF.HasAdditiveProcessNoise = hasAdditiveProcessNoise;

            % Set data type
            if ~isempty(state)
                classToUse = class(state);
            else
                classToUse = PF.constDataType;
            end

            cond = ~strcmpi(classToUse,'double');
            coder.internal.errorIf(cond,'fusion:trackingPF:nonSupportedSingle');

            if ~coder.internal.is_defined(PF.pDataType)
                PF.pDataType = classToUse;
            end

            % Set state
            if ~isempty(state)
                % Set state first so that filter knows state dimensions
                validateState(PF,state);
            end
 
            % Set state covariance
            if ~isempty(stateCovariance)
                validateStateCovariance(PF,stateCovariance);
                covariance = stateCovariance;
            elseif coder.internal.is_defined(PF.pM)
                covariance = matlabshared.tracking.internal.expandScalarValue(cast(PF.constStateCovariance,classToUse),[PF.pM PF.pM]);
            end

            % Set state transition function
            if ~isempty(stateTransitionFcn)
                PF.StateTransitionFcn = stateTransitionFcn;
            end

            % Set measurement function
            if ~isempty(measurementFcn)
                PF.MeasurementFcn = measurementFcn;
            end

            % Set measurement likelihood function
            if ~isempty(likelihoodFcn)
                PF.MeasurementLikelihoodFcn = likelihoodFcn;
            end

            % Set process noise sampling function
            if ~isempty(samplingFcn)
                PF.ProcessNoiseSamplingFcn = samplingFcn;
            end

           % Set process noise
           PF.pProcessNoiseScalar = cast(PF.constProcessNoise,classToUse);
            if ~isempty(processNoise)
                PF.ProcessNoise = processNoise;
            elseif coder.internal.is_defined(PF.pW)
                % User did not provide Q, but we know the noise dims W.
                % Make the default assignment.
                PF.ProcessNoise = cast(PF.constProcessNoise, classToUse);
            end

           % Set measurement noise
            PF.pMeasurementNoiseScalar = cast(PF.constMeasurementNoise,classToUse);
            if ~isempty(measurementNoise)
                PF.MeasurementNoise = cast(measurementNoise,classToUse);
            elseif coder.internal.is_defined(PF.pV)
                PF.MeasurementNoise = cast(PF.constMeasurementNoise, classToUse);
            end

            % Set resampling method
            if ~isempty(ResamplingMethod)
                PF.ResamplingMethod = ResamplingMethod;
            end
 
            % Set state estimation method
            if ~isempty(StateEstimationMethod)
                PF.StateEstimationMethod = StateEstimationMethod;
            end
    
            
            % NumParticles can only be specified using initialize method.
            % If it's empty, use defaultNumParticles.
            if isempty(numParticles)
                NumParticles = PF.defaultNumParticles();
            else
                NumParticles = numParticles;
            end

            % Initialize the particle filter if state dimensions are known
            if ~isempty(state) 
                if ~isempty(CircularVar)
                    initialize(PF,NumParticles,state,covariance,'CircularVariables',CircularVar,'StateOrientation',stateOrientation);
                else
                    initialize(PF,NumParticles,state,covariance,'StateOrientation',stateOrientation);
                end
            end
        end
%% -------------------------------------------------------------------------
        % Parser for simulation
        function [stateTransitionFcn, measurementFcn, state, stateCovariance, numParticles, stateOrientation, likelihoodFcn, CircularVar, ResamplingMethod, StateEstimationMethod,...
                processNoise, measurementNoise, hasAdditiveProcessNoise, samplingFcn] ...
                = parseInputsSimulation(PF,varargin)

            parser = inputParser;
            parser.FunctionName = mfilename;
            constParams = struct('constStateEstimationMethod', 'mean','constResamplingMethod', 'multinomial');
            parser.addOptional('StateTransitionFcn', []);
            parser.addOptional('MeasurementFcn',     []);
            parser.addOptional('State',      []);
            % Set Likelihood and Sampling Fcn in MATLAB. 
            parser.addParameter('MeasurementLikelihoodFcn',[]);
            parser.addParameter('ProcessNoiseSamplingFcn',[]);
            parser.addParameter('StateCovariance',[]);
            parser.addParameter('NumParticles',[]);
            parser.addParameter('StateOrientation',PF.defaultStateOrientation);
            parser.addParameter('MeasurementNoise',[]);
            parser.addParameter('ProcessNoise',[]);
            parser.addParameter('ResamplingMethod',  constParams.constResamplingMethod);
            parser.addParameter('StateEstimationMethod',constParams.constStateEstimationMethod);
            parser.addParameter('IsStateVariableCircular',[]);
            parser.addParameter('HasAdditiveProcessNoise', ...
                PF.constHasAdditiveProcessNoise);

            % Parse parameters
            parse(parser, varargin{:});
            r = parser.Results;
            stateTransitionFcn      =  r.StateTransitionFcn;
            measurementFcn          =  r.MeasurementFcn;
            state                   =  r.State;
            stateCovariance         =  r.StateCovariance;
            numParticles            =  r.NumParticles;
            measurementNoise        =  r.MeasurementNoise;
            processNoise            =  r.ProcessNoise;
            CircularVar             =  r.IsStateVariableCircular;
            stateOrientation        =  r.StateOrientation;
            ResamplingMethod        =  r.ResamplingMethod;
            StateEstimationMethod   =  r.StateEstimationMethod;
            hasAdditiveProcessNoise =  r.HasAdditiveProcessNoise;
            likelihoodFcn           =  r.MeasurementLikelihoodFcn;
            samplingFcn             =  r.ProcessNoiseSamplingFcn;

        end
        
%% ------------------------------------------------------------------------
        %Parser for Codegen
        function [stateTransitionFcn, measurementFcn, state, stateCovariance, numParticles, stateOrientation, likelihoodFcn, CircularVar, ResamplingMethod, StateEstimationMethod,...
                processNoise, measurementNoise,hasAdditiveProcessNoise, samplingFcn] ...
                = parseInputsCodegen(PF,varargin)
            coder.internal.prefer_const(varargin);

            % Find the position of the first name-property pair, firstPNIndex
            firstNVIndex = matlabshared.tracking.internal.findFirstNVPair(varargin{:});

            parms = struct( ...
                'StateTransitionFcn',           uint32(0), ...
                'MeasurementFcn',               uint32(0), ...
                'State',                        uint32(0), ...
                'StateCovariance',              uint32(0), ...
                'NumParticles',                 uint32(0), ...
                'StateOrientation',             uint32(0), ...
                'ProcessNoise',                 uint32(0), ...
                'MeasurementNoise',             uint32(0), ...
                'MeasurementLikelihoodFcn',     uint32(0), ...
                'ProcessNoiseSamplingFcn',      uint32(0), ...
                'IsStateVariableCircular',      uint32(0), ...
                'ResamplingMethod',             uint32(0),...
                'StateEstimationMethod',        uint32(0),...
                'HasAdditiveProcessNoise',      uint32(0)...
                );
  

            popt = struct( ...
                'CaseSensitivity', false, ...
                'StructExpand',    true, ...
                'PartialMatching', false);

            if firstNVIndex == 1 % Can't assign function_handles on codegen
                defaultStateTransitionFcn = [];
                defaultMeasurementFcn     = [];
                defaultState      = []; % Don't know the size of state
            elseif firstNVIndex == 3 % State Transition, Measurement Fcn provided
                defaultStateTransitionFcn = varargin{1};
                defaultMeasurementFcn     = varargin{2};
                defaultState      = []; % Don't know about State yet. It might be provided as NV pair
            elseif firstNVIndex == 4 % All first 3 inputs are provided
                defaultStateTransitionFcn = varargin{1};
                defaultMeasurementFcn     = varargin{2};
                defaultState      = varargin{3};
            end
            
            defaultResamplingMethod = 'multinomial';
            defaultStateEstimationMethod = 'mean';
            
            optarg              = eml_parse_parameter_inputs(parms, popt, ...
                varargin{firstNVIndex:end});
            stateTransitionFcn  = eml_get_parameter_value(optarg.StateTransitionFcn,...
                defaultStateTransitionFcn, varargin{firstNVIndex:end});
            measurementFcn      = eml_get_parameter_value(optarg.MeasurementFcn,...
                defaultMeasurementFcn, varargin{firstNVIndex:end});
            state               = eml_get_parameter_value(optarg.State,...
                defaultState,varargin{firstNVIndex:end});
            stateCovariance     = eml_get_parameter_value(optarg.StateCovariance,...
                [],varargin{firstNVIndex:end});
            numParticles        = eml_get_parameter_value(optarg.NumParticles,...
                [],varargin{firstNVIndex:end});
            stateOrientation    = eml_get_parameter_value(optarg.StateOrientation,...
                PF.defaultStateOrientation,varargin{firstNVIndex:end});
            measurementNoise    = eml_get_parameter_value(optarg.MeasurementNoise,...
                [],varargin{firstNVIndex:end});
            processNoise        = eml_get_parameter_value(optarg.ProcessNoise,...
                [],varargin{firstNVIndex:end});
            CircularVar         = eml_get_parameter_value(optarg.IsStateVariableCircular,...
                [],varargin{firstNVIndex:end});
            ResamplingMethod    = eml_get_parameter_value(optarg.ResamplingMethod,...
                defaultResamplingMethod, varargin{firstNVIndex:end});
            StateEstimationMethod = eml_get_parameter_value(optarg.StateEstimationMethod,...
                defaultStateEstimationMethod, varargin{firstNVIndex:end});
            hasAdditiveProcessNoise     = eml_get_parameter_value(optarg.HasAdditiveProcessNoise,...
                PF.constHasAdditiveProcessNoise, varargin{firstNVIndex:end});
            likelihoodFcn = eml_get_parameter_value(optarg.MeasurementLikelihoodFcn,...
                [], varargin{firstNVIndex:end});
            samplingFcn = eml_get_parameter_value(optarg.ProcessNoiseSamplingFcn,...
                [], varargin{firstNVIndex:end});

        end

    end


    methods (Access = protected)
        % Display method for trackingPF
        function propGroups = getPropertyGroups(PF)
            stateIdentifier = 'toReplaceStateString';
            stateCovIdentifier = 'toReplaceStateCovString';
            propGroupsStructState = struct('State',stateIdentifier,...
                'StateCovariance',stateCovIdentifier,...
                'IsStateVariableCircular',PF.IsStateVariableCircular);
            propGroupsStructStateTrans = {'StateTransitionFcn','ProcessNoiseSamplingFcn','ProcessNoise','HasAdditiveProcessNoise'};
            propGroupsStructMeas = {'MeasurementFcn','MeasurementLikelihoodFcn','MeasurementNoise'}; 
            propGroupsStructResampling = {'Particles','Weights','ResamplingPolicy','ResamplingMethod'};
            propGroups = [matlab.mixin.util.PropertyGroup(propGroupsStructState),...
                matlab.mixin.util.PropertyGroup(propGroupsStructStateTrans),...
                matlab.mixin.util.PropertyGroup(propGroupsStructMeas),...
                matlab.mixin.util.PropertyGroup(propGroupsStructResampling)];
        end

        function displayScalarObject(PF)
            dispStr = evalc('displayScalarObject@matlabshared.tracking.internal.CustomDisplay(PF)');
            if ~isempty(PF.NumStateVariables)
                if strcmpi(PF.StateOrientation,'row')
                    stateStr = sprintf('[%d%s%d %s]',1,char(10799),PF.NumStateVariables,PF.constDataType);
                else
                    stateStr = sprintf('[%d%s%d %s]',PF.NumStateVariables,char(10799),1,PF.constDataType);
                end
                if strcmpi(PF.StateEstimationMethod,'mean')
                    stateCovStr = sprintf('[%d%s%d %s]',PF.NumStateVariables,char(10799),PF.NumStateVariables,PF.constDataType);
                else
                    stateCovStr = '[]';
                end
            else
                stateStr = '[]';
                stateCovStr = '[]';
            end    
            dispStr = strrep(dispStr,'''toReplaceStateString''',stateStr);
            dispStr = strrep(dispStr,'''toReplaceStateCovString''',stateCovStr);
            fprintf(dispStr);
        end
    end
    methods (Access = public,Hidden)

        function likelihood = gaussianLikelihood(PF,predMeasurement,measurement,varargin)     
            %GAUSSIANLIKELIHOOD Compute likelihood of particles using Gaussian
            %measurement noise
            % likelihood = GAUSSIANLIKELIHOOD(PF,predictedMeasurements,measurement)
            %
            % This function computes the likelihood of particles assuming Gaussian
            % distribution around the measurement with measurement noise governed by
            % the particle filter property MeasurementNoise.
            %
            % The first input to the function is the trackingPF object.
            %
            % predMeasurement is a M-by-N or N-by-M matrix, where M is the number of
            % particles and N is the size of the measurement.
            %
            % measurement is a 1-by-N or N-by-1 vector, where N is the size of the
            % measurement
            %
            % % Example: Compute likelihood of 3-D position measurement
            % % -----------------------------------------------------------------------
            % % Initialize the filter using measurement x = 1; y = 1; z = 1; and
            % % measurement noise as diag([1 1 1]);
            % pf = trackingPF(@constvel,@cvmeas,[1 0 1 0 1 0],'StateCovariance',diag([1 10 1 10 1 10]));
            % % Get predicted measurement using measurement function
            % predMeas = pf.MeasurementFcn(pf.Particles);
            % likelihood = gaussianLikelihood(pf,predMeas,[1;1;1]);
            measNoise = PF.MeasurementNoise;

            if size(predMeasurement,1) == PF.NumParticles
                predMeasurementsCol = predMeasurement';
            else
                predMeasurementsCol = predMeasurement;
            end
            % measurements are concatenated along columns.
            z_error = bsxfun(@minus,predMeasurementsCol,measurement(:));

            % Vectorized implementation of z'/S*z
            zTSinv = z_error'/measNoise;
            zT = z_error';
            dist = dot(zTSinv,zT,2);

            % Likelihood for Gaussian Distribution
            likelihood = (1/det(2*pi*measNoise)^0.5)*exp(-0.5*dist);
        end

        function noise = gaussianSampler(PF)
            %GAUSSIANSAMPLER Generates a sample of noise from ProcessNoise 
            % matrix by assuming Gaussian distribution. 
            % The noise sample is used for predicting particles at the next 
            % time step when process noise is additive or non-additive. 
            % 
            % The input to the function is the trackingPF object.
            %
            % The output of the function is a M-by-N matrix, where M is the
            % number of process noise terms and N is the number of
            % particles.
            %
            % Set the ProcessNoiseSamplingFcn property of the trackingPF to
            % a custom function following the input/output specification 
            % to use a custom noise sampling function. 
            % Example:
            % PF = trackingPF;
            % noise = gaussianSampler(PF);

            processNoise = PF.ProcessNoise;
            sampler = matlabshared.tracking.internal.NormalDistribution(size(processNoise,1));
            sampler.Covariance = processNoise;
            sampler.Mean = zeros(1,size(processNoise,1));
            noise = sampler.sample(PF.NumParticles,PF.StateOrientation);
        end

    end

    methods(Static,Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % Let the coder know about non-tunable parameters so that it
            % can generate more efficient code.
            props = {'pM','pN','pW','pV',...
                'pIsStateColumnVector',...
                'HasAdditiveProcessNoise','HasGaussianProcessNoise',...
                'StateTransitionFcn','MeasurementFcn',...
                'MeasurementLikelihoodFcn','pDataType','pIsCostOfAssignmentAvailable'};
        end
    end
end