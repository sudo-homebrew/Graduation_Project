% ggiwphd Gamma Gaussian Inverse Wishart (GGIW) PHD filter
%   GGIWPHD implements the probability hypothesis density (PHD) using a
%   mixture of Gamma Gaussian Inverse Wishart components. GGIW
%   implementation of PHD filter is typically used to track extended
%   objects using the random matrix model. Extended objects produce
%   multiple detections on the sensor. The GGIW filter is comprised of
%   three different distributions to represent the state of a target.
%
%   1. Gaussian distribution is typically used to represent the kinematic
%   state of the target. For example, a constant velocity state
%   [x;vx;y;vy].
%
%   2. Gamma distribution is used to represent the Poisson rate of
%   detections the object produces on a sensor.
%
%   3. Inverse-Wishart distribution is used to represent the extent of the
%   target. For a target in 2-dimensional space, the extent is defined by a
%   2x2 random positive definite matrix, whose probability density is
%   assumed to be Inverse-Wishart. Similarly for a target in 3-dimensional
%   space, the extent is represented by a 3x3 random matrix. By
%   representing the target's extent with a random positive definite
%   matrix, the filter inherently assumes that the target has an elliptical
%   shape.
%
%   PHD = GGIWPHD returns a GGIW PHD filter with default state transition
%   function (@constvel), measurement function(@cvmeas) and two components
%   in the density at states [0;0;0;0;0;0] and [1;1;1;1;1;1] with unit
%   state covariance in each dimension. The Gamma distribution for both
%   components is initialized with unit shape and unit rate, which
%   represents an expected Poisson rate of 1 with unit variance. The
%   Inverse-Wishart distribution is initialized with degrees of freedom =
%   100 for each component and scale matrices are set to 100*eye(3). The
%   weight of each component in the filter is set to 1.
%
%   PHD = GGIWPHD(states,covariances) allows specifying the states and
%   covariances for Gaussian distribution of each component in the PHD
%   filter. states is a NxM matrix where each column represents a state. M
%   is the number of Components in the density. covariances is a NxNxM
%   matrix, where each page or sheet represents the covariance of each
%   component.
%
%   PHD = GGIWPHD(states,covariances,'Name',value) allows specifying
%   additional properties for the filter during construction using
%   name-value pairs.
%
%   ggiwphd properties:
%   States                      - State of each component in the filter
%   StateCovariances            - Error covariance of each state
%   PositionIndex               - Indices of position in the state
%   StateTransitionFcn          - A function to compute the state at next 
%                                 time step, (f)
%   StateTransitionJacobianFcn  - A function to compute State transition 
%                                 jacobian matrix, (df/dx)
%   ProcessNoise                - A matrix to represent the covariance of 
%                                 process noise (Q)
%   HasAdditiveProcessNoise     - A flag indicating if the process noise is 
%                                 of additive nature
%   Shapes                      - Gamma-distribution parameter shape of 
%                                 each component
%   Rates                       - Gamma-distribution parameter rate of each 
%                                 component
%   GammaForgettingFactors      - A forgetting factor to model the Poisson 
%                                 rate transition
%   DegreesOfFreedom            - IW-distribution parameter degrees of 
%                                 freedom of each component
%   ScaleMatrices               - IW-distribution parameters scale matrix 
%                                 of each component
%   ExtentRotationFcn           - A function to compute rotation transition 
%                                 of the extent.
%   TemporalDecay               - A scaling factor to model the propagation 
%                                 of IW distribution
%   Detections                  - Cell array of objectDetection objects at 
%                                 current time step.
%   MeasurementFcn              - Measurement model (h)
%   MeasurementJacobianFcn      - Measurement jacobian wrt state (dh/dx)
%   HasAdditiveMeasurementNoise - A flag indicating if the measurement 
%                                 noise is of additive nature
%   MaxNumDetections            - Maximum number of allowed detections
%   MaxNumComponents            - Maximum number of allowed components in 
%                                 the filter
%
%   ggiwphd methods:
%   predict           - Predict the density in future
%   correctUndetected - Correct the density with a missed hypothesis
%   correct           - Correct the density with detections
%   likelihood        - Calculate the likelihood of detections
%   append            - Append two densities together
%   merge             - Merge components in the density
%   scale             - Scale the weights of components in the density
%   prune             - Remove components from the density
%   labeledDensity    - Discard components other than a specific label
%   extractState      - Extract target state estimates from the density
%   clone             - Create a copy of the object
%   
%   
%   % Example: Running a ggiwphd filter
%   % Creating a filter with two 3-Dimensional constant velocity components, 
%   % one at [0;0;0;0;0;0] and other at [1;0;1;0;1;0] and unit covariance
%   % in position and 100 covariance in velocity and a process noise with 
%   % unit standard deviation acceleration. By default, ggiwphd creates a 
%   % 3-D extent matrix.
%   states = [zeros(6,1),[1;0;1;0;1;0]];
%   cov1 = diag([1 100 1 100 1 100]);
%   covariances = cat(3,cov1,cov1);
%   phd = ggiwphd(states,covariances,'StateTransitionFcn',@constvel,...
%                 'StateTransitionJacobianFcn',@constveljac,...
%                 'MeasurementFcn',@cvmeas,'MeasurementJacobianFcn',@cvmeasjac,...
%                 'ProcessNoise',eye(3),'HasAdditiveProcessNoise',false,...
%                 'PositionIndex',[1;3;5]);
%
%   % Specifying information about expected measurement rate (number of
%   % detections) at each scan. These values used correspond to expected 
%   % number of detections as 10 and 3.
%   shapes = [1 1.5];
%   rates =  [0.1 0.5];
%   phd.Shapes = shapes;
%   phd.Rates = rates;
%   
%   % Specifying information about extent.
%   dofs = [21 30];
%   scaleMatrix1 = 13*diag([4.7 1.8 1.4].^2);
%   scaleMatrix2 = 22*diag([1.8 4.7 1.4].^2); % rotated 90 degrees
%   scaleMatrices = cat(3,scaleMatrix1,scaleMatrix2);
%   phd.DegreesOfFreedom = dofs;
%   phd.ScaleMatrices = scaleMatrices;
%   phd.ExtentRotationFcn = @(x,dT)eye(3); % No rotation during prediction
%
%   % Predict the filter dT time step ahead.
%   predict(phd,0.1);
%   
%   % Specifying the set of detections.
%   % Consider the filter received 10 detections at the current scan.
%   detections = cell(10,1);
%   rng(2018); % Reproducible results
%   for i = 1:10
%       detections{i} = objectDetection(0,randi([0 1]) + randn(3,1));
%   end
%   phd.Detections = detections;
%  
%   % Calculate likelihoods of different subsets of detections
%   detectionIDs = false(10,2);
%   detectionIDs([1 3 5 7],1) = true;
%   detectionIDs([2 4 6 8],2) = true;
%   lhood = likelihood(phd,detectionIDs);
%
%   % Correct the filter with the subset
%   correct(phd,detectionIDs,lhood);
%   
%   % Merge the components
%   merge(phd,5);
%
%   % Get state estimates
%   targetStates = extractState(phd,0.5);
%
%   See also: partitionDetections, objectDetection, constvel, cvmeas,
%   constveljac, cvmeasjac, constacc, cameas, constaccjac, cameasjac,
%   constturn, ctmeas, constturnjac, ctmeasjac

%   Copyright 2018 The MathWorks, Inc.

%#codegen
    
classdef ggiwphd < fusion.internal.AbstractPHDFilter ...
        & matlabshared.tracking.internal.fusion.CustomDisplay
    
    % Gaussian properties
    properties (Dependent)
        % States State of each component.
        % States is a P-by-N matrix, where P is the dimension of the state and
        % N is the number of components. Each column of the States define
        % the state of each component.
        States
        % StateCovariances Error covariance of each component's state
        % estimate.
        % StateCovariances is a P-by-P-by-N matrix, where each page or
        % sheet of the matrix defines the covariance of each component.
        StateCovariances
    end
    
    % Gamma properties
    properties (Dependent)
        % Shapes Gamma parameter Shape of each component.
        % Shapes is a 1-by-N vector, where N is the number of components in
        % the density.
        Shapes
        % Rates Gamma parameter Rate of each component.
        % Rates is a 1-by-N vector, where N is the number of components in
        % the density.
        Rates
    end
    
    % IW properties
    properties (Dependent)
        % DegreesOfFreedom Inverse-Wishart parameter degrees of freedom of
        % each component.
        % DegreesOfFreedom is a 1-by-N vector, where N is the number of
        % components in the density.
        DegreesOfFreedom
        % ScaleMatrices Inverse-Wishart parameter scale matrix of each
        % component.
        % ScaleMatrices is a d-by-d-by-N matrix, where d is the number of
        % dimensions of target's extent and N is the number of components.
        ScaleMatrices
    end
    
    % Private copies
    properties (Access = protected)
        pStates
        pStateCovariances
        pShapes
        pRates
        pScaleMatrices
        pDegreesOfFreedom
        pTemporalDecay
    end
    
    % Allow static memory allocation
    properties (Access = protected)
        pMaxNumDetections
    end
    
    % Gaussian filtering properties
    properties
        % StateTransitionFcn A function calculating the state at the next
        % time step, (f).
        %   Specify the transition of state between times as a function
        %   that calculates an M dimensional state vector at time k given
        %   the state vector at time k-1. The function may take additional
        %   input parameters if needed, e.g., control inputs or the size of
        %   the time step.
        %
        %   If HasAdditiveProcessNoise is true, the function should have
        %   one of the following signatures:
        %       x(k) = StateTransitionFcn(x(k-1))
        %       x(k) = StateTransitionFcn(x(k-1), dT)
        %   where:
        %       x(k) is the (estimated) state at time k.
        %       dT is the time-step.
        %
        %   If HasAdditiveProcessNoise is false, the function should have
        %   one of the following signatures:
        %       x(k) = StateTransitionFcn(x(k-1), w(k-1))
        %       x(k) = StateTransitionFcn(x(k-1), w(k-1), dT)
        %   where:
        %       x(k) is the (estimated) state at time k.
        %       w(k) is the process noise at time k.
        %       dT is the time-step.
        StateTransitionFcn
        
        % StateTransitionJacobianFcn Jacobian of StateTransitionFcn
        %   Specify the function that calculates the Jacobian of the
        %   StateTransitionFcn, f. This function must take the same input
        %   arguments as the StateTransitionFcn. If not specified, the
        %   Jacobian will be numerically computed at every call to predict,
        %   which may increase processing time and numerical inaccuracy.
        %
        %   If HasAdditiveProcessNoise is true, the function should have
        %   one of the following signatures:
        %       dfdx(k) = StateTransitionJacobianFcn(x(k))
        %       dfdx(k) = StateTransitionJacobianFcn(x(k), dT)
        %   where:
        %       dfdx(k)    - Jacobian of StateTransitionFcn with respect
        %                    to states x, df/dx, evaluated at x(k). An
        %                    M-by-M matrix where M is the number of states.
        %       x(k)       - Estimated state at time k.
        %       dT         - time-step.
        %
        %   If HasAdditiveProcessNoise is false, the function should have
        %   one of the following signatures:
        %       [dfdx(k), dfdw(k)] = StateTransitionJacobianFcn(x(k), w(k))
        %       [dfdx(k), dfdw(k)] = ...
        %           StateTransitionJacobianFcn(x(k), w(k), dT)
        %   where:
        %       dfdx(k)    - Jacobian of StateTransitionFcn with respect
        %                    to states x, df/dx, evaluated at x(k), w(k).
        %                    An M-by-M matrix where M is the number of
        %                    states.
        %       dfdw(k)    - Jacobian of StateTransitionFcn with respect to
        %                    process noise w, df/dw, evaluated at x(k),
        %                    w(k).
        %                    An M-by-W matrix where W is the number of
        %                    process noise terms in w.
        %       x(k)       - Estimated state at time k.
        %       w(k)       - Process noise at time k.
        %       dT         - time-step.
        %
        %   Default: StateTransitionJacobianFcn = []
        StateTransitionJacobianFcn
        
        % MeasurementFcn A function calculating the measurement, (h).
        %   Specify the transition from state to measurement as a function
        %   that produces an N-element measurement vector for an M-element
        %   state. The function may take additional input parameters if
        %   needed, e.g., in order to specify the sensor position.
        %
        %   If HasAdditiveMeasurementNoise is true, the function should
        %   have one of the following signatures:
        %       z(k) = MeasurementFcn(x(k))
        %       z(k) = MeasurementFcn(x(k), parameters)
        %   where:
        %       x(k) is the (estimated) state at time k.
        %       z(k) is the (estimated) measurement at time k.
        %       parameters are MeasurementParameters provided in the
        %       objectDetections set in Detections property
        %
        %   If HasAdditiveMeasurementNoise is false, the function should
        %   have one of the following signatures:
        %       z(k) = MeasurementFcn(x(k), v(k))
        %       z(k) = MeasurementFcn(x(k), v(k), parameters)
        %   where:
        %       x(k) is the (estimated) state at time k.
        %       z(k) is the (estimated) measurement at time k.
        %       v(k) is the measurement noise at time k.
        %       parameters are MeasurementParameters provided in the
        %       objectDetections set in Detections property
        MeasurementFcn
        
        % MeasurementJacobianFcn Jacobian of MeasurementFcn
        %   Specify the function that calculates the Jacobian of the
        %   MeasurementFcn, h. This function must take the same input
        %   arguments as the MeasurementFcn. If not specified, the
        %   Jacobian will be numerically computed at every call to correct,
        %   which may increase processing time and numerical inaccuracy.
        %
        %   If HasAdditiveMeasurementNoise is true, the function should
        %   have one of the following signatures:
        %       dhdx(k) = MeasurementJacobianFcn(x(k))
        %       dhdx(k) = MeasurementJacobianFcn(x(k), parameters)
        %   where:
        %       dhdx(k)    - Jacobian of MeasurementFcn with respect to
        %                    states x, dh/dx, evaluated at x(k). An N-by-M
        %                    matrix where N is the number of measurements,
        %                    M is the number of states.
        %       x(k)       - Estimated state at time k.
        %       parameters - MeasurementParameters provided in the
        %                    objectDetections set in Detections property
        %
        %   If HasAdditiveMeasurementNoise is false, the function should
        %   have one of the following signatures:
        %       [dhdx(k), dhdv(k)] = MeasurementJacobianFcn(x(k), v(k))
        %       [dhdx(k), dhdv(k)] = ...
        %           MeasurementJacobianFcn(x(k), v(k), parameters)
        %   where:
        %       dhdx(k)    - Jacobian of MeasurementFcn with respect to
        %                    states x, dh/dx, evaluated at x(k), v(k). An
        %                    N-by-M matrix. N is the number of
        %                    measurements, M is the number of states.
        %       dhdv(k)    - Jacobian of MeasurementFcn with respect to
        %                    measurement noise v, dh/dv, evaluated at x(k),
        %                    v(k). An N-by-V matrix where V is the number
        %                    of measurement noise terms in v.
        %       x(k)       - Estimated state at time k.
        %       v(k)       - Measurement noise at time k.
        %       parameters - MeasurementParameters provided in the
        %                    objectDetections set in Detections property
        %
        %   Default: MeasurementJacobianFcn = []
        MeasurementJacobianFcn

        % HasAdditiveMeasurementNoise A Boolean flag that defines whether
        % the noise affecting the measurement is additive (true) or
        % non-additive (false).
        %
        % This property can only be set during filter construction.
        HasAdditiveMeasurementNoise
        
        % HasAdditiveProcessNoise A Boolean flag that defines whether the
        % noise affecting the state transition is additive (true) or
        % non-additive (false).
        %
        % This property can only be set during filter construction.
        HasAdditiveProcessNoise
        
        % ProcessNoise Process noise covariance (Q)
        %   If HasAdditiveProcessNoise is true: specify the covariance of
        %   process noise as a scalar or an M-by-M matrix.
        %
        %   If HasAdditiveProcessNoise is false: specify the covariance of
        %   process noise as a W-by-W matrix, where W is the number of the
        %   process noise terms.
        %
        %   Default: eye(M) if HasAdditiveProcessNoise is true
        ProcessNoise
        
        % PositionIndex Indices of position in the state
        % For example, [1 3 5] for a constant velocity state defined according
        % to the convention [x;vx;y;vy;z;vz].
        PositionIndex
    end
    
    % Gamma filtering properties
    properties (Dependent)
        % GammaForgettingFactors Forgetting factors to model the
        % exponential forgetting of the measurement rate. During state
        % transition shape and rate of the Gamma distribution is scaled
        % down by this factor. This leads to a transition of measurement
        % rate, where the expected value remains constant and the variance
        % increases proportional to the forgetting rate.
        GammaForgettingFactors
    end
    
    properties (Access = private)
        pGammaForgettingFactors
    end
    
    % IW filtering properties
    properties
        % ExtentRotationFcn A function handle to model the rotation
        % transition of the target's extent. The ExtentRotationFcn allows
        % predicting the rotation of the extent when orientation or angular
        % velocity is estimated in the state vector. For example, the
        % constant-turn motion model.
        % R = ExtentRotationFcn(x,dT) must return a matrix with same
        % dimensions as the extent of the target. R defines the
        % delta-rotation of the extent after time-step dT.
        % R is a 3-by-3 matrix if extent is 3-dimensional.
        % R is a 2-by-2 matrix if the extent is 2-dimensional.
        % During prediction, the effect of delta-rotation, R is modeled as
        % extentT = R*extent*R';
        ExtentRotationFcn
    end
    properties (Dependent)
        % TemporalDecay A scalar value to control the variance of the
        % extent uncertainty during prediction. Lower the temporal decay,
        % higher the variance-increase during prediction.
        TemporalDecay
    end
    
    properties (Dependent)
        % Detections Current set of detections
        % Detections is a cell array of objectDetection objects.
        % Use the property to set the current set of detections for the
        % filter.
        % Default = {}
        Detections
        
        % MaxNumDetections Maximum number of detections
        % A scalar value representing the maximum number of allowed
        % detections.
        % Default = 1000
        MaxNumDetections
    end
    
    % Mixture properties
    properties (Dependent)
        % Weights Weight of each component.
        Weights
        
        % Labels Identifier for each component. Each component can be
        % assigned a label. The Labeling of a component allows maintaining
        % trajectories over time. Once a label is assigned to a component,
        % it is maintained during prediction and correction stages.
        Labels
    end
    
    properties (Access = private)
        pWeights;
        pLabels;
    end
    
    properties (Access = private)
        pFilter
        pNumDetections
    end
    
    properties (SetAccess = private)
        % NumComponents (read-only) Current number of components in the filter.
        NumComponents
    end
    
    % Memory control properties
    properties
        % MaxNumComponents Maximum number of allowed components in the
        % filter.
        % You can only set this property during construction of the object.
        MaxNumComponents
    end
    
    % Some private properties for memory allocation and validation.
    properties (Access = {?ggiwphd, ?matlab.unittest.TestCase})
        pExpectedMeasurements
        pMeasurementJacobians
        pMeasurementToNoiseJacobians;
        pDetectionBuffer
        pM
        pN
        pV
        pW
        pDataType
        pIsPredictReady = false;
        pIsDetectReady = false;
    end
    
    methods
        % Constructor
        function obj = ggiwphd(states,covariances,varargin)
            % obj = GGIWPHD
            % obj = GGIWPHD(states,covariances);
            % obj = GGIWPHD(states,covariances,'Name',value);
            if nargin == 0
                states = [zeros(6,1) ones(6,1)];
                covariances = repmat(eye(6),[1 1 2]);
                inputArgs = getDefaultArgs(obj);
            else
                inputArgs = varargin;
            end
            allocateMemory(obj,states,covariances,inputArgs{:});
            matlabshared.fusionutils.internal.setProperties(obj,numel(inputArgs),inputArgs{:});
            obj.States = states;
            obj.StateCovariances = covariances;
            obj.pFilter = fusion.internal.ggiwFilter;
            if ~coder.internal.is_defined(obj.HasAdditiveProcessNoise)
                if coder.internal.is_defined(obj.ProcessNoise)
                    obj.HasAdditiveProcessNoise = size(obj.ProcessNoise,1) == obj.pM;
                else
                    obj.HasAdditiveProcessNoise = true;
                end
            end
        end
        
        % predict method
        function predict(obj,dT)
            % PREDICT Predict the filter dT time ahead.
            % PREDICT(obj,dT) predicts the filter dT time step ahead. The
            % function changes the state of the object,obj, with the
            % predicted state.
            testPredictionReadiness(obj);
            validateattributes(dT,{'single','double'},{'real','finite','nonsparse','scalar'},'predict','dT',2);
            f = obj.StateTransitionFcn;
            df = obj.StateTransitionJacobianFcn;
            M = obj.ExtentRotationFcn;
            n = obj.NumComponents;
            if n > 0
                tau = obj.TemporalDecay;
                nk = obj.GammaForgettingFactors;
                Q = obj.ProcessNoise;
                hasAPN = obj.HasAdditiveProcessNoise;
                [x,P,alpha,beta,nu,V] = getStates(obj);
                [x,P,alpha,beta,nu,V] = obj.pFilter.predict(x,P,alpha,beta,nu,V,nk,f,df,M,Q,dT,tau,hasAPN);
                setStates(obj,x,P,alpha,beta,nu,V);
            end
        end
        
        
        function lhood = likelihood(obj,detectionIndices,varargin)
            % LIKELIHOOD Calculate likelihood of detections.
            % lhood = LIKELIHOOD(obj,detectionIndices) returns the
            % log-likelihood for each column of detectionIndices.
            % detectionIndices is a logical matrix of size m-by-p, where m
            % is the number of total detections and p is the number of
            % detection cells.
            % lhood is a N-by-p matrix, where each column represents the
            % log-likelihood of N components in the obj against each cell 
            % of detection.   
            coder.internal.assert(obj.pIsDetectReady,'fusion:ggiwphd:detectNotReady','likelihood');
            validateattributes(detectionIndices,{'logical'},{'nrows',obj.pNumDetections},'likelihood','detectionIndices',2);
            numCells = size(detectionIndices,2);
            m = obj.pNumDetections;
            assert(m <= obj.pMaxNumDetections,message('fusion:ggiwphd:exceedsMaxNumDetections'));
            n = obj.NumComponents;
            lhood = zeros(n,numCells,obj.pDataType);
            [z,R] = getCurrentMeasurementAndNoise(obj);
            zExp = obj.pExpectedMeasurements(:,1:n);
            H = obj.pMeasurementJacobians(:,:,1:n);
            [x,P,alpha,beta,nu,V] = getStates(obj);
            
            % Check if cardinality must be calculated based on the sensor.
            if nargin > 2
               validateattributes(varargin{1},{'fusion.internal.AbstractTrackingSensorConfiguration'},...
                   {'nonempty'},'ggiwphd','likelihood',3);
               sensorConfig = varargin{1};
               isPointSensor = sensorConfig.MaxNumDetsPerObject == 1;
            else
               isPointSensor = false;
            end
            if ~isPointSensor
                % Construct cardinality matrix.
                alphaKMatrix = bsxfun(@plus,alpha(:),0:m);
                alphaMatrix = repmat(alpha(:),1,m+1);
                betaMatrix = repmat(beta(:),1,m+1);
                betaKMatrix = betaMatrix + 1;
                logLhood = gammaln(alphaKMatrix) - gammaln(alphaMatrix) + ...
                    alphaMatrix.*log(betaMatrix) - alphaKMatrix.*log(betaKMatrix);
                cardLhood = logLhood;
            else
                % Only cells with 1 detections must be allowed.
                cardLhood = -inf(n,m+1,obj.pDataType);
                cardLhood(:,2) = 1;
            end
            
            % Position IDs
            posID = obj.PositionIndex;
            
            % Number of detections per cell
            detsPerCell = sum(detectionIndices,1);
            
            % Loop over each cell.
            for i = 1:numCells
                thisIndices = detectionIndices(:,i);
                thisZ = z(:,thisIndices);
                thisR = R(:,:,thisIndices);
                if obj.HasAdditiveMeasurementNoise
                    measLikelihood = fusion.internal.giwLikelihood(x,P,nu,V,thisZ,zExp,thisR,posID,H);
                else
                    U = obj.pMeasurementToNoiseJacobians(:,:,1:n);
                    measLikelihood = fusion.internal.giwLikelihood(x,P,nu,V,thisZ,zExp,thisR,posID,H,U);
                end
                cardinalitylhood = cardLhood(:,detsPerCell(i) + 1);
                lhood(:,i) = measLikelihood + cardinalitylhood;
            end
        end
        
        % correct method
        function correct(obj,detectionIndices,cellLikelihood,varargin)
            % CORRECT Correct the filter with subsets of obj.Detections
            % CORRECT(obj,detectionIndices,likelihood)
            % obj is a ggiwphd object.
            %
            % detectionIndices is a logical matrix of size m-by-P, where m 
            % is the number of detections denoted by property Detections 
            % and p is the number of cells. Each column represents one
            % particular subset of the detection. detectionIndices(i,j)
            % represents if ith detection is included in the jth subset.
            % 
            % likelihood is a matrix of size n-by-P, defining the 
            % likelihood of each component against each subset of
            % detections.
            narginchk(3,3);
            n = obj.NumComponents;
            coder.internal.assert(obj.pIsDetectReady,'fusion:ggiwphd:detectNotReady','correct');
            validateattributes(detectionIndices,{'logical'},{'nrows',obj.pNumDetections},'correct','detectionIndices',2);
            numCells = size(detectionIndices,2);
            validateattributes(cellLikelihood,{'single','double'},{'real','finite','nonsparse','nrows',n,'ncols',numCells},'correct','detectionLikelihood',3);
            
            % Each cell must contain at least 1 detection
            coder.internal.assert(all(sum(detectionIndices,1) >= 1),'fusion:ggiwphd:mustSumToAtleastOne');
            
            m = obj.pNumDetections;
            assert(m <= obj.pMaxNumDetections,message('fusion:ggiwphd:exceedsMaxNumDetections'));
            
            M = obj.MaxNumComponents;
            assert(n <= M,message('fusion:ggiwphd:exceedsMaxNumComponents'));
            
            [z,R] = getCurrentMeasurementAndNoise(obj);
            
            zExp = obj.pExpectedMeasurements(:,1:n);
            H = obj.pMeasurementJacobians(:,:,1:n);
            U = obj.pMeasurementToNoiseJacobians(:,:,1:n);
            currentWeights = obj.Weights;
            forgettingFactors = obj.GammaForgettingFactors;
            labels = obj.pLabels(1,1:n);
            
            [x,P,alpha,beta,nu,V] = getStates(obj);
            posID = obj.PositionIndex;
            
            % Start with 0 components
            n = 0;
            
            % Correct and add component for each cell
            for i = 1:numCells
                compLikelihood = cellLikelihood(:,i);
                
                % Only correct where likelihood is greater than eps
                permitCorrection = compLikelihood(:).*currentWeights(:) > eps;
                
                % Get states of components permitted correction.
                xCor = x(:,permitCorrection);
                PCor = P(:,:,permitCorrection);
                nuCor = nu(1,permitCorrection);
                VCor = V(:,:,permitCorrection);
                alphaCor = alpha(1,permitCorrection);
                betaCor = beta(1,permitCorrection);
                zExpCor = zExp(:,permitCorrection);
                HCor = H(:,:,permitCorrection);
                
                % Number of components after correction with this cell.
                nNew = sum(permitCorrection);
                
                if nNew > 0
                    % Inform codegen that n < M
                    assert(n + nNew <= M,message('fusion:ggiwphd:exceedsMaxNumComponents'));
                    
                    % New indices
                    newIndices = n+(1:nNew);
                    
                    zCell = z(:,detectionIndices(:,i));
                    RCell = R(:,:,detectionIndices(:,i));
                    % Correct states
                    if obj.HasAdditiveMeasurementNoise
                        [xk,Pk,alphak,betak,vk,Vk] = obj.pFilter.correct(xCor,PCor,alphaCor,betaCor,nuCor,VCor,zCell,zExpCor,RCell,posID,HCor);
                    else
                        Ucor = U(:,:,permitCorrection);
                        [xk,Pk,alphak,betak,vk,Vk] = obj.pFilter.correct(xCor,PCor,alphaCor,betaCor,nuCor,VCor,zCell,zExpCor,RCell,posID,HCor,Ucor);
                    end
                    setStates(obj,xk,Pk,alphak,betak,vk,Vk,newIndices);
                    
                    % Set weights and labels and forgetting factors.
                    weights = currentWeights(permitCorrection);
                    likelihood = compLikelihood(permitCorrection);
                    obj.pWeights(newIndices) = weights(:).*likelihood(:);
                    obj.pLabels(newIndices) = labels(permitCorrection);
                    obj.pGammaForgettingFactors(newIndices) = forgettingFactors(permitCorrection);
                    n = n + nNew;
                end
            end
            
            % Set number of components
            obj.NumComponents = n;
        end
        
        % correct undetected
        function correctUndetected(obj,Pd,PzeroDets)
            % CORRECTUNDETECTED Correct the filter with the hypothesis that
            % no component in the density was detected.
            % CORRECTUNDETECTED(obj,Pd);
            % CORRECTUNDETECTED(obj,Pd,PzeroDets);
            % obj is a ggiwphd object
            % Pd is a vector of length n, where n is the current number of
            % components. Each element of Pd defines the probability of
            % detection of each component.
            % PzeroDets is a optional input vector of length n, where n is
            % the current number of components. Each element of PzeroDets
            % defines the conditional probability of generating no
            % detections by each component upon detection. If unspecified,
            % the filter calculates the probability of zero detections
            % using its current Gamma distribution prior.
            n = obj.NumComponents;
            validateattributes(Pd,{'single','double'},{'real','finite','nonsparse','vector','numel',n,'>=',0,'<=',1},'correctUndetected','Pd',2);
            if nargin > 2
                validateattributes(PzeroDets,{'single','double'},{'real','finite','nonsparse','vector','numel',n,'>=',0,'<=',1},'correctUndetected','PzeroDets',2);
            else
                PzeroDets = probZeroDetections(obj);
            end
            if n > 0
                currentWeights = obj.Weights;
                % Bi-modal weight updates.
                w1 = 1 - Pd(:) + eps(Pd(1));
                w2 = Pd(:).*PzeroDets(:) + eps(Pd(1));
                qd = w1 + w2;
                updatedWeights = qd(:).*currentWeights(:);
                obj.pWeights(1,1:n) = updatedWeights;
                obj.pWeights(1,n+1:end) = obj.scalarZero;
                beta = obj.Rates;
                % Merge betas
                beta2 = beta(:) + 1;
                beta1 = beta(:);
                w = w1(:) + w2(:);
                betaUpdated = w./(w1./beta1 + w2./beta2);
                obj.pRates(1,1:n) = betaUpdated;
            end
        end
        
        
        % merge method
        function merge(obj,threshold)
            % MERGE Merge components in the filter
            % MERGE(obj,threshold) merges components in the filter whose
            % Kullback-Leibler difference is below the input, threshold.
            
            validateattributes(threshold,...
                {'single','double'},{'real','finite','scalar','nonsparse'},'merge','Threshold',2);
            n = obj.NumComponents;
            M = obj.MaxNumComponents;
            assert(n <= M,message('fusion:ggiwphd:exceedsMaxNumComponents'));
            if n > 0
                [x,P,alpha,beta,nu,V] = getStates(obj);
                w = obj.Weights;
                label = obj.Labels;
                gaussDistance = fusion.internal.gaussKLDiff(x,P);
                iwDistance = fusion.internal.iwKLDiff(nu,V);
                gammaDistance = fusion.internal.gammaKLDiff(alpha,beta);
                totalDistance = gaussDistance + iwDistance + gammaDistance;
                clusters = fusion.internal.clusterUsingDistance(totalDistance,threshold);
                numComps = cast(max(clusters),obj.pDataType);
                
                % Allocated memory
                xMerge = x(:,1:numComps);
                PMerge = P(:,:,1:numComps);
                alphaMerge = alpha(1,1:numComps);
                betaMerge = beta(1,1:numComps);
                vMerge = nu(1,1:numComps);
                VMerge = V(:,:,1:numComps);
                wMerge = w(1,1:numComps);
                labelMerge = label(1,1:numComps);
                
                for i = 1:numComps
                    thisCluster = clusters == i;
                    xC = x(:,thisCluster);
                    PC = P(:,:,thisCluster);
                    alphaC = alpha(1,thisCluster);
                    betaC = beta(1,thisCluster);
                    vC = nu(1,thisCluster);
                    VC = V(:,:,thisCluster);
                    wC = w(1,thisCluster);
                    labelC = label(1,thisCluster);
                    [wM,xM,PM,alphaM,betaM,vM,VM,labelM] = fusion.internal.ggiwMerge(wC,xC,PC,alphaC,betaC,vC,VC,labelC);
                    xMerge(:,i) = xM;
                    PMerge(:,:,i) = PM;
                    alphaMerge(1,i) = alphaM;
                    betaMerge(1,i) = betaM;
                    vMerge(1,i) = vM;
                    wMerge(1,i) = wM;
                    VMerge(:,:,i) = VM;
                    labelMerge(1,i) = labelM;
                end
                
                obj.NumComponents = numComps;
                setStates(obj,xMerge,PMerge,alphaMerge,betaMerge,vMerge,VMerge);
                obj.pWeights(1,1:numComps) = wMerge;
                obj.pWeights(1,(numComps+1):end) = obj.scalarZero;
                obj.pLabels(1,(numComps+1):end) = obj.scalarZero;
            end
        end
        
        % prune
        function prune(obj, pruneIndices)
            % PRUNE Prune the filter by removing selected components.
            % PRUNE(obj, pruneIndices) removes the components for which
            % pruneIndices is true.
            % pruneIndices is a logical vector of length N, where N is the
            % number of components in the filter.
            
            n = obj.NumComponents;
            validateattributes(pruneIndices,...
                {'logical'},{'numel',n},'prune','Indices',2);
            
            [x,P,alpha,beta,nu,V] = getStates(obj);
            weights = obj.Weights;
            labels = obj.Labels;
            accept = ~pruneIndices;
            xPruned = x(:,accept);
            PPruned = P(:,:,accept);
            vPruned = nu(1,accept);
            VPruned = V(:,:,accept);
            alphaPruned = alpha(1,accept);
            betaPruned = beta(1,accept);
            labels = labels(1,accept);
            numComponents = sum(accept);
            obj.NumComponents = numComponents;
            setStates(obj,xPruned,PPruned,alphaPruned,betaPruned,vPruned,VPruned);
            obj.pLabels(1:numComponents) = labels;
            obj.pWeights(1:numComponents) = weights(1,accept);
        end
        
        % add two densities
        function append(obj,obj2)
            % APPEND Append two filters.
            % APPEND (obj, obj2) appends the components in obj2 to the
            % components in obj. The total number of components in obj must
            % not exceed MaxNumComponents.
            % obj2 is a ggiwphd object.
            n = obj.NumComponents;
            m = obj2.NumComponents;
            assert(n + m <= obj.MaxNumComponents,message('fusion:ggiwphd:exceedsMaxNumComponents'));
            newStates = obj2.States;
            newCovariances = obj2.StateCovariances;
            newAlphas = obj2.Shapes;
            newBetas = obj2.Rates;
            newVs = obj2.ScaleMatrices;
            newvs = obj2.DegreesOfFreedom;
            weights = obj2.Weights;
            forgettingFactors = obj2.GammaForgettingFactors;
            labels = obj2.Labels;
            newIndices = n+(1:m);
            obj.pStates(:,newIndices) = newStates(:,1:m);
            obj.pStateCovariances(:,:,newIndices) = newCovariances(:,:,1:m);
            obj.pShapes(1,newIndices) = newAlphas(1,1:m);
            obj.pRates(1,newIndices) = newBetas(1,1:m);
            obj.pDegreesOfFreedom(1,newIndices) = newvs(1,1:m);
            obj.pScaleMatrices(:,:,newIndices) = newVs(:,:,1:m);
            obj.pWeights(1,newIndices) = weights(1,1:m);
            obj.NumComponents = n + m;
            obj.pLabels(1,newIndices) = labels(1,1:m);
            obj.pGammaForgettingFactors(1,newIndices) = forgettingFactors(1,1:m);
        end
        
        % clone density
        function obj2 = clone(obj)
            % CLONE clones the filter
            % obj2 = CLONE(obj) returns a clone of the object, obj.
            [x,P,~,~,~,V] = getStates(obj);
            % Construct with memory-related and non-tunable properties
            obj2 = ggiwphd(x,P,'MaxNumComponents',obj.MaxNumComponents,...
                'ScaleMatrices',V,...
                'HasAdditiveProcessNoise',obj.HasAdditiveProcessNoise,...
                'HasAdditiveMeasurementNoise',obj.HasAdditiveMeasurementNoise,...
                'ProcessNoise',obj.ProcessNoise);
            
            propList = {'StateTransitionFcn',...
                'MeasurementFcn',...
                'StateTransitionJacobianFcn',...
                'MeasurementJacobianFcn',...
                'PositionIndex',...
                'pShapes',...
                'pRates',...
                'pGammaForgettingFactors',...
                'pN','pM','pV','pW',...
                'pDegreesOfFreedom'...
                'pWeights',...
                'pLabels',...
                'ExtentRotationFcn',...
                'ProcessNoise',...
                'pStates',...
                'pStateCovariances',...
                'pScaleMatrices',...
                'TemporalDecay',...
                'pDetectionBuffer',...
                'pIsPredictReady',...
                'pIsDetectReady',...
                'pNumDetections',...
                'pMaxNumDetections',...
                'pExpectedMeasurements',...
                'pMeasurementJacobians',...
                'pMeasurementToNoiseJacobians'...
        };
            for i = 1:numel(propList)
                if coder.internal.is_defined(obj.(propList{i}))
                    obj2.(propList{i}) = obj.(propList{i});
                end
            end
        end
        
        % keep labeled density
        function labeledDensity(obj,labelID)
            % LABELEDDENSITY Keep density with given labelID
            % LABELEDDENSITY(obj,L) removes all components from the filter
            % except components with Label set to L.
            validateattributes(labelID,{'numeric'},{'integer'},'labeledDensity','LabelID',2);
            labels = obj.Labels;
            weights = obj.Weights;
            toKeep = labels == labelID;
            [x,P,alpha,beta,nu,V] = getStates(obj);
            gff = obj.GammaForgettingFactors;
            x = x(:,toKeep);
            P = P(:,:,toKeep);
            alpha = alpha(1,toKeep);
            beta = beta(1,toKeep);
            nu = nu(1,toKeep);
            V = V(:,:,toKeep);
            weights = weights(1,toKeep);
            gff = gff(1,toKeep);
            n = sum(toKeep);
            obj.NumComponents = n;
            setStates(obj,x,P,alpha,beta,nu,V);
            obj.pLabels(1,1:n) = labelID;
            obj.pLabels(1,n+1:end) = obj.scalarZero;
            obj.pWeights(1,1:n) = weights;
            obj.pWeights(1,n+1:end) = obj.scalarZero;
            obj.pGammaForgettingFactors(1,1:n) = gff;
            obj.pGammaForgettingFactors(1,n+1:end) = obj.scalarZero;
        end
        
        % nullify density
        function nullify(obj)
            % nullifies the ggiwphd filter
            obj.pStates(:) = 0;
            obj.pStateCovariances(:) = 0;
            obj.pWeights(:) = 0;
            obj.pLabels(:) = 0;
            obj.pGammaForgettingFactors(:) = 0;
            obj.pScaleMatrices(:) = 0;
            obj.pShapes(:) = 0;
            obj.pRates(:) = 0;
            obj.NumComponents = 0;
        end
        
        % Extract state
        function [structStates, indices] = extractState(obj,threshold)
            % EXTRACTSTATE Extract target state estimates from the filter
            % [structStates,indices] = EXTRACTSTATE(obj,threshold)
            % extracts all states of components from the filter whose
            % weights are above the input, threshold.
            %
            % structStates is a n-element structure with the following
            % fields:
            % State: State estimate of the target
            % StateCovariances: Error in state estimate
            % MeasurementRate: Estimated number of detections from the target
            % Extent: Estimated Extent matrix of the target.
            %
            % indices is a n-element vector providing indices of the
            % components whose states are extracted.
            validateattributes(threshold,{'single','double'},...
                {'real','finite','scalar','nonsparse'},'extractState','Threshold',2);
            n = obj.NumComponents;
            assert(n <= obj.MaxNumComponents,message('fusion:ggiwphd:exceedsMaxNumComponents'));
            weights = obj.Weights;
            toOutput = weights > threshold;% | labels ~=0;
            indices = find(toOutput);
            [x,P,alpha,beta,nu,V] = getStates(obj);
            states = x(:,toOutput);
            covariances = P(:,:,toOutput);
            scales = V(:,:,toOutput);
            d = size(V,1);
            nStates = sum(toOutput);
            vLin = reshape(nu(1,toOutput)- d - 1,[1 1 nStates]);
            extent = bsxfun(@rdivide, scales, vLin);
            measurementRate = alpha(1,toOutput)./beta(1,toOutput);
            emptyStruct = struct('State',zeros(obj.pM,1),'StateCovariance',zeros(obj.pM, obj.pM),'Extent',zeros(d,d,class(states)),'MeasurementRate',cast(0,class(states)));
            assert(nStates <= obj.MaxNumComponents,message('fusion:ggiwphd:exceedsMaxNumComponents'));
            structStates = repmat(emptyStruct,[nStates 1]);
            for i = 1:nStates
                structStates(i).State = states(:,i);
                structStates(i).StateCovariance = covariances(:,:,i);
                structStates(i).Extent = extent(:,:,i);
                structStates(i).MeasurementRate = measurementRate(i);
            end
        end
        
        % sync density
        function sync(obj,obj2)
            % SYNC Synchronize obj with obj2
            % SYNC(obj,obj2) sets distribution properties of obj to match
            % those of obj2.
            % sync helps to ensure two filters produce same results on
            % calling methods like predict, correct, likelihood. All
            % non-tunable properties, function_handles are assumed same
            % between the two objects.
            n = obj2.NumComponents;
            obj.pStates(:,1:n) = obj2.States;
            obj.pStateCovariances(:,:,1:n) = obj2.StateCovariances;
            obj.pGammaForgettingFactors(1,1:n) = obj2.GammaForgettingFactors;
            obj.pRates(1,1:n) = obj2.Rates;
            obj.pShapes(1,1:n) = obj2.Shapes;
            obj.pDegreesOfFreedom(1,1:n) = obj2.DegreesOfFreedom;
            obj.pScaleMatrices(:,:,1:n) = obj2.ScaleMatrices;
            obj.pWeights(1,1:n) = obj2.Weights;
            obj.pLabels(1,1:n) = obj2.Labels;
            obj.NumComponents = n;
        end
        
        % scale
        function scale(obj,a)
            % SCALE Scale the density
            % SCALE(obj,a) Scales the density by a.
            % a is a scalar or n-element vector representing the scaling
            % factor for each component. The weight of each component is
            % multiplied by a.
            n = obj.NumComponents;
            validateattributes(a,{'single','double'},{'real','finite','nonsparse','positive'},'scale','Scales',2);
            coder.internal.assert(numel(a) == n || isscalar(a),'fusion:ggiwphd:scalarProp','Scales',n);
            weights = obj.Weights;
            weights = a(:).*weights(:);
            obj.pWeights(1,1:n) = weights;
        end
        
    end
    
    methods
        % Setters, Getters
        function set.Detections(obj,detections)
            validateattributes(detections,{'cell'},{'nonempty','vector'},'likelihood','detections');
            validateattributes(detections{1},{'objectDetection'},{'scalar'},'likelihood','detections{:}');
            testDetectionReadiness(obj);
            m = numel(detections);
            n = obj.NumComponents;
            assertMemoryIsAllocatedForBuffer(obj,detections);
            if coder.target('MATLAB')
                [obj.pDetectionBuffer{1:m}] = deal(detections{:});
            else
                for i = 1:m
                    obj.pDetectionBuffer{i} = detections{i};
                end
            end
            obj.pNumDetections = m;
            measParams = detections{1}.MeasurementParameters;
            x = obj.States;
            [zExp,H,U] = getExpectedMeasAndJacobians(obj,x,measParams);
            assertMemoryIsAllocatedForJacobians(obj);
            obj.pExpectedMeasurements(:,1:n) = zExp;
            obj.pMeasurementJacobians(:,:,1:n) = H;
            obj.pMeasurementToNoiseJacobians(:,:,1:n) = U;
        end
        
        function set.MaxNumDetections(obj,val)
            validateattributes(val,{'single','double'},{'scalar','integer','positive'},'ggiwphd','MaxNumDetections');
            obj.pMaxNumDetections = cast(val,obj.pDataType);
        end
        
        function val = get.MaxNumDetections(obj)
            val = obj.pMaxNumDetections;
        end
        
        function val = get.Detections(obj)
            if coder.internal.is_defined(obj.pDetectionBuffer)
                val = {obj.pDetectionBuffer{1:obj.pNumDetections}};
            else
                val = {};
            end
        end
        
        function set.States(obj,val)
            numComps = getAllowedSize(obj);
            d = size(obj.pStates,1);
            validateattributes(val,{'single','double'},...
                {'real','finite','nonsparse','2d','size',[d numComps]},'ggiwphd','States');
            obj.pStates(:,1:numComps) = val(:,1:numComps);
        end
        
        function val = get.States(obj)
            val = obj.pStates(:,1:obj.NumComponents);
        end
        
        function set.StateCovariances(obj,val)
            numComps = getAllowedSize(obj);
            d = size(obj.pStateCovariances,1);
            validateattributes(val,{'single','double'},...
                {'real','finite','nonsparse','3d','size',[d d numComps]},'ggiwphd','StateCovariances');
            % Verify positive definite value
            obj.pStateCovariances(:,:,1:numComps) = val(:,:,1:numComps);
        end
        
        function val = get.StateCovariances(obj)
            val = obj.pStateCovariances(:,:,1:obj.NumComponents);
        end
        
        function set.PositionIndex(obj,val)
            validatePosIndex(obj,val);
            obj.PositionIndex = val;
        end
        
        function set.Shapes(obj,val)
            numComps = validateScalarProperty(obj,val,'Shapes');
            obj.pShapes(1,1:numComps) = val(1:numComps);
        end
        
        function val = get.Shapes(obj)
            val = obj.pShapes(1,1:obj.NumComponents);
        end
        
        function set.Rates(obj,val)
            numComps = validateScalarProperty(obj,val,'Rates');
            obj.pRates(1,1:numComps) = val(1:numComps);
        end
        
        function val = get.Rates(obj)
            val = obj.pRates(1,1:obj.NumComponents);
        end
        
        function set.DegreesOfFreedom(obj,val)
            numComps = validateScalarProperty(obj,val,'DegreesOfFreedom');
            obj.pDegreesOfFreedom(1,1:numComps) = val(1:numComps);
        end
        
        function val = get.DegreesOfFreedom(obj)
            val = obj.pDegreesOfFreedom(1,1:obj.NumComponents);
        end
        
        function set.GammaForgettingFactors(obj,val)
            numComps = validateScalarProperty(obj,val,'GammaForgettingFactors');
            obj.pGammaForgettingFactors(1,1:numComps) = val(1:numComps);
        end
        
        function val = get.GammaForgettingFactors(obj)
            val = obj.pGammaForgettingFactors(1,1:obj.NumComponents);
        end
        
        function set.ScaleMatrices(obj,val)
            numComps = getAllowedSize(obj);
            d = size(obj.pScaleMatrices,1);
            validateattributes(val,{'single','double'},{'real','finite','nonsparse','3d','size',[d d numComps]},'ggiwphd','ScaleMatrices');
            obj.pScaleMatrices(:,:,1:numComps) = val(:,:,1:numComps);
        end
        
        function val = get.ScaleMatrices(obj)
            val = obj.pScaleMatrices(:,:,1:obj.NumComponents);
        end
        function set.Weights(obj,val)
            numComps = validateScalarProperty(obj,val,'Weights');
            obj.pWeights(1,1:numComps) = val(1,1:numComps);
        end
        
        function val = get.Weights(obj)
            val = obj.pWeights(1,1:obj.NumComponents);
        end
        
        function set.TemporalDecay(obj,val)
            validateattributes(val,...
                {'single','double'},{'real','finite','scalar','nonsparse'},'','TemporalDecay');
            obj.pTemporalDecay = cast(val,obj.pDataType);
        end
        
        function val = get.TemporalDecay(obj)
            val = obj.pTemporalDecay;           
        end
        
        function set.Labels(obj,val)
            numComps = validateScalarProperty(obj,val,'Labels',{'numeric'});
            obj.pLabels(1,1:numComps) = val(1,1:numComps);
        end
        
        function val = get.Labels(obj)
            val = obj.pLabels(1,1:obj.NumComponents);
        end
        
        function set.StateTransitionFcn(obj,val)
            validateattributes(val,{'function_handle'},{'scalar'},'ggiwphd','StateTransitionFcn');
            obj.StateTransitionFcn = val;
        end
        
        function set.StateTransitionJacobianFcn(obj,val)
            validateattributes(val,{'function_handle'},{'scalar'},'ggiwphd','StateTransitionJacobianFcn');
            obj.StateTransitionJacobianFcn = val;
        end
        
        function set.MeasurementFcn(obj,val)
            validateattributes(val,{'function_handle'},{'scalar'},'ggiwphd','MeasurementFcn');
            obj.MeasurementFcn = val;
        end
        
        function set.MeasurementJacobianFcn(obj,val)
            validateattributes(val,{'function_handle'},{'scalar'},'ggiwphd','MeasurementJacobianFcn');
            obj.MeasurementJacobianFcn = val;
        end
        
        function set.ExtentRotationFcn(obj,val)
            validateattributes(val,{'function_handle'},{'scalar'},'ggiwphd','ExtentRotationFcn');
            obj.ExtentRotationFcn = val;
        end
        
        function set.NumComponents(obj,val)
            coder.internal.assert(val <= obj.MaxNumComponents,'fusion:ggiwphd:exceedsMaxNumComponents'); %#ok<MCSUP>
            obj.NumComponents = val;
        end
        
        function val = get.NumComponents(obj)
            val = obj.NumComponents;
        end
        
        function set.ProcessNoise(obj,val)
            validateattributes(val,{'single','double'},{'real','finite','nonsparse'},'ggiwphd','ProcessNoise');
            noise = validateProcessNoiseSize(obj,val);
            obj.ProcessNoise = noise;
        end
        
        function set.HasAdditiveProcessNoise(obj,val)
            coder.internal.assert(~coder.internal.is_defined(obj.HasAdditiveProcessNoise),...
                'fusion:ggiwphd:nonTunableProp','HasAdditiveProcessNoise');
            validateattributes(val,{'logical'},{'scalar'},'','HasAdditiveProcessNoise');
            obj.HasAdditiveProcessNoise = val;
            setAdditiveNoiseSize(obj);
        end
        
        function set.HasAdditiveMeasurementNoise(obj,val)
           obj.HasAdditiveMeasurementNoise = val;
        end
    end
    
    methods (Hidden)
        function [states,stateCovs] = sigmaPoints(phd)
            states = phd.States;
            stateCovs = phd.StateCovariances;
            scales = phd.ScaleMatrices;
            nStates = size(states,2);
            d = size(scales,1);
            nu = phd.DegreesOfFreedom;
            vLin = reshape(nu - d - 1,[1 1 nStates]);
            extent = bsxfun(@rdivide, scales, vLin);
            stateCovs(phd.PositionIndex,phd.PositionIndex,:) =  stateCovs(phd.PositionIndex,phd.PositionIndex,:) + extent;
        end
        
        function stm = models(phd)
           if phd.HasAdditiveProcessNoise
               stm = @(x,dt)phd.StateTransitionFcn(x,dt);
           else
               v = zeros(1,1,'like',phd.States);
               stm = @(x,dt) phd.StateTransitionFcn(x,v,dt);
           end
        end
        
        function structState = sampleStruct(obj)
            % structState: Format,size,datatype for a struct which gets
            % added onto the track.
            d = size(obj.pScaleMatrices,1);
            structState = struct('State',zeros(obj.pM,1,obj.pDataType),'StateCovariance',...
                zeros(obj.pM, obj.pM),'Extent',zeros(d,d,obj.pDataType),...
                'MeasurementRate',cast(0,obj.pDataType));
        end
        
        function PzeroDets = probZeroDetections(obj, sensorConfig)
            % If using a point-target sensor, PzeroDets must be zero.
            if nargin > 1 && sensorConfig.MaxNumDetsPerObject == 1
                PzeroDets = zeros(obj.NumComponents,1,obj.pDataType);
            else
                % GGIW-PHD filter approximates the measurement rate using a
                % Gamma distribution.
                alpha = obj.Shapes;
                beta = obj.Rates;
                PzeroDets = max(0,(beta./(beta + 1)).^(alpha));
            end
        end
    end
    
    methods (Access = protected)
        
        function val =  getMaxNumDetections(obj)
            val = cast(obj.MaxNumDetections,obj.pDataType);
        end
        
        function validatePosIndex(obj,val)
            d = size(obj.pScaleMatrices,1);
            p = size(obj.pStates,1);
            validateattributes(val,{'single','double'},...
                {'real','finite','nonsparse','nonempty','vector','increasing','numel',d,'>=',1,'<=',p},'ggiwphd','PositionIndex');
        end
        
        function assertMemoryIsAllocatedForBuffer(obj,detections)
            coder.internal.assert(numel(detections) <= obj.MaxNumDetections,'fusion:ggiwphd:exceedsMaxNumDetections');
            if ~coder.internal.is_defined(obj.pDetectionBuffer)
                sampleDetection = {detections{1}};
                if ~coder.internal.is_defined(obj.pN)
                    obj.pN = numel(sampleDetection{1}.Measurement);
                    obj.pW = size(sampleDetection{1}.MeasurementNoise,1);
                    if coder.internal.is_defined(obj.HasAdditiveMeasurementNoise) && obj.HasAdditiveMeasurementNoise
                        coder.internal.assert(obj.pN == obj.pW,'fusion:ggiwphd:MustMatchAdditive');
                    elseif ~coder.internal.is_defined(obj.HasAdditiveMeasurementNoise)
                        obj.HasAdditiveMeasurementNoise = obj.pN == obj.pW;
                    end
                end
                obj.pDetectionBuffer = repmat(sampleDetection,[obj.MaxNumDetections 1]);
            end
        end
        
        function assertMemoryIsAllocatedForJacobians(obj)
            M = obj.MaxNumComponents;
            if ~coder.internal.is_defined(obj.pMeasurementJacobians)
                sampleMeasurement = repmat(zeros(obj.pN,1),[1 M]);
                sampleJacobian = repmat(zeros(obj.pN,obj.pM,1),[1 1 M]);
                sampleMeasToNoiseJacobian = repmat(zeros(obj.pW, obj.pW,1),[1 1 M]);
                obj.pExpectedMeasurements = sampleMeasurement;
                obj.pMeasurementJacobians = sampleJacobian;
                obj.pMeasurementToNoiseJacobians = sampleMeasToNoiseJacobian;
            end
        end
        
        function setAdditiveNoiseSize(obj)
            if obj.HasAdditiveProcessNoise
                obj.pV = obj.pM;
                obj.ProcessNoise = eye(obj.pV,obj.pDataType);
            end
        end
        
        function testDetectionReadiness(obj)
            % MeasurementFcn must be defined.
            if ~obj.pIsDetectReady
                coder.internal.assert(coder.internal.is_defined(obj.MeasurementFcn),...
                    'fusion:ggiwphd:undefinedProperty','MeasurementFcn','ggiwphd');
                coder.internal.assert(coder.internal.is_defined(obj.PositionIndex),...
                    'fusion:ggiwphd:undefinedProperty','PositionIndex','ggiwphd');
                obj.pIsDetectReady = true;
            end
        end
        
        function testPredictionReadiness(obj)
            if ~obj.pIsPredictReady
                testGaussianPredictionReadiness(obj);
                testIWPredictionReadiness(obj)
                obj.pIsPredictReady = true;
            end
        end
        
        function testGaussianPredictionReadiness(obj)
            % StateTransitionFcn must be defined
            coder.internal.assert(coder.internal.is_defined(obj.StateTransitionFcn),...
                'fusion:ggiwphd:undefinedProperty','StateTransitionFcn','predict');
            
            % ProcessNoise must be defined
            if obj.HasAdditiveProcessNoise && ~coder.internal.is_defined(obj.ProcessNoise)
                obj.pV = obj.pM;
                obj.ProcessNoise = cast(1,obj.pDataType);
            else
                coder.internal.assert(coder.internal.is_defined(obj.ProcessNoise),...
                    'fusion:ggiwphd:undefinedProperty','ProcessNoise','predict');
            end
        end
        
        function testIWPredictionReadiness(obj)
            % ExtentRotationFcn must be defined.
            coder.internal.assert(coder.internal.is_defined(obj.ExtentRotationFcn),...
                'fusion:ggiwphd:undefinedProperty','ExtentRotationFcn','predict');
        end
        
        function noise = validateProcessNoiseSize(obj,val)
            if isscalar(val) && coder.internal.is_defined(obj.pV)
                noise = val*eye(obj.pV,obj.pDataType);
            elseif ~isscalar(val) && coder.internal.is_defined(obj.pV)
                validateattributes(val,{'single','double'},{'size',[obj.pV obj.pV]},'ggiwphd','ProcessNoise');
                noise = val;
            else
                obj.pV = size(val,1);
                noise = val;
            end
        end
        
        function [numComps,maxNumComps] = getAllowedSize(obj)
            numComps = obj.NumComponents;
            if nargout == 2
                maxNumComps = obj.MaxNumComponents;
            end
        end
        
        function numComps = validateScalarProperty(obj,val,propName,classAllowed)
            numComps = getAllowedSize(obj);
            if nargin == 3
                classAllowed = {'single','double'};
            end
            validateattributes(val,classAllowed,{'real','finite','nonsparse','vector','numel',numComps},'ggiwphd',propName);
        end
        
        function testCorrectionReadiness(obj,methodName)
            coder.internal.assert(coder.internal.is_defined(obj.MeasurementFcn),...
                'fusion:ggiwphd:undefinedProperty','MeasurementFcn',methodName)
            coder.internal.assert(coder.internal.is_defined(obj.pDetectionBuffer),...
                'fusion:ggiwphd:undefinedProperty','Detections',methodName);
        end
        
        function allocateMemory(obj,states,covariances,varargin)
            validateattributes(states,{'single','double'},...
                {'real','finite','2d','nonsparse'},...
                'ggiwphd','States',1);
            
            % Set size and data type from states
            stateSize = size(states,1);
            obj.pM = stateSize;
            numComponents = size(states,2);
            dataType = class(states);
            obj.pDataType = dataType;
            
            % Validate covariance against state
            validateattributes(covariances,{'single','double'},...
                {'real','finite','3d','nonsparse',...
                'size',[stateSize stateSize numComponents]},...
                'ggiwphd','StateCovariances',2);
            
            % Check for MaxNumComponents property in N/V pairs.
            maxCompIndex = fusion.internal.findProp('MaxNumComponents',varargin{:});
            if maxCompIndex <= nargin - 3
                maxNumComps = varargin{maxCompIndex + 1};
            else
                maxNumComps = cast(max(numComponents,1000),dataType);
            end
            obj.MaxNumComponents = maxNumComps;
            m = obj.MaxNumComponents;            
            obj.NumComponents = numComponents;
            
            % Assert and information codegen for static memory allocation.
            assert(obj.NumComponents <= obj.MaxNumComponents);
            
            % Allocate memory for state and state covariances.
            obj.pStates = zeros(stateSize,m,dataType);
            obj.pStateCovariances = zeros(stateSize,stateSize,m,dataType);
            
            % Check if scale matrices were provided. This helps to define
            % memory for the IW distribution. Default is 3-Dimensional.
            scaleMatrixIndex = fusion.internal.findProp('ScaleMatrices',varargin{:});
            posIDIndex = fusion.internal.findProp('PositionIndex',varargin{:});
            
            % Use scale matrix first. If PositionIndex is invalid, it will
            % error out when setting it.
            if scaleMatrixIndex <= nargin - 3
                V = varargin{scaleMatrixIndex + 1};
                validateattributes(V,{'single','double'},...
                    {'real','finite','nonsparse','3d'},...
                    'ggiwphd','ScaleMatrices');
                dimSize = size(V,1);
            elseif posIDIndex <= nargin - 3
                posID = varargin{posIDIndex + 1};
                validateattributes(posID,{'single','double'},...
                    {'real','finite','nonsparse','nonempty','increasing','vector','>=',1,'<=',stateSize},...
                    'ggiwphd','PositionIndex');
                dimSize = numel(posID);
            else
                dimSize = cast(3,obj.pDataType);
            end
            
            % Allocate memory for scale matrices.
            obj.pScaleMatrices = repmat(100*eye(dimSize,dataType),[1 1 m]);
            
            % These properties are either scalar or vectors, easy to allocate
            % memory for them.
            obj.pDegreesOfFreedom = 100*ones(1,m,dataType);
            obj.pShapes = ones(1,m,dataType);
            obj.pRates = ones(1,m,dataType);
            obj.pWeights = ones(1,m,dataType);
            obj.pLabels = zeros(1,m,'uint32');
            obj.pGammaForgettingFactors = ones(1,m,dataType);
            % maxNumDetsIndex
            maxNumDetsIndex = fusion.internal.findProp('MaxNumDetections',varargin{:});
            if maxNumDetsIndex <= nargin - 3
                maxDets = varargin{maxNumDetsIndex + 1};
                validateattributes(maxDets,{'single','double'},{'integer','positive'},'ggiwphd','MaxNumDetections');
            else
                maxDets = 1000;
            end
            obj.pMaxNumDetections = cast(maxDets,obj.pDataType);
            obj.pNumDetections = obj.scalarZero;
            obj.pTemporalDecay = cast(100,obj.pDataType);
        end
        
        function propGroups = getPropertyGroups(~)
            gauss = {'States','StateCovariances','PositionIndex','StateTransitionFcn',...
                'StateTransitionJacobianFcn','ProcessNoise','HasAdditiveProcessNoise'};
            propGroups = matlab.mixin.util.PropertyGroup(gauss);
            
            gamma = {'Shapes','Rates','GammaForgettingFactors'};
            propGroups(2) = matlab.mixin.util.PropertyGroup(gamma);
            
            iw = {'DegreesOfFreedom','ScaleMatrices','ExtentRotationFcn','TemporalDecay'};
            propGroups(3) = matlab.mixin.util.PropertyGroup(iw);
            
            other = {'Weights','Labels'};
            propGroups(4) = matlab.mixin.util.PropertyGroup(other);
            
            models = {'Detections','MeasurementFcn','MeasurementJacobianFcn',...
                'HasAdditiveMeasurementNoise'};
            propGroups(5) = matlab.mixin.util.PropertyGroup(models);
        end
        
        function setStates(obj,x,P,alpha,beta,nu,V,inputID)
            if nargin < 8
                n = obj.NumComponents;
                indices = false(obj.MaxNumComponents,1);
                indices(1:n) = true;
            else
                if ~islogical(inputID)
                    indices = false(obj.MaxNumComponents,1);
                    indices(inputID) = true;
                end
            end
            n = sum(indices);
            obj.pStates(:,indices) = x(:,1:n);
            obj.pStateCovariances(:,:,indices) = P(:,:,1:n);
            obj.pShapes(1,indices) = alpha(1,1:n);
            obj.pRates(1,indices) = beta(1,1:n);
            obj.pDegreesOfFreedom(1,indices) = nu(1,1:n);
            obj.pScaleMatrices(:,:,indices)= V(:,:,1:n);
        end
        
        function [x,P,alpha,beta,nu,V] = getStates(obj)
            n = obj.NumComponents;
            M = obj.MaxNumComponents;
            assert(n <= M,message('fusion:ggiwphd:exceedsMaxNumComponents'));
            indices = 1:n;
            x = obj.pStates(:,indices);
            P = obj.pStateCovariances(:,:,indices);
            alpha = obj.pShapes(1,indices);
            beta = obj.pRates(1,indices);
            nu = obj.pDegreesOfFreedom(1,indices);
            V = obj.pScaleMatrices(:,:,indices);
        end
        
        function [zExp,H,U] = getExpectedMeasAndJacobians(obj,states,p)
            h = obj.MeasurementFcn;
            dh = obj.MeasurementJacobianFcn;
            n = obj.NumComponents;
            M = obj.MaxNumComponents;
            hasAMN = obj.HasAdditiveMeasurementNoise;
            assert(n <= M,message('fusion:ggiwphd:exceedsMaxNumComponents'));
            vZeros = zeros(obj.pW,n,class(states));
            H = zeros(obj.pN,obj.pM,n,obj.pDataType);
            U = zeros(obj.pW,obj.pN,n,obj.pDataType);
            zExp = zeros(obj.pN, n, obj.pDataType);
            
            % Don't need to calculate anything. Some measurement models may
            % not be able to accept an empty state.
            if n == 0
                return;
            end
            
            if iscell(p)
                if hasAMN
                    zExp = h(states,p{:});
                    for i = 1:n
                        x = states(:,i);
                        if isempty(dh)
                            Hi = matlabshared.tracking.internal.numericJacobian(h, {x, p{:}}, 1);
                        else
                            Hi = dh(x,p{:});
                        end
                        Ui = 0;
                        H(:,:,i) = Hi;
                        U(:,:,i) = Ui;
                    end
                else
                    zExp = h(states,vZeros,p{:});
                    for i = 1:n
                        x = states(:,i);
                        if isempty(dh)
                            Hi = matlabshared.tracking.internal.numericJacobian(h, {x, vZeros(:,i), p{:}}, 1);
                            Ui = matlabshared.tracking.internal.numericJacobian(h, {x, vZeros(:,i), p{:}}, 2);
                        else
                            [Hi,Ui] = dh(x,vZeros(:,i),p{:});
                        end
                        H(:,:,i) = Hi;
                        U(:,:,i) = Ui;
                    end
                end
            else
                if hasAMN
                    zExp = h(states,p);
                    for i = 1:n
                        x = states(:,i);
                        if isempty(dh)
                            Hi = matlabshared.tracking.internal.numericJacobian(h, {x, p}, 1);
                        else
                            Hi = dh(x,p);
                        end
                        Ui = 0;
                        H(:,:,i) = Hi;
                        U(:,:,i) = Ui;
                    end
                else
                    zExp = h(states,vZeros,p);
                    for i = 1:n
                        x = states(:,i);
                        if isempty(dh)
                            Hi = matlabshared.tracking.internal.numericJacobian(h, {x, vZeros(:,i), p}, 1);
                            Ui = matlabshared.tracking.internal.numericJacobian(h, {x, vZeros(:,i), p}, 2);
                        else
                            [Hi,Ui] = dh(x,vZeros(:,i),p);
                        end
                        H(:,:,i) = Hi;
                        U(:,:,i) = Ui;
                    end
                end
            end
        end
            
            function [z,R] = getCurrentMeasurementAndNoise(obj)
                m = obj.pNumDetections;
                detBuffer = obj.pDetectionBuffer;
                % Collect all z and R
                if coder.target('MATLAB')
                    allDets = [detBuffer{1:m}];
                    z = horzcat(allDets.Measurement);
                    R = cat(3,allDets.MeasurementNoise);
                else
                    z = zeros(obj.pN,m);
                    R = zeros(obj.pN,obj.pN,m);
                    for i = 1:m
                        z(:,i) = detBuffer{i}.Measurement(:);
                        R(:,:,i) = detBuffer{i}.MeasurementNoise;
                    end
                end
            end
            
            function val = scalarZero(obj)
                val = zeros(1,obj.pDataType);
            end
    end
        
        
    methods (Static, Hidden)
        function phd = initializeFromTrack(varargin) %#ok<STOUT>
            % A ggiwphd cannot be initiailized from a track because of its
            % Non Gaussian distributions (Inverse-Wishart and Gamma).
            % Throw an error when this method is called by the tracker.
            coder.internal.assert(false,'fusion:trackerPHD:needsFilterToInitialize','ggiwphd');
        end
    end
    
    methods (Access = {?ggiwphd, ?matlab.unittest.TestCase})
        function args = getDefaultArgs(~)
            args =  {'StateTransitionFcn',@constvel,'StateTransitionJacobianFcn',@constveljac,...
                'MeasurementFcn',@cvmeas,'MeasurementJacobianFcn',@cvmeasjac,'HasAdditiveProcessNoise',false,...
                'HasAdditiveMeasurementNoise',true,'ProcessNoise',eye(3),'ExtentRotationFcn',@(x,varargin)eye(3),'ScaleMatrices',repmat(100*eye(3),[1 1 2]),'DegreesOfFreedom',[100 100],'PositionIndex',[1 3 5]};
        end
    end
    
    methods(Static,Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % Let the coder know about non-tunable parameters so that it
            % can generate more efficient code.
            props = {'MaxNumComponents','HasAdditiveProcessNoise',...
                'pN','pM','pV','pW','HasAdditiveMeasurementNoise','pDataType'};
        end
    end
        
end
    
    
