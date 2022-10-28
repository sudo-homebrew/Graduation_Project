% gmphd Gaussian-mixture (GM) PHD filter
%   GMPHD implements the probability hypothesis density (PHD) using a
%   mixture of Gaussian components. The filter assumes that the target is
%   completely defined by a Gaussian state. You can use this filter to
%   track extended or point objects.
%
%   phd = gmphd returns a GM-PHD filter with default state transition
%   function (@constvel), measurement function (@cvmeas) and two components
%   in the density at states [0;0;0;0;0;0] and [1;1;1;1;1;1] with unit
%   state covariance in each dimension. 
%   
%   phd = gmphd(states, covariances) allows specifying the states and
%   covariances for the Gaussian distribution of each component. states is
%   a NxM matrix where each column represents the state of component. M is
%   the number of components in the density. covariances is a NxNxM matrix,
%   where each page or sheet represents the covariance of each component.
% 
%   phd = gmphd(states, covariances, 'Name', value) allows specifying
%   additional properties for the filter during construction using
%   name-value pairs.
% 
%   gmphd properties:
%   States                      - State of each component in the filter
%   StateCovariances            - Error covariance of each state
%   StateTransitionFcn          - A function to compute the state at next 
%                                 time step, (f)
%   StateTransitionJacobianFcn  - A function to compute State transition 
%                                 jacobian matrix, (df/dx)
%   ProcessNoise                - A matrix to represent the covariance of 
%                                 process noise (Q)
%   HasAdditiveProcessNoise     - A flag indicating if the process noise is 
%                                 of additive nature
%   HasExtent                   - A flag indicating if the components have
%                                 extent (generate more than one 
%                                 measurement per sensor)
%   MeasurementOrigin           - Origin of measurement from extended
%                                 objects
%   Weights                     - Current weight of each component
%   Labels                      - Current label of each component
%   Detections                  - Cell array of objectDetection objects at 
%                                 current time step
%   MeasurementFcn              - A function to compute the expected 
%                                 measurement from a state, (h)
%   MeasurementJacobianFcn      - A function to compute the jacobian of 
%                                 measurement function (dh/dx)
%   HasAdditiveMeasurementNoise - A flag indicating if the measurement 
%                                 noise is of additive nature
%   MaxNumDetections            - Maximum number of allowed detections
%   MaxNumComponents            - Maximum number of allowed components in 
%                                 the filter
%
%   gmphd methods:
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
%   Example: Running a gmphd filter for point objects
%   % Creating a filter with two 3-Dimensional constant velocity
%   % components, one at [0;0;0;0;0;0] and other at [1;0;1;0;1;0] and unit
%   % covariance in position and 100 covariance in velocity.
%   states = [zeros(6,1) [1;0;1;0;1;0]];
%   cov1 = diag([1 100 1 100 1 100]);
%   covariances = cat(3,cov1,cov1);
%   phd = gmphd(states, covariances, 'StateTransitionFcn', @constvel,...
%               'StateTransitionJacobianFcn',@constveljac,...
%               'MeasurementFcn',@cvmeas,...
%               'MeasurementJacobianFcn',@cvmeasjac,...
%               'ProcessNoise', eye(3),...
%               'HasAdditiveProcessNoise',false);
%   
%   % Predicting the filter dT = 0.1 time step ahead
%   predict(phd, 0.1);
%   
%   % Specifying a set of received detections
%   % Consider that the filter received 3 detections
%   detections = cell(3,1);
%   detections{1} = objectDetection(0,[1;1;1] + randn(3,1));
%   detections{2} = objectDetection(0,[0;0;0] + randn(3,1));
%   detections{3} = objectDetection(0,[4;5;3] + randn(3,1));
%   phd.Detections = detections;
%   
%   % Calculate the likelihood of detections.
%   % For a point-target filter, the partition of detections is simple and
%   % each detection must lie in a separate cell. Therefore the
%   % detectionIndices are simply an identity matrix with true values at the
%   % diagonal.
%   detectionIndices = logical(eye(3)); 
%   logLikelihood = likelihood(phd, detectionIndices)
%   
%   % Notice that the likelihood of detection 1 and 2 is higher as they are
%   % close to the components. However, likelihood of detection 3 is really
%   % low.
%
%   % correct the filter
%   % scale the likelihood
%   lhood = exp(logLikelihood);
%   lhood = lhood./sum(lhood,2);
%   correct(phd, detectionIndices, lhood);
%   
%   % merge the components
%   merge (phd, 1);
%   
%   % extract state estimates
%   minWeight = 0.5;
%   targetStates = extractState(phd, minWeight);
%
%   See also: objectDetection, constvel, cvmeas, constveljac, cvmeasjac,
%   constacc, cameas, constaccjac, cameasjac, constturn, ctmeas,
%   constturnjac, ctmeasjac, trackingSensorConfiguration, trackerPHD,
%   ggiwphd.

% Copyright 2019 The MathWorks, Inc.

%#codegen

classdef gmphd < fusion.internal.AbstractPHDFilter ...
        & matlabshared.tracking.internal.fusion.CustomDisplay
    
    % Gaussian properties
    properties (Dependent)
        % States State of each component. States is a M-by-N matrix, where
        % M is the dimension of the state and N is the number of
        % components. Each column of the States defines the state of each
        % component.
        States
        % StateCovariances Error covariance of each component's state
        % estimate.
        % StateCovariances is a M-by-M-by-N matrix, where each page or
        % sheet of the matrix defines the covariance of each component.
        StateCovariances
    end
    
    % Private copies
    properties (Access = protected)
        pStates
        pStateCovariances
    end
    
    % Allow static memory allocation
    properties (Access = protected)
        pMaxNumDetections
    end
    
    properties
        % HasExtent A flag to indicate if the components model extended
        % objects.
        % Set this value to true if the filter is intended to track
        % extended objects. Extended objects can generate more than one
        % measurement per sensor scan. Specifying HasExtent as true allows
        % specifying the MeasurementOrigin property.
        %
        %   Default: false
        HasExtent = false;
        
    end
    
    properties (Dependent)
        % MeasurementOrigin Origin of the measurement from extended
        % objects.
        % MeasurementOrigin allows you to specify the origination of
        % measurements from the target. It can be set to 'center' or
        % 'extent'. This property can be set when the property HasExtent is
        % set to true.
        %
        % When MeasurementOrigin is set to 'center':
        % The filter assumes that the measurements originate from the mean
        % state of the component. This approach is applicable when the
        % state does not model the extent of the target, but the target
        % generates more than 1 measurement per sensor scan.
        %
        % When MeasurementOrigin is set to 'extent': 
        % The filter assumes that expected measurements from the target are
        % not centered at the mean state. For computational efficiency, the
        % expected measurement is often calculated as a function of the
        % reported measurements. For example, expected measurement can be
        % computed by selecting the nearest point on extent to the reported
        % measurement.
        %
        % The function signatures of MeasurementFcn and
        % MeasurementJacobianFcn are different for both cases. Refer to the
        % MeasurementFcn and MeasurementJacobianFcn properties for more
        % information.
        %
        % Default: 'center'
        MeasurementOrigin
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
        %   The MeasurementFcn must support different signatures depending
        %   on the properties of the filter. 
        %   
        %   Case 1: HasExtent is false or HasExtent is true and
        %   MeasurementOrigin is "center"
        %   
        %     Specify the transition from state to measurement as a
        %     function. The function may take additional input parameters
        %     if needed, e.g., in order to specify the sensor position.
        %
        %     a. If HasAdditiveMeasurementNoise is true, the function
        %     should have the following signature:
        %       z = MeasurementFcn(x, parameters)
        %
        %     b. If HasAdditiveMeasurementNoise is false, the function
        %     should have the following signature:
        %       z = MeasurementFcn(x, v, parameters)
        %
        %       x is the (estimated) states at time k, where x(:,i)
        %       represents the ith state.
        %
        %       v is the measurement noise at time k, where v(:,i)
        %       represents the measurement noise for ith state.
        %
        %       parameters are MeasurementParameters provided in 
        %       the objectDetections set in Detections property.
        %
        %       z is the (estimated) measurements at time k, where z(:,i)
        %       represents the measurement from the ith state.
        %
        %   Case 2: HasExtent is true and MeasurementOrigin is "extent"
        %
        %     When the expected measurements originate from the extent of
        %     the target, they are often calculated as a function of the
        %     current set of measurements. In this case, the MeasurementFcn
        %     must support the following signatures:
        %
        %     a. If HasAdditiveMeasurementNoise is true, the function 
        %     should have the following signature:
        %       z = MeasurementFcn(x, detections)
        %
        %     b. If HasAdditiveMeasurementNoise is false, the function 
        %     should have the following signature:
        %       z = MeasurementFcn(x, v, detections)
        %
        %     where:
        %       x is the (estimated) states at time k, where x(:,i)
        %       represents the ith state.
        %          
        %       detections is a set of detections defined as a
        %       cell array of objectDetection objects.
        %       
        %       v is the measurement noise at time k, where v(:,i,j)
        %       represents the measurement noise for ith state and jth
        %       measurement.
        %
        %       z is the estimated measurements at time k, where z(:,i,j)
        %       represents the measurement from ith state, when considered
        %       with the jth detection.
        %
        MeasurementFcn
        
        % MeasurementJacobianFcn Jacobian of MeasurementFcn
        %   The MeasurementJacobianFcn must support different signatures
        %   depending on the properties of the filter.
        %   
        %   Case 1: HasExtent is false or HasExtent is true and
        %   MeasurementOrigin is "center"
        %
        %       Specify the function that calculates the Jacobian of the
        %       MeasurementFcn, h. This function must take the same input
        %       arguments as the MeasurementFcn. If not specified, the
        %       Jacobian will be numerically computed at every call to
        %       methods requiring Jacobian, which may increase processing
        %       time and numerical inaccuracy.
        %
        %       a. If HasAdditiveMeasurementNoise is true, the function 
        %       should have the following signature
        %           dhdx = MeasurementJacobianFcn(x, parameters)
        %
        %       b. If HasAdditiveMeasurementNoise is false, the function
        %       should have the following signature;
        %           [dhdx, dhdv] = MeasurementJacobianFcn(x, v, parameters)
        %       
        %       where:
        %       x is a column vector specifying one state at time k.
        %      
        %       v is a column vector specifying measurement noise at time k.
        %       
        %       parameters are MeasurementParameters provided in 
        %       the objectDetections set in Detections property
        %
        %       dhdx is Jacobian of MeasurementFcn with respect to
        %       states, dh/dx, evaluated at x. dhdx must be N-by-M
        %       matrix where N is the size of measurements,
        %       M is the size of the state.
        %       
        %       dhdv is Jacobian of MeasurementFcn with respect to
        %       measurement noise v, dh/dv, evaluated at x, v.
        %       dhdv must be N-by-V matrix where V is the number of
        %       measurement noise terms in v.
        %
        %   Case 2: HasExtent is true and MeasurementOrigin is "extent"
        %
        %       a. If HasAdditiveMeasurementNoise is true, the function
        %       should have the following signature:           
        %           dhdx = MeasurementJacobianFcn(x, detections)
        %
        %       b. If HasAdditiveMeasurementNoise is false, the function
        %       should have the following signature:
        %           [dhdx, dhdv] = MeasurementJacobianFcn(x, v, detections);
        %       
        %       where:
        %       x is a column vector specifying one state at time k.
        %   
        %       v is a column vector specifying measurement noise at time k.
        %       
        %       detections is a set of detections defined as a
        %       cell array of objectDetection objects.
        %       
        %       dhdx is Jacobian of MeasurementFcn with respect to
        %       state evaluated at x. dhdx must be a N-by-M-by-P matrix,
        %       where N is the size of measurements, M is the size of
        %       state and P are the number of detections. dhdx(:,:,j)
        %       represents the measurement Jacobian for the jth detection.
        %
        %       dhdv is the Jacobian of the MeasurementFcn with respect to
        %       measurement noise. dhdv must be a N-by-V-by-P matrix, where
        %       V is the number of measurement noise terms in V.
        %       dhdv(:,:,j) represents the measurement Jacobian for the jth
        %       detection.
        %       
        %       Note: In this case, the filter does not compute the
        %       Jacobian numerically and the function must be provided to 
        %       use the filter.
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
        %   process noise as a scalar or an M-by-M matrix, where M is the
        %   dimension of the state.
        %
        %   If HasAdditiveProcessNoise is false: specify the covariance of
        %   process noise as a W-by-W matrix, where W is the number of the
        %   process noise terms.
        %
        %   Default: eye(M) if HasAdditiveProcessNoise is true.
        ProcessNoise
    end
    
    properties (Dependent)
        % Detections Current set of detections
        % Detections is a cell array of objectDetection objects.
        % Use the property to set the current set of detections for the
        % filter.
        % Default: {}
        Detections
        
        % MaxNumDetections Maximum number of detections
        % A scalar value representing the maximum number of allowed
        % detections.
        % Default: 1000
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
    properties (Access = {?gmphd, ?matlab.unittest.TestCase})
        pDetectionBuffer
        pM
        pN
        pV
        pW
        pDataType
        pIsPredictReady = false;
        pIsDetectReady = false;
        pHasDetectionInput;
        pMeasurementOrigin
    end
    
    methods
        % Constructor
        function obj = gmphd(states,covariances,varargin)
            % obj = GMPHD
            % obj = GMPHD(states,covariances);
            % obj = GMPHD(states,covariances,'Name',value);
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
            obj.pFilter = fusion.internal.gaussEKFilter;
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
            % function changes the state of the object, obj, with the
            % predicted state.
            testPredictionReadiness(obj);
            validateattributes(dT,{'single','double'},{'real','finite','nonsparse','scalar'},'predict','dT',2);
            f = obj.StateTransitionFcn;
            df = obj.StateTransitionJacobianFcn;
            n = obj.NumComponents;
            if n > 0
                Q = obj.ProcessNoise;
                hasAPN = obj.HasAdditiveProcessNoise;
                [x,P] = getStates(obj);
                [x,P] = obj.pFilter.predict(x,P,Q,f,df,hasAPN,dT);
                setStates(obj,x,P);
            end
        end
        
        
        function lhood = likelihood(obj,detectionIndices,varargin)
            % LIKELIHOOD Calculate likelihood of current detections
            % lhood = LIKELIHOOD(obj,detectionIndices) returns the
            % log-likelihood for each column of detectionIndices.
            % detectionIndices is a logical matrix of size m-by-p, where m
            % is the number of total detections and p is the number of
            % detection cells.
            % lhood is a N-by-p matrix, where each column represents the
            % log-likelihood of N components in the obj against each cell
            % of detection.
            coder.internal.assert(obj.pIsDetectReady,'fusion:gmphd:detectNotReady','likelihood');
            validateattributes(detectionIndices,{'logical'},{'nrows',obj.pNumDetections},'likelihood','detectionIndices',2);
            coder.internal.assert(all(sum(detectionIndices,1) >= 1),'fusion:gmphd:mustSumToAtleastOne');
            numCells = size(detectionIndices,2);
            m = obj.pNumDetections;
            assert(m <= obj.pMaxNumDetections,message('fusion:gmphd:exceedsMaxNumDetections'));
            n = obj.NumComponents;
            [x,P] = getStates(obj);
            [zAll,RAll] = getCurrentMeasurementAndNoise(obj);
            
            % Get values for filter
            hasAMN = obj.HasAdditiveMeasurementNoise;
            noiseSize = obj.pW;
            measSize = obj.pN;
            detections = obj.Detections;
            h = obj.MeasurementFcn;
            dh = obj.MeasurementJacobianFcn;
            
            hasExtent = obj.HasExtent;
                       
            numDetsPerCell = sum(detectionIndices,1);
            
            % Measurement Likelihood
            measLhood = zeros(n,numCells,obj.pDataType);
            
            % Don't need to calculate anything. Some measurement models may
            % not be able to accept an empty state.
            if n == 0
                lhood = measLhood;
                return;
            end
            
            for cellIdx = 1:numCells
                thisCell = detectionIndices(:,cellIdx);
               
                detectionCell = getDetectionCell(detections, thisCell);
                
                % Non-extent filter with extended measurements yield 0
                % likelihood
                if numel(detectionCell) > 1 && ~hasExtent
                    measLhood(:,cellIdx) = -inf;
                    continue;
                end
                
                [z, R] = concatenateMeasurement(zAll,RAll,thisCell);
                
                if obj.pHasDetectionInput
                    zExp = obj.pFilter.getExpectedExtentMeasurements(x, detectionCell, h, hasAMN, noiseSize);
                else
                    zExp = obj.pFilter.getExpectedMeasurements(x, detectionCell, h, hasAMN, noiseSize);
                end
                
                for i = 1:n
                    xi = x(:,i);
                    Pi = P(:,:,i);
                    zExpi = zExp(:,i,:);
                    
                    if obj.pHasDetectionInput
                        % Hi and Ui are 2-D matrix concatenated for measurements.
                        [Hi,Ui] = obj.pFilter.getExtentMeasurementJacobians(xi, detectionCell, dh, hasAMN, noiseSize, measSize);
                    else
                        [Hi,Ui] = obj.pFilter.getMeasurementJacobians(xi, detectionCell, h, dh, hasAMN, noiseSize, measSize);
                    end
                    
                    % Residual
                    r = z - zExpi(:);
                    
                    % Residual covariance. U is identity when hasAMN is
                    % true.
                    S = Hi*Pi*Hi' + Ui*R*Ui';
                    
                    % log-likelihood
                    k = numel(r);
                    measLhood(i,cellIdx) = -k/2*log(2*pi) - 1/2*log(det(S)) - 1/2*r'/S*r;
                end
            end
            
            % Cardinality Likelihood
            if nargin > 2 
                sensorConfig = varargin{1};
                if sensorConfig.MaxNumDetsPerObject == 1
                    cardLhood = zeros(n,numCells,obj.pDataType);
                else
                    Nexp = expectedNumDets(sensorConfig, x, P);
                    
                    % Row vector
                    K = numDetsPerCell(:)';
                    logfactorialK = arrayfun(@(x)sum(log(1:x)),K);
                    % Column vector
                    lambda = max(eps,Nexp(:));
                    
                    % Poisson distribution
                    cardLhood = bsxfun(@minus, bsxfun(@minus,bsxfun(@times,K,log(lambda)), lambda), logfactorialK);
                end
            else
                cardLhood = zeros(n,numCells,obj.pDataType);
            end
            
            % Add log-likelihoods
            lhood = measLhood + cardLhood;
        end
        
        % correct method
        function correct(obj,detectionIndices,cellLikelihood,varargin)
            % CORRECT Correct the filter with subsets of obj.Detections
            % CORRECT(obj,detectionIndices,likelihood)
            % obj is a gmphd object.
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
            coder.internal.assert(obj.pIsDetectReady,'fusion:gmphd:detectNotReady','correct');
            validateattributes(detectionIndices,{'logical'},{'nrows',obj.pNumDetections},'correct','detectionIndices',2);
            numCells = size(detectionIndices,2);
            validateattributes(cellLikelihood,{'single','double'},{'real','finite','nonsparse','nrows',n,'ncols',numCells},'correct','detectionLikelihood',3);
            
            % Each cell must contain at least 1 detection
            coder.internal.assert(all(sum(detectionIndices,1) >= 1),'fusion:gmphd:mustSumToAtleastOne');
            
            m = obj.pNumDetections;
            assert(m <= obj.pMaxNumDetections,message('fusion:gmphd:exceedsMaxNumDetections'));
            
            M = obj.MaxNumComponents;
            assert(n <= M,message('fusion:gmphd:exceedsMaxNumComponents'));
            
            [zAll,RAll] = getCurrentMeasurementAndNoise(obj);
            detections = obj.Detections;
            
            currentWeights = obj.Weights;
            labels = obj.pLabels(1,1:n);
            
            [x,P] = getStates(obj);
            
            % Start with 0 components
            n = 0;
            measSize = obj.pN;
            noiseSize = obj.pW;
            stateSize = obj.pM;
            hasAMN = obj.HasAdditiveMeasurementNoise;
            
            % Extract functions
            h = obj.MeasurementFcn;
            dh = obj.MeasurementJacobianFcn;
            
            % Correct and add component for each cell
            for cellIdx = 1:numCells
                thisCell = detectionIndices(:,cellIdx);
                detectionCell = getDetectionCell(detections, thisCell);
                
                numMeas = sum(thisCell);
              
                [z,R] = concatenateMeasurement(zAll,RAll,thisCell);
                
                compLikelihood = cellLikelihood(:,cellIdx);
                
                % Only correct where likelihood is greater than eps
                permitCorrection = compLikelihood(:).*currentWeights(:) > eps;
                
                % Get states of components permitted correction.
                xCor = x(:,permitCorrection);
                PCor = P(:,:,permitCorrection);
                
                nNew = sum(permitCorrection);
                
                if nNew > 0
                    % Inform codegen that n < M
                    assert(n + nNew <= M,message('fusion:gmphd:exceedsMaxNumComponents'));
                    
                    % New indices
                    newIndices = n+(1:nNew);
                    
                    % Collect residual and residual covariances.
                    resTotal = zeros(numMeas*measSize,nNew,obj.pDataType);
                    resCovTotal = zeros(numMeas*measSize,numMeas*measSize,nNew,obj.pDataType);
                    HTotal = zeros(numMeas*measSize, stateSize, nNew, obj.pDataType);
                    
                    if obj.pHasDetectionInput
                        zExp = obj.pFilter.getExpectedExtentMeasurements(xCor, detectionCell, h, hasAMN, noiseSize);
                    else
                        zExp = obj.pFilter.getExpectedMeasurements(xCor, detectionCell, h, hasAMN, noiseSize);
                    end
                    
                    for i = 1:nNew
                        xi = xCor(:,i);
                        Pi = PCor(:,:,i);
                        zExpi = zExp(:,i,:);
                        resTotal(:,i) = z - zExpi(:);
                        if obj.pHasDetectionInput
                            % Hi and Ui are 2-D matrix concatenated for measurements.
                            [Hi,Ui] = obj.pFilter.getExtentMeasurementJacobians(xi, detectionCell, dh, hasAMN, noiseSize, measSize);
                        else
                            [Hi,Ui] = obj.pFilter.getMeasurementJacobians(xi, detectionCell, h, dh, hasAMN, noiseSize, measSize);
                        end
                        % Ui is identity for additive noise.
                        resCovTotal(:,:,i) = Hi*Pi*Hi' + Ui*R*Ui';
                        HTotal(:,:,i) = Hi;
                    end
                    
                    % Correct using Gauss filter
                    [xk,Pk] = obj.pFilter.correct(xCor,PCor,resTotal,resCovTotal,HTotal);

                    setStates(obj,xk,Pk,newIndices);
                    
                    % Set weights and labels
                    weights = currentWeights(permitCorrection);
                    likelihood = compLikelihood(permitCorrection);
                    obj.pWeights(newIndices) = weights(:).*likelihood(:);
                    obj.pLabels(newIndices) = labels(permitCorrection);
                    n = n + nNew;
                end
            end
            
            % Set number of components
            obj.NumComponents = n;
        end
        
        % correct undetected
        function correctUndetected(obj,Pd,PzeroDets)
            % CORRECTUNDETECTED correct the filter with the hypothesis that
            % no component in the density was detected.
            % CORRECTUNDETECTED(obj,Pd);
            % CORRECTUNDETECTED(obj,Pd,PzeroDets);
            % obj is a gmphd object
            % Pd is a vector of length n, where n is the current number of
            % components. Each element of Pd defines the probability of
            % detection of each component.
            % PzeroDets is an optional input vector of length n, where n is
            % the current number of components. Each element of PzeroDets
            % defines the conditional probability of generating no
            % detections by each component upon detection. If unspecified,
            % it is assumed to be zero.
            %
            n = obj.NumComponents;
            validateattributes(Pd,{'single','double'},{'real','finite','nonsparse','vector','numel',n,'>=',0,'<=',1},'correctUndetected','Pd',2);
            if nargin > 2
                validateattributes(PzeroDets,{'single','double'},{'real','finite','nonsparse','vector','numel',n,'>=',0,'<=',1},'correctUndetected','PzeroDets',2);
            else
                PzeroDets = zeros(n,1,obj.pDataType); 
            end
            if n > 0
                currentWeights = obj.Weights;
                w1 = 1 - Pd(:) + eps(Pd(1));
                w2 = Pd(:).*PzeroDets(:) + eps(Pd(1));
                qd = w1 + w2;
                updatedWeights = qd(:).*currentWeights(:);
                obj.pWeights(1,1:n) = updatedWeights;
                obj.pWeights(1,n+1:end) = obj.scalarZero;
            end
        end
        
        
        % merge method
        function merge(obj,threshold)
            % MERGE Merge components in the filter.
            % MERGE(obj,threshold) merges components in the filter whose
            % Kullback-Leibler difference is below the input, threshold.
            
            validateattributes(threshold,...
                {'single','double'},{'real','finite','scalar','nonsparse'},'merge','Threshold',2);
            n = obj.NumComponents;
            M = obj.MaxNumComponents;
            assert(n <= M,message('fusion:gmphd:exceedsMaxNumComponents'));
            if n > 0
                [x,P] = getStates(obj);
                w = obj.Weights;
                label = obj.Labels;
                gaussDistance = fusion.internal.gaussKLDiff(x,P);
                clusters = fusion.internal.clusterUsingDistance(gaussDistance,threshold);
                numComps = cast(max(clusters),obj.pDataType);
                
                % Allocated memory
                xMerge = x(:,1:numComps);
                PMerge = P(:,:,1:numComps);
                wMerge = w(1,1:numComps);
                labelMerge = label(1,1:numComps);
                
                for i = 1:numComps
                    thisCluster = clusters == i;
                    xC = x(:,thisCluster);
                    PC = P(:,:,thisCluster);
                    wC = w(:,thisCluster);
                    labelC = label(1,thisCluster);
                    [wM,xM,PM] = fusion.internal.ggiwMerge(wC,xC,PC);
                    xMerge(:,i) = xM;
                    PMerge(:,:,i) = PM;
                    wMerge(1,i) = wM;
                    labelMerge(1,i) = labelC(1);
                end
                
                obj.NumComponents = numComps;
                setStates(obj,xMerge,PMerge);
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
            
            [x,P] = getStates(obj);
            weights = obj.Weights;
            labels = obj.Labels;
            accept = ~pruneIndices;
            xPruned = x(:,accept);
            PPruned = P(:,:,accept);
            labels = labels(1,accept);
            numComponents = sum(accept);
            obj.NumComponents = numComponents;
            setStates(obj,xPruned,PPruned);
            obj.pLabels(1:numComponents) = labels;
            obj.pWeights(1:numComponents) = weights(1,accept);
        end
        
        % add two densities
        function append(obj,obj2)
            % APPEND Append two filters.
            % APPEND (obj, obj2) appends the components in obj2 to the
            % components in obj. The total number of components in obj must
            % not exceed MaxNumComponents.
            % obj2 is a gmphd object.
            n = obj.NumComponents;
            m = obj2.NumComponents;
            assert(n + m <= obj.MaxNumComponents,message('fusion:gmphd:exceedsMaxNumComponents'));
            newStates = obj2.States;
            newCovariances = obj2.StateCovariances;
            weights = obj2.Weights;
            labels = obj2.Labels;
            newIndices = n+(1:m);
            obj.pStates(:,newIndices) = newStates(:,1:m);
            obj.pStateCovariances(:,:,newIndices) = newCovariances(:,:,1:m);
            obj.pWeights(1,newIndices) = weights(1,1:m);
            obj.NumComponents = n + m;
            obj.pLabels(1,newIndices) = labels(1,1:m);
            obj.HasExtent = obj.HasExtent || obj2.HasExtent;
        end
        
        % clone density
        function obj2 = clone(obj)
            % CLONE clone the filter
            % obj2 = CLONE(obj) returns a clone of the object, obj.
            [x,P] = getStates(obj);
            % Construct with memory-related and non-tunable properties
            obj2 = gmphd(x,P,'MaxNumComponents',obj.MaxNumComponents,...
                'HasAdditiveProcessNoise',obj.HasAdditiveProcessNoise,...
                'ProcessNoise',obj.ProcessNoise);
            
            propList = {'StateTransitionFcn',...
                'MeasurementFcn',...
                'StateTransitionJacobianFcn',...
                'MeasurementJacobianFcn',...
                'pN','pM','pV','pW',...
                'pWeights',...
                'pLabels',...
                'ProcessNoise',...
                'pStates',...
                'pStateCovariances',...
                'pDetectionBuffer',...
                'pIsPredictReady',...
                'pIsDetectReady',...
                'pNumDetections',...
                'pMaxNumDetections',...
                'HasAdditiveMeasurementNoise',...
                'HasExtent',...
                'pMeasurementOrigin',...
                'pHasDetectionInput',...
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
            [x,P] = getStates(obj);
            x = x(:,toKeep);
            P = P(:,:,toKeep);
            weights = weights(1,toKeep);
            n = sum(toKeep);
            obj.NumComponents = n;
            setStates(obj,x,P);
            obj.pLabels(1,1:n) = labelID;
            obj.pLabels(1,n+1:end) = obj.scalarZero;
            obj.pWeights(1,1:n) = weights;
            obj.pWeights(1,n+1:end) = obj.scalarZero;
        end
        
        % nullify density
        function nullify(obj)
            % nullifies the gmphd filter
            obj.pStates(:) = 0;
            obj.pStateCovariances(:) = 0;
            obj.pWeights(:) = 0;
            obj.pLabels(:) = 0;
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
            
            % indices is a n-element vector providing indices of the
            % components whose states are extracted.
            validateattributes(threshold,{'single','double'},...
                {'real','finite','scalar','nonsparse'},'extractState','Threshold',2);
            n = obj.NumComponents;
            assert(n <= obj.MaxNumComponents,message('fusion:gmphd:exceedsMaxNumComponents'));
            weights = obj.Weights;
            toOutput = weights > threshold;% | labels ~=0;
            indices = find(toOutput);
            [x,P] = getStates(obj);
            states = x(:,toOutput);
            covariances = P(:,:,toOutput);
            nStates = sum(toOutput);
            emptyStruct = struct('State',zeros(obj.pM,1),'StateCovariance',zeros(obj.pM, obj.pM));
            assert(nStates <= obj.MaxNumComponents,message('fusion:gmphd:exceedsMaxNumComponents'));
            structStates = repmat(emptyStruct,[nStates 1]);
            for i = 1:nStates
                structStates(i).State = states(:,i);
                structStates(i).StateCovariance = covariances(:,:,i);
            end
        end
        
        % sync density
        function sync(obj,obj2)
            % SYNC synchronize obj with obj2
            % SYNC(obj,obj2) sets distribution properties of obj to match
            % those of obj2.
            % SYNC helps to ensure two filters produce same results on
            % calling methods like predict, correct, likelihood. All
            % non-tunable properties, function_handles are assumed same
            % between the two objects.
            n = obj2.NumComponents;
            obj.pStates(:,1:n) = obj2.States;
            obj.pStateCovariances(:,:,1:n) = obj2.StateCovariances;
            obj.pWeights(1,1:n) = obj2.Weights;
            obj.pLabels(1,1:n) = obj2.Labels;
            obj.NumComponents = n;
        end
        
        % scale
        function scale(obj,a)
            % scale Scale the density
            % SCALE(obj,a) Scale the density by a.
            % a is a scalar or n-element vector representing the scaling
            % factor for each component. The weight of each component is
            % multiplied by a.
            n = obj.NumComponents;
            validateattributes(a,{'single','double'},{'real','finite','nonsparse','positive'},'scale','Scales',2);
            coder.internal.assert(numel(a) == n || isscalar(a),'fusion:gmphd:scalarProp','Scales',n);
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
            assertMemoryIsAllocatedForBuffer(obj,detections);
            if coder.target('MATLAB')
                [obj.pDetectionBuffer{1:m}] = deal(detections{:});
            else
                for i = 1:m
                    obj.pDetectionBuffer{i} = detections{i};
                end
            end
            obj.pNumDetections = m;
        end
        
        function set.MeasurementOrigin(obj, val)
            coder.internal.assert(obj.HasExtent,'fusion:gmphd:originActiveWithExtent');
            val = validatestring(val, {'extent','center'},'gmphd','MeasurementOrigin');
            if ~coder.internal.is_defined(obj.pMeasurementOrigin)
                obj.pMeasurementOrigin = fusion.internal.MeasurementOriginValue(uint8(strcmpi(val,'extent')));
            else
                coder.internal.assert(false,'fusion:gmphd:nonTunableProp','MeasurementOrigin');
            end
        end
        
        function val = get.MeasurementOrigin(obj)
            val = char(obj.pMeasurementOrigin);
        end
            
        function set.MaxNumDetections(obj,val)
            validateattributes(val,{'single','double'},{'scalar','integer','positive'},'gmphd','MaxNumDetections');
            obj.pMaxNumDetections = cast(val,obj.pDataType);
        end
        
        function val = get.MaxNumDetections(obj)
            val = obj.pMaxNumDetections;
        end
        
        function val = get.Detections(obj)
            if coder.internal.is_defined(obj.pDetectionBuffer)
                if coder.target('MATLAB')
                    val = {obj.pDetectionBuffer{1:obj.pNumDetections}};
                else
                    val = cell(obj.pNumDetections,1);
                    for i = 1:numel(val)
                        val{i} = obj.pDetectionBuffer{i};
                    end
                end
            else
                val = {};
            end
        end
        
        function set.States(obj,val)
            numComps = getAllowedSize(obj);
            d = size(obj.pStates,1);
            validateattributes(val,{'single','double'},...
                {'real','finite','nonsparse','2d','size',[d numComps]},'gmphd','States');
            obj.pStates(:,1:numComps) = val(:,1:numComps);
        end
        
        function val = get.States(obj)
            val = obj.pStates(:,1:obj.NumComponents);
        end
        
        function set.StateCovariances(obj,val)
            numComps = getAllowedSize(obj);
            d = size(obj.pStateCovariances,1);
            validateattributes(val,{'single','double'},...
                {'real','finite','nonsparse','3d','size',[d d numComps]},'gmphd','StateCovariances');
            % Verify positive definite value
            obj.pStateCovariances(:,:,1:numComps) = val(:,:,1:numComps);
        end
        
        function val = get.StateCovariances(obj)
            val = obj.pStateCovariances(:,:,1:obj.NumComponents);
        end
        
        
        function set.Weights(obj,val)
            numComps = validateScalarProperty(obj,val,'Weights');
            obj.pWeights(1,1:numComps) = val(1,1:numComps);
        end
        
        function val = get.Weights(obj)
            val = obj.pWeights(1,1:obj.NumComponents);
        end
        
        function set.Labels(obj,val)
            numComps = validateScalarProperty(obj,val,'Labels',{'numeric'});
            obj.pLabels(1,1:numComps) = val(1,1:numComps);
        end
        
        function val = get.Labels(obj)
            val = obj.pLabels(1,1:obj.NumComponents);
        end
        
        function set.StateTransitionFcn(obj,val)
            validateattributes(val,{'function_handle'},{'scalar'},'gmphd','StateTransitionFcn');
            obj.StateTransitionFcn = val;
        end
        
        function set.StateTransitionJacobianFcn(obj,val)
            validateattributes(val,{'function_handle'},{'scalar'},'gmphd','StateTransitionJacobianFcn');
            obj.StateTransitionJacobianFcn = val;
        end
        
        function set.MeasurementFcn(obj,val)
            validateattributes(val,{'function_handle'},{'scalar'},'gmphd','MeasurementFcn');
            obj.MeasurementFcn = val;
        end
        
        function set.MeasurementJacobianFcn(obj,val)
            validateattributes(val,{'function_handle'},{'scalar'},'gmphd','MeasurementJacobianFcn');
            obj.MeasurementJacobianFcn = val;
        end
        
        function set.NumComponents(obj,val)
            coder.internal.assert(val <= obj.MaxNumComponents,'fusion:gmphd:exceedsMaxNumComponents'); %#ok<MCSUP>
            obj.NumComponents = val;
        end
        
        function val = get.NumComponents(obj)
            val = obj.NumComponents;
        end
        
        function set.ProcessNoise(obj,val)
            validateattributes(val,{'single','double'},{'real','finite','nonsparse'},'gmphd','ProcessNoise');
            noise = validateProcessNoiseSize(obj,val);
            obj.ProcessNoise = noise;
        end
        
        function set.HasAdditiveProcessNoise(obj,val)
            coder.internal.assert(~coder.internal.is_defined(obj.HasAdditiveProcessNoise),...
                'fusion:gmphd:nonTunableProp','HasAdditiveProcessNoise');
            validateattributes(val,{'numeric','logical'},{'scalar','binary'},'','HasAdditiveProcessNoise');
            obj.HasAdditiveProcessNoise = logical(val);
            setAdditiveNoiseSize(obj);
        end
        
        function set.HasAdditiveMeasurementNoise(obj,val)
            coder.internal.assert(~coder.internal.is_defined(obj.HasAdditiveMeasurementNoise),...
                'fusion:gmphd:nonTunableProp','HasAdditiveMeasurementNoise');
            validateattributes(val,{'numeric','logical'},{'scalar','binary'},'','HasAdditiveMeasurementNoise');
            obj.HasAdditiveMeasurementNoise = val;
        end
        
        function set.HasExtent(obj, val)
            validateattributes(val,{'numeric','logical'},{'scalar','binary'},'gmphd','HasExtent');
            obj.HasExtent = logical(val);
        end
    end
    
    % Methods for trackerPHD to call
    methods (Hidden)
        function [states,stateCovs] = sigmaPoints(phd)
            states = phd.States;
            stateCovs = phd.StateCovariances;
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
            structState = struct('State',zeros(obj.pM,1,obj.pDataType),...
                'StateCovariance',zeros(obj.pM, obj.pM, obj.pDataType));
        end
        
        function PzeroDets = probZeroDetections(obj, sensorConfig)
            % Probability of generating zero detections
            if sensorConfig.MaxNumDetsPerObject == 1
                PzeroDets = zeros(obj.NumComponents,1,obj.pDataType);
            else
                x = obj.States;
                P = obj.StateCovariances;
                Nexp = expectedNumDets(sensorConfig, x, P);
                PzeroDets = cast(exp(-Nexp(:)),obj.pDataType);
            end
        end
    end
    
    methods (Access = protected)
        
        function val =  getMaxNumDetections(obj)
            val = cast(obj.MaxNumDetections,obj.pDataType);
        end
        
        function assertMemoryIsAllocatedForBuffer(obj,detections)
            coder.internal.assert(numel(detections) <= obj.MaxNumDetections,'fusion:gmphd:exceedsMaxNumDetections');
            if ~coder.internal.is_defined(obj.pDetectionBuffer)
                sampleDetection = {detections{1}};
                if ~coder.internal.is_defined(obj.pN)
                    obj.pN = numel(sampleDetection{1}.Measurement);
                    obj.pW = size(sampleDetection{1}.MeasurementNoise,1);
                    if coder.internal.is_defined(obj.HasAdditiveMeasurementNoise) && obj.HasAdditiveMeasurementNoise
                        coder.internal.assert(obj.pN == obj.pW,'fusion:gmphd:MustMatchAdditive');
                    elseif ~coder.internal.is_defined(obj.HasAdditiveMeasurementNoise)
                        obj.HasAdditiveMeasurementNoise = obj.pN == obj.pW;
                    end
                end
                obj.pDetectionBuffer = repmat(sampleDetection,[obj.MaxNumDetections 1]);
            end
        end
        
        function setAdditiveNoiseSize(obj)
            if obj.HasAdditiveProcessNoise
                obj.pV = obj.pM;
                obj.ProcessNoise = eye(obj.pV,obj.pDataType);
            end
        end
        
        function testDetectionReadiness(obj)
            % If measurement origin is not defined, specify the default
            % "center".
            if ~coder.internal.is_defined(obj.pMeasurementOrigin)
                obj.pMeasurementOrigin = fusion.internal.MeasurementOriginValue.center;
            end
            if ~coder.internal.is_defined(obj.pHasDetectionInput)
                obj.pHasDetectionInput = obj.pMeasurementOrigin == 1;
            end
            % MeasurementFcn must be defined.
            if ~obj.pIsDetectReady
                coder.internal.assert(coder.internal.is_defined(obj.MeasurementFcn),...
                    'fusion:gmphd:undefinedProperty','MeasurementFcn','gmphd');
                if obj.pMeasurementOrigin == 1
                    coder.internal.assert(coder.internal.is_defined(obj.MeasurementJacobianFcn),'fusion:gmphd:extentRequireJacobian');
                end
                obj.pIsDetectReady = true;
            end
        end
        
        function testPredictionReadiness(obj)
            if ~obj.pIsPredictReady
                testGaussianPredictionReadiness(obj);
                obj.pIsPredictReady = true;
            end
        end
        
        function testGaussianPredictionReadiness(obj)
            % StateTransitionFcn must be defined
            coder.internal.assert(coder.internal.is_defined(obj.StateTransitionFcn),...
                'fusion:gmphd:undefinedProperty','StateTransitionFcn','predict');
            
            % ProcessNoise must be defined
            if obj.HasAdditiveProcessNoise && ~coder.internal.is_defined(obj.ProcessNoise)
                obj.pV = obj.pM;
                obj.ProcessNoise = cast(1,obj.pDataType);
            else
                coder.internal.assert(coder.internal.is_defined(obj.ProcessNoise),...
                    'fusion:gmphd:undefinedProperty','ProcessNoise','predict');
            end
        end
        
        function noise = validateProcessNoiseSize(obj,val)
            if isscalar(val) && coder.internal.is_defined(obj.pV)
                noise = val*eye(obj.pV,obj.pDataType);
            elseif ~isscalar(val) && coder.internal.is_defined(obj.pV)
                validateattributes(val,{'single','double'},{'size',[obj.pV obj.pV]},'gmphd','ProcessNoise');
                noise = val;
            else
                obj.pV = size(val,1);
                noise = val;
            end
        end
        
        function numComps = getAllowedSize(obj)
            numComps = obj.NumComponents;
        end
        
        function numComps = validateScalarProperty(obj,val,propName,classAllowed)
            numComps = getAllowedSize(obj);
            if nargin == 3
                classAllowed = {'single','double'};
            end
            validateattributes(val,classAllowed,{'real','finite','nonsparse','vector','numel',numComps},'gmphd',propName);
        end
        
        function allocateMemory(obj,states,covariances,varargin)
            validateattributes(states,{'single','double'},...
                {'real','finite','2d','nonsparse'},...
                'gmphd','States',1);
            
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
                'gmphd','StateCovariances',2);
            
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
            
            % These properties are either scalar or vectors, easy to allocate
            % memory for them.
            obj.pWeights = ones(1,m,dataType);
            obj.pLabels = zeros(1,m,'uint32');
            
            maxNumDetsIndex = fusion.internal.findProp('MaxNumDetections',varargin{:});
            if maxNumDetsIndex <= nargin - 3
                maxDets = varargin{maxNumDetsIndex + 1};
                validateattributes(maxDets,{'single','double'},{'integer','positive'},'gmphd','MaxNumDetections');
            else
                maxDets = 1000;
            end
            obj.pMaxNumDetections = cast(maxDets,obj.pDataType);
            obj.pNumDetections = obj.scalarZero;
                
            % If HasExtent is provided and MeasurementOrigin is not
            % specified, set MeasurementOrigin to center
            hasExtentIndex = fusion.internal.findProp('HasExtent',varargin{:});
            measOriginIndex = fusion.internal.findProp('MeasurementOrigin',varargin{:});
            if hasExtentIndex <= nargin - 3
                hasExtent = varargin{hasExtentIndex + 1};
                obj.HasExtent = hasExtent; % This will validate HasExtent
                if hasExtent && measOriginIndex > nargin - 3
                    obj.pMeasurementOrigin = fusion.internal.MeasurementOriginValue.center;
                end
            end
        end
        
        function propGroups = getPropertyGroups(obj)
            gauss = {'States','StateCovariances'};
            
            propGroups = matlab.mixin.util.PropertyGroup(gauss);
            
            stateTrans = {'StateTransitionFcn',...
                'StateTransitionJacobianFcn','ProcessNoise','HasAdditiveProcessNoise'};
            
            
            propGroups(2) = matlab.mixin.util.PropertyGroup(stateTrans);
                        
            extent = {'HasExtent'};
            if obj.HasExtent
                extent = {'HasExtent','MeasurementOrigin'};
            end
            propGroups(3) = matlab.mixin.util.PropertyGroup(extent);
            
            other = {'Weights','Labels'};
            propGroups(4) = matlab.mixin.util.PropertyGroup(other);
            
            models = {'Detections','MeasurementFcn','MeasurementJacobianFcn',...
                'HasAdditiveMeasurementNoise'};
            propGroups(5) = matlab.mixin.util.PropertyGroup(models);
            
        end
        
        function setStates(obj,x,P,inputID)
            if nargin < 4
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
        end
        
        function [x,P] = getStates(obj)
            n = obj.NumComponents;
            M = obj.MaxNumComponents;
            assert(n <= M,message('fusion:gmphd:exceedsMaxNumComponents'));
            indices = 1:n;
            x = obj.pStates(:,indices);
            P = obj.pStateCovariances(:,:,indices);
        end
        
        function [z,R,time] = getCurrentMeasurementAndNoise(obj)
            m = obj.pNumDetections;
            detBuffer = obj.pDetectionBuffer;
            % Collect all z and R
            if coder.target('MATLAB')
                allDets = [detBuffer{1:m}];
                z = horzcat(allDets.Measurement);
                R = cat(3,allDets.MeasurementNoise);
                time = horzcat(allDets.Time);
            else
                z = zeros(obj.pN,m);
                R = zeros(obj.pN,obj.pN,m);
                time = zeros(1,m);
                for i = 1:m
                    z(:,i) = detBuffer{i}.Measurement(:);
                    R(:,:,i) = detBuffer{i}.MeasurementNoise;
                    time(i) = detBuffer{i}.Time;
                end
            end
        end
        
        function val = scalarZero(obj)
            val = zeros(1,obj.pDataType);
        end
    end
    
    methods (Access = {?gmphd, ?matlab.unittest.TestCase})
        function args = getDefaultArgs(~)
            args =  {'StateTransitionFcn',@constvel,'StateTransitionJacobianFcn',@constveljac,...
                'MeasurementFcn',@cvmeas,'MeasurementJacobianFcn',@cvmeasjac,'HasAdditiveProcessNoise',false,...
                'HasAdditiveMeasurementNoise',true,'ProcessNoise',eye(3),'HasExtent',false};
        end
    end
    
    methods(Static,Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % Let the coder know about non-tunable parameters so that it
            % can generate more efficient code.
            props = {'MaxNumComponents','HasAdditiveProcessNoise',...
                'pN','pM','pV','pW','HasAdditiveMeasurementNoise','pDataType',...
                'pHasDetectionInput','pMeasurementOrigin'};
        end
    end
    
    methods (Static, Hidden)
        function phd = initializeFromTrack(track)
            % phd = initializeFromTrack initializes a phd filter from a
            % track. track can be objectTrack or a struct with similar
            % fields.
            phd = gmphd(track.State,...
                track.StateCovariance,...
                'Weights',1,...
                'MaxNumComponents',1);
        end
    end
end

function [zTotal,RTotal] = concatenateMeasurement(z,R,indices)
    numMeas = sum(indices);
    noiseSize = size(R,1);
    thisZ = z(:,indices);
    thisR = R(:,:,indices);
    zTotal = thisZ(:);
    RTotal = zeros(size(R,1)*numMeas,'like',R);
    for i = 1:numMeas
        index = ((i-1)*noiseSize + 1):(i*noiseSize);
        RTotal(index,index) = thisR(:,:,i);
    end
end

function detectionCell = getDetectionCell(detections, thisCell)
    if coder.target('MATLAB')
        detectionCell = detections(thisCell);
    else
        ind = find(thisCell);
        detectionCell = cell(numel(ind),1);
        for i = 1:numel(detectionCell)
            detectionCell{i} = detections{ind(i)};
        end
    end
end


