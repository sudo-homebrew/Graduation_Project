classdef gmphd< fusion.internal.AbstractPHDFilter & matlabshared.tracking.internal.fusion.CustomDisplay
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

    methods
        function out=gmphd
            % obj = GMPHD
            % obj = GMPHD(states,covariances);
            % obj = GMPHD(states,covariances,'Name',value);
        end

        function out=allocateMemory(~) %#ok<STOUT>
        end

        function out=append(~) %#ok<STOUT>
            % APPEND Append two filters.
            % APPEND (obj, obj2) appends the components in obj2 to the
            % components in obj. The total number of components in obj must
            % not exceed MaxNumComponents.
            % obj2 is a gmphd object.
        end

        function out=assertMemoryIsAllocatedForBuffer(~) %#ok<STOUT>
        end

        function out=clone(~) %#ok<STOUT>
            % CLONE clone the filter
            % obj2 = CLONE(obj) returns a clone of the object, obj.
        end

        function out=correct(~) %#ok<STOUT>
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
        end

        function out=correctUndetected(~) %#ok<STOUT>
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
        end

        function out=extractState(~) %#ok<STOUT>
            % EXTRACTSTATE Extract target state estimates from the filter
            % [structStates,indices] = EXTRACTSTATE(obj,threshold)
            % extracts all states of components from the filter whose
            % weights are above the input, threshold.
            %
            % structStates is a n-element structure with the following
            % fields:
            % State: State estimate of the target
            % StateCovariances: Error in state estimate
        end

        function out=getAllowedSize(~) %#ok<STOUT>
        end

        function out=getCurrentMeasurementAndNoise(~) %#ok<STOUT>
        end

        function out=getMaxNumDetections(~) %#ok<STOUT>
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

        function out=getStates(~) %#ok<STOUT>
        end

        function out=labeledDensity(~) %#ok<STOUT>
            % LABELEDDENSITY Keep density with given labelID
            % LABELEDDENSITY(obj,L) removes all components from the filter
            % except components with Label set to L.
        end

        function out=likelihood(~) %#ok<STOUT>
            % LIKELIHOOD Calculate likelihood of current detections
            % lhood = LIKELIHOOD(obj,detectionIndices) returns the
            % log-likelihood for each column of detectionIndices.
            % detectionIndices is a logical matrix of size m-by-p, where m
            % is the number of total detections and p is the number of
            % detection cells.
            % lhood is a N-by-p matrix, where each column represents the
            % log-likelihood of N components in the obj against each cell
            % of detection.
        end

        function out=merge(~) %#ok<STOUT>
            % MERGE Merge components in the filter.
            % MERGE(obj,threshold) merges components in the filter whose
            % Kullback-Leibler difference is below the input, threshold.
        end

        function out=nullify(~) %#ok<STOUT>
            % nullifies the gmphd filter
        end

        function out=predict(~) %#ok<STOUT>
            % PREDICT Predict the filter dT time ahead.
            % PREDICT(obj,dT) predicts the filter dT time step ahead. The
            % function changes the state of the object, obj, with the
            % predicted state.
        end

        function out=prune(~) %#ok<STOUT>
            % PRUNE Prune the filter by removing selected components.
            % PRUNE(obj, pruneIndices) removes the components for which
            % pruneIndices is true.
            % pruneIndices is a logical vector of length N, where N is the
            % number of components in the filter.
        end

        function out=scalarZero(~) %#ok<STOUT>
        end

        function out=scale(~) %#ok<STOUT>
            % scale Scale the density
            % SCALE(obj,a) Scale the density by a.
            % a is a scalar or n-element vector representing the scaling
            % factor for each component. The weight of each component is
            % multiplied by a.
        end

        function out=setAdditiveNoiseSize(~) %#ok<STOUT>
        end

        function out=setStates(~) %#ok<STOUT>
        end

        function out=sync(~) %#ok<STOUT>
            % SYNC synchronize obj with obj2
            % SYNC(obj,obj2) sets distribution properties of obj to match
            % those of obj2.
            % SYNC helps to ensure two filters produce same results on
            % calling methods like predict, correct, likelihood. All
            % non-tunable properties, function_handles are assumed same
            % between the two objects.
        end

        function out=testDetectionReadiness(~) %#ok<STOUT>
            % If measurement origin is not defined, specify the default
            % "center".
        end

        function out=testGaussianPredictionReadiness(~) %#ok<STOUT>
            % StateTransitionFcn must be defined
        end

        function out=testPredictionReadiness(~) %#ok<STOUT>
        end

        function out=validateProcessNoiseSize(~) %#ok<STOUT>
        end

        function out=validateScalarProperty(~) %#ok<STOUT>
        end

    end
    properties
        % Detections Current set of detections
        % Detections is a cell array of objectDetection objects.
        % Use the property to set the current set of detections for the
        % filter.
        % Default: {}
        Detections;

        % HasAdditiveMeasurementNoise A Boolean flag that defines whether
        % the noise affecting the measurement is additive (true) or
        % non-additive (false).
        %
        % This property can only be set during filter construction.
        HasAdditiveMeasurementNoise;

        % HasAdditiveProcessNoise A Boolean flag that defines whether the
        % noise affecting the state transition is additive (true) or
        % non-additive (false).
        %
        % This property can only be set during filter construction.
        HasAdditiveProcessNoise;

        % HasExtent A flag to indicate if the components model extended
        % objects.
        % Set this value to true if the filter is intended to track
        % extended objects. Extended objects can generate more than one
        % measurement per sensor scan. Specifying HasExtent as true allows
        % specifying the MeasurementOrigin property.
        %
        %   Default: false
        HasExtent;

        % Labels Identifier for each component. Each component can be
        % assigned a label. The Labeling of a component allows maintaining
        % trajectories over time. Once a label is assigned to a component,
        % it is maintained during prediction and correction stages.
        Labels;

        % MaxNumComponents Maximum number of allowed components in the
        % filter.
        % You can only set this property during construction of the object.
        MaxNumComponents;

        % MaxNumDetections Maximum number of detections
        % A scalar value representing the maximum number of allowed
        % detections.
        % Default: 1000
        MaxNumDetections;

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
        MeasurementFcn;

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
        MeasurementJacobianFcn;

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
        MeasurementOrigin;

        % NumComponents (read-only) Current number of components in the filter.
        NumComponents;

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
        ProcessNoise;

        % StateCovariances Error covariance of each component's state
        % estimate.
        % StateCovariances is a M-by-M-by-N matrix, where each page or
        % sheet of the matrix defines the covariance of each component.
        StateCovariances;

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
        StateTransitionFcn;

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
        StateTransitionJacobianFcn;

        % States State of each component. States is a M-by-N matrix, where
        % M is the dimension of the state and N is the number of
        % components. Each column of the States defines the state of each
        % component.
        States;

        % Weights Weight of each component.
        Weights;

        pMaxNumDetections;

        pStateCovariances;

        pStates;

    end
end
