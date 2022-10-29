classdef ggiwphd< fusion.internal.AbstractPHDFilter & matlabshared.tracking.internal.fusion.CustomDisplay
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

    methods
        function out=ggiwphd
            % obj = GGIWPHD
            % obj = GGIWPHD(states,covariances);
            % obj = GGIWPHD(states,covariances,'Name',value);
        end

        function out=allocateMemory(~) %#ok<STOUT>
        end

        function out=append(~) %#ok<STOUT>
            % APPEND Append two filters.
            % APPEND (obj, obj2) appends the components in obj2 to the
            % components in obj. The total number of components in obj must
            % not exceed MaxNumComponents.
            % obj2 is a ggiwphd object.
        end

        function out=assertMemoryIsAllocatedForBuffer(~) %#ok<STOUT>
        end

        function out=assertMemoryIsAllocatedForJacobians(~) %#ok<STOUT>
        end

        function out=clone(~) %#ok<STOUT>
            % CLONE clones the filter
            % obj2 = CLONE(obj) returns a clone of the object, obj.
        end

        function out=correct(~) %#ok<STOUT>
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
        end

        function out=correctUndetected(~) %#ok<STOUT>
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
            % MeasurementRate: Estimated number of detections from the target
            % Extent: Estimated Extent matrix of the target.
            %
            % indices is a n-element vector providing indices of the
            % components whose states are extracted.
        end

        function out=getAllowedSize(~) %#ok<STOUT>
        end

        function out=getCurrentMeasurementAndNoise(~) %#ok<STOUT>
        end

        function out=getExpectedMeasAndJacobians(~) %#ok<STOUT>
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
            % LIKELIHOOD Calculate likelihood of detections.
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
            % MERGE Merge components in the filter
            % MERGE(obj,threshold) merges components in the filter whose
            % Kullback-Leibler difference is below the input, threshold.
        end

        function out=nullify(~) %#ok<STOUT>
            % nullifies the ggiwphd filter
        end

        function out=predict(~) %#ok<STOUT>
            % PREDICT Predict the filter dT time ahead.
            % PREDICT(obj,dT) predicts the filter dT time step ahead. The
            % function changes the state of the object,obj, with the
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
            % SCALE Scale the density
            % SCALE(obj,a) Scales the density by a.
            % a is a scalar or n-element vector representing the scaling
            % factor for each component. The weight of each component is
            % multiplied by a.
        end

        function out=setAdditiveNoiseSize(~) %#ok<STOUT>
        end

        function out=setStates(~) %#ok<STOUT>
        end

        function out=sync(~) %#ok<STOUT>
            % SYNC Synchronize obj with obj2
            % SYNC(obj,obj2) sets distribution properties of obj to match
            % those of obj2.
            % sync helps to ensure two filters produce same results on
            % calling methods like predict, correct, likelihood. All
            % non-tunable properties, function_handles are assumed same
            % between the two objects.
        end

        function out=testCorrectionReadiness(~) %#ok<STOUT>
        end

        function out=testDetectionReadiness(~) %#ok<STOUT>
            % MeasurementFcn must be defined.
        end

        function out=testGaussianPredictionReadiness(~) %#ok<STOUT>
            % StateTransitionFcn must be defined
        end

        function out=testIWPredictionReadiness(~) %#ok<STOUT>
            % ExtentRotationFcn must be defined.
        end

        function out=testPredictionReadiness(~) %#ok<STOUT>
        end

        function out=validatePosIndex(~) %#ok<STOUT>
        end

        function out=validateProcessNoiseSize(~) %#ok<STOUT>
        end

        function out=validateScalarProperty(~) %#ok<STOUT>
        end

    end
    properties
        % DegreesOfFreedom Inverse-Wishart parameter degrees of freedom of
        % each component.
        % DegreesOfFreedom is a 1-by-N vector, where N is the number of
        % components in the density.
        DegreesOfFreedom;

        % Detections Current set of detections
        % Detections is a cell array of objectDetection objects.
        % Use the property to set the current set of detections for the
        % filter.
        % Default = {}
        Detections;

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
        ExtentRotationFcn;

        % GammaForgettingFactors Forgetting factors to model the
        % exponential forgetting of the measurement rate. During state
        % transition shape and rate of the Gamma distribution is scaled
        % down by this factor. This leads to a transition of measurement
        % rate, where the expected value remains constant and the variance
        % increases proportional to the forgetting rate.
        GammaForgettingFactors;

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
        % Default = 1000
        MaxNumDetections;

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
        MeasurementFcn;

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
        MeasurementJacobianFcn;

        % NumComponents (read-only) Current number of components in the filter.
        NumComponents;

        % PositionIndex Indices of position in the state
        % For example, [1 3 5] for a constant velocity state defined according
        % to the convention [x;vx;y;vy;z;vz].
        PositionIndex;

        % ProcessNoise Process noise covariance (Q)
        %   If HasAdditiveProcessNoise is true: specify the covariance of
        %   process noise as a scalar or an M-by-M matrix.
        %
        %   If HasAdditiveProcessNoise is false: specify the covariance of
        %   process noise as a W-by-W matrix, where W is the number of the
        %   process noise terms.
        %
        %   Default: eye(M) if HasAdditiveProcessNoise is true
        ProcessNoise;

        % Rates Gamma parameter Rate of each component.
        % Rates is a 1-by-N vector, where N is the number of components in
        % the density.
        Rates;

        % ScaleMatrices Inverse-Wishart parameter scale matrix of each
        % component.
        % ScaleMatrices is a d-by-d-by-N matrix, where d is the number of
        % dimensions of target's extent and N is the number of components.
        ScaleMatrices;

        % Shapes Gamma parameter Shape of each component.
        % Shapes is a 1-by-N vector, where N is the number of components in
        % the density.
        Shapes;

        % StateCovariances Error covariance of each component's state
        % estimate.
        % StateCovariances is a P-by-P-by-N matrix, where each page or
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

        % States State of each component.
        % States is a P-by-N matrix, where P is the dimension of the state and
        % N is the number of components. Each column of the States define
        % the state of each component.
        States;

        % TemporalDecay A scalar value to control the variance of the
        % extent uncertainty during prediction. Lower the temporal decay,
        % higher the variance-increase during prediction.
        TemporalDecay;

        % Weights Weight of each component.
        Weights;

        pDegreesOfFreedom;

        pMaxNumDetections;

        pRates;

        pScaleMatrices;

        pShapes;

        pStateCovariances;

        pStates;

        pTemporalDecay;

    end
end
