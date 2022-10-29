classdef (Hidden) EKF < handle
% This class is for internal use only. It may be removed in the future.

%EKF Extended Kalman filter
%
%   EKF methods:
%
%   predict  - Predict state and state error covariance
%   correct  - Correct state and state error covariance
%
%   EKF public properties:
%
%   State                       - State vector
%   StateCovariance             - State estimation error covariance
%   StateTransitionFcn          - Propagate state to next time step
%   ProcessNoise                - Process noise covariance
%   MeasurementFcn              - Calculate measurement from landmark position

% Copyright 2021 The MathWorks, Inc.

%#codegen

    properties(SetAccess = protected, GetAccess = public)
        %State State vector
        %   Initial state, specified as an M-element vector.
        %
        %   Default: [0; 0; 0]
        State;

        %StateCovariance State estimation error covariance
        %   State estimation error covariance, specified as an M-by-M
        %   matrix. M is the number of states.
        %
        %   Default: eye(3)
        StateCovariance;
    end

    properties(SetAccess = protected, GetAccess = public)
        %StateTransitionFcn Propagate state to next time step
        %   State transition function, specified as a function handle. This
        %   function calculates the state vector at time step k from the
        %   state vector at time step k-1. The function can take additional
        %   input parameters, such as control inputs or time step size. The
        %   function also calculates the Jacobians with respect to current
        %   pose and controller input. If not specified, the Jacobians are
        %   computed using numerical differencing at each call to the
        %   predict function. This computation can increase processing time
        %   and numerical inaccuracy.
        %
        %   The function considers non-additive process noise and should 
        %   have following signature:
        %       [pose(k),jacPose,jacControl] = ...
        %          StateTransitionFcn(pose(k-1), controlInput, parameters);
        %   where:
        %       'pose(k)' is the (estimated) pose at time k.
        %       'controlInput' is the input for propagating the state.
        %       'parameters' are any additional arguments that are needed
        %       by the state transition function.
        %       'jacPose' is Jacobian of StateTransitionFcn with respect to
        %       pose(k-1).
        %       'jacControl' is Jacobian of StateTransitionFcn with respect
        %       to controlInput.
        %
        %   Default: nav.algs.velocityMotionModel
        StateTransitionFcn;
    end

    properties(Access = public)
        %ProcessNoise Process noise covariance
        %   Process noise covariance, specified as a W-by-W matrix, where
        %   W is the number of the process noise terms.
        %
        %   Default: eye(2)
        ProcessNoise;
    end

    properties(Access = public)
        %MeasurementFcn Calculate measurement from landmark position
        %   Measurement function, specified as a function handle. This 
        %   function calculates N-element measurement vector for an 
        %   M-element state vector. The function also calculates the 
        %   Jacobians with respect to current pose and landmark position.
        %   If not specified, the Jacobians are computed using numerical 
        %   differencing at each call to the correct function. This 
        %   computation can increase processing time and numerical inaccuracy.
        %
        %   The function considers additive measurement noise and should 
        %   have following signature:
        %       [measurements(k),jacPose,jacLandmarks] = ...
        %                               MeasurementFcn(pose(k),landmarks)
        %   where:
        %       'pose(k)' is the (estimated) pose at time k.
        %       'measurements(k)' is the (estimated) measurement at time k.
        %       'landmarks' are position of landmarks.
        %       'jacPose' is Jacobian of MeasurementFcn with respect to
        %       pose(k).
        %       'jacLandmarks' is Jacobian of MeasurementFcn with respect
        %       to landmarks.
        %
        %   Default: nav.algs.rangeBearingMeasurement
        MeasurementFcn;
    end

    properties(Access = protected, Hidden)
        % Maximum number of poses
        MaxNumPoseStored = inf;

        % True if MaxNumPoseStored is set
        IsMaxNumPoseStoredDefined = false;
    end

    properties(Access = protected)
        % Current length of State
        CurStateLen = zeros(1,0);
        % Maximum state size
        MaxStateLen = inf;

        % Length of process noise
        ProcessNoiseLen = zeros(1,0);

        % Store the predicted and corrected pose
        StoredPredictedPose
        StoredCorrectedPose
        % Counter for predicted and corrected pose
        PredictedPoseCounter = 0;
        CorrectedPoseCounter = 0;
    end

    properties(Access = protected, Constant)
        % Number of elements representing the size of state vector of robot
        RobotStateSize = 3;

        % Number of elements representing the size of each landmark
        LandmarkStateSize = 2;
    end

    properties(Access = private, Constant)
        EKFcorrector = nav.algs.internal.ekfSLAM.EKFCorrectorAdditiveMulArgs;
    end

    methods
        %% PREDICT
        function predict(obj, controlInput, varargin)
        % PREDICT Predict state and state error covariance
        %   predict(SLAMOBJ, CONTROLINPUT) predicts the state and state 
        %   error covariance based on a given system model. It uses the 
        %   system model specified in StateTransitionFcn and the 
        %   controller input, CONTROLINPUT, to evolve the state.
        %
        %   predict(SLAMOBJ, CONTROLINPUT, VARARGIN) specifies additional
        %   arguments supplied in VARARGIN to the StateTransitionFcn 
        %   property of SLAMOBJ. The first input to StateTransitionFcn is 
        %   the pose from the previous time step, followed by all user 
        %   defined arguments.
        %
        %       CONTROLINPUT - Controller input required to propagate the
        %                      state from any initial value to any final
        %                      value, specified as an N-element vector.
        %
        %                      Note: The dimension of process noise should
        %                      be equal to the number of elements in
        %                      CONTROLINPUT.
        %
        %   Example:
        %       % Specify the Robot pose
        %       robotPose = [1; -2; 0.1];
        %
        %       % Specify Robot pose covariance
        %       robotPoseCovar = 0.1*eye(3);
        %
        %       % Specify input arguments to StateTransitionFcn
        %       velocity = [1 0];
        %       timeStep = 0.25;
        %
        %       % Create a new ekfSLAM object with initial robot pose and
        %       % robot pose covariance
        %       ekfSlamObj = ekfSLAM('State', robotPose, ...
        %                         'StateCovariance', robotPoseCovar);
        %
        %       % Run one prediction step
        %       predict(ekfSlamObj, velocity, timeStep);
        %
        %   See also ekfSLAM, correct
        
            % Check that number of arguments is atleast 2
            narginchk(2,inf);
            
            % validate controlInput
            validateattributes(controlInput, ...
                               {'single','double'}, ...
                               {'real', 'nonnan', 'finite', 'nonempty', ...
                                '2d', 'numel', obj.ProcessNoiseLen}, ...
                               'ekfSLAM', ...
                               'controlInput');

            if nargout(obj.StateTransitionFcn) == 3
                % User provided analytical jacobian fcn
                [statePred, dFdx, dFdv] = obj.StateTransitionFcn( ...
                    obj.State(1:obj.RobotStateSize), ...
                    controlInput, varargin{:});
            elseif nargout(obj.StateTransitionFcn) == 1
                % predict the state
                statePred = obj.StateTransitionFcn( ...
                    obj.State(1:obj.RobotStateSize), ...
                    controlInput, varargin{:});
                % Use numerical perturbation to get Jacobians
                dFdx = matlabshared.tracking.internal.numericJacobian( ...
                    obj.StateTransitionFcn, ...
                    {obj.State(1:obj.RobotStateSize), ...
                     controlInput, varargin{:}}, 1);
                dFdv = matlabshared.tracking.internal.numericJacobian( ...
                    obj.StateTransitionFcn, ...
                    {obj.State(1:obj.RobotStateSize), ...
                     controlInput, varargin{:}}, 2);
            else
                coder.internal.error( ...
                    'nav:navalgs:ekfslam:invalidNumOutputsFromFcn', ...
                    'StateTransitionFcn');
            end

            covarPred = obj.StateCovariance;
            % predict the state covariance
            covarPred(1:obj.RobotStateSize, 1:obj.RobotStateSize) = ...
                dFdx*covarPred(1:obj.RobotStateSize, 1:obj.RobotStateSize)*dFdx' + ...
                dFdv*obj.ProcessNoise*dFdv';

            % Cross correlation between robot pose covariance and landmark
            % covariance
            if obj.CurStateLen >  obj.RobotStateSize
                covarPred(1:obj.RobotStateSize, obj.RobotStateSize+1:obj.CurStateLen) = ...
                    dFdx*covarPred(1:obj.RobotStateSize, ...
                                   obj.RobotStateSize+1:obj.CurStateLen);
                covarPred(obj.RobotStateSize+1:obj.CurStateLen, 1:obj.RobotStateSize) = ...
                    covarPred(1:obj.RobotStateSize, ...
                              obj.RobotStateSize+1:obj.CurStateLen)';
            end

            obj.StateCovariance = covarPred;
            obj.State(1:obj.RobotStateSize) = statePred;

            % Store the predicted pose
            obj.PredictedPoseCounter = obj.PredictedPoseCounter+1;
            % Error out if MaxNumPoseStored size reached
            if ~coder.target('MATLAB') && ...
                    (obj.PredictedPoseCounter > obj.MaxNumPoseStored)
                coder.internal.error( ...
                    'nav:navalgs:ekfslam:MaxSizePoseVarReached');
            end
            if obj.PredictedPoseCounter <= obj.CorrectedPoseCounter
                obj.StoredPredictedPose( ...
                    obj.PredictedPoseCounter:obj.CorrectedPoseCounter,:) = NaN;
                obj.PredictedPoseCounter = obj.CorrectedPoseCounter+1;
            end
            obj.StoredPredictedPose(obj.PredictedPoseCounter,:) = ...
                obj.State(1:3)';
        end
    end

    methods
        %% CORRECT
        function correct(obj, associations, ...
                         measurement, measurementCovariance)
            % EKF correction for models with additive noise

            %   Note: No validation is done at this level. Any additional
            %   input validation should be done in a function or object
            %   that use this function.

            validateattributes(associations, ...
                               {class(obj.State)}, ...
                               {'real', 'nonnan', 'finite', 'nonempty', ...
                                '2d', 'positive', 'integer', 'ncols', 2}, ...
                               'ekfSLAM', ...
                               'associations');

            % Validate and transform measurement and measurementCovariance
            % in appropriate form for correction
            [meas, measCovar] = obj.reshapeMeasurement(associations, ...
                                                       measurement, ...
                                                       measurementCovariance);

            [obj.State(1:obj.CurStateLen), ...
             obj.StateCovariance(1:obj.CurStateLen,1:obj.CurStateLen)] =...
                                            obj.EKFcorrector.correct( ...
                meas, measCovar, ...
                obj.State(1:obj.CurStateLen), ...
                obj.StateCovariance(1:obj.CurStateLen,1:obj.CurStateLen), ...
                obj.MeasurementFcn, ...
                associations(:,1));

            % Store the corrected pose
            obj.CorrectedPoseCounter = obj.CorrectedPoseCounter + 1;
            % Error out if MaxNumPoseStored size reached
            if ~coder.target('MATLAB') && ...
                    (obj.CorrectedPoseCounter == obj.MaxNumPoseStored)
                coder.internal.error( ...
                    'nav:navalgs:ekfslam:MaxSizePoseVarReached');
            end
            if obj.CorrectedPoseCounter < obj.PredictedPoseCounter
                obj.StoredCorrectedPose( ...
                    obj.CorrectedPoseCounter:obj.PredictedPoseCounter-1,:) = NaN;
                obj.CorrectedPoseCounter = obj.PredictedPoseCounter;
            end
            obj.StoredCorrectedPose(obj.CorrectedPoseCounter,:) = ...
                obj.State(1:3)';
        end
    end

    methods % Set methods
        function set.StateTransitionFcn(obj, value)
            validateattributes(value, {'function_handle'},...
                               {'nonempty'}, ...
                               'ekfSLAM', ...
                               'StateTransitionFcn');
            obj.StateTransitionFcn = value;
        end

        function set.ProcessNoise(obj, value)
        % validate process noise value
            validateProcessNoise(obj, value);
            % cast the value to class of State
            castedValue = castValue(obj, value);
            % set the value
            obj.ProcessNoise = castedValue;
        end

        function set.MeasurementFcn(obj, value)
            validateattributes(value, {'function_handle'}, ...
                               {'nonempty'}, ...
                               'ekfSLAM', ...
                               'MeasurementFcn');

            obj.MeasurementFcn = value;
        end

        function set.MaxNumPoseStored(obj, value)
            validateattributes(value, {'single', 'double'}, ...
                               {'real', 'nonnan', 'finite', 'nonempty', ...
                                'positive', 'integer', 'scalar'}, ...
                               'ekfSLAM', ...
                               'MaxNumPoseStored');
            obj.MaxNumPoseStored = value;
        end
    end

    methods (Access = protected) % Validation methods
        function validateState(obj, value)
            validateattributes(value, {'single', 'double'}, ...
                               {'real', 'nonnan', 'nonempty', ...
                                'finite', 'vector'}, ...
                               'ekfSLAM', ...
                               'State');
            errorCond = numel(value)<obj.RobotStateSize || ...
                (rem((numel(value)-obj.RobotStateSize),obj.LandmarkStateSize));
            coder.internal.errorIf(errorCond, ...
                                   'nav:navalgs:ekfslam:InvalidNumEleState', ...
                                   obj.RobotStateSize, obj.LandmarkStateSize);

        end

        function validateStateCovariance(obj, value)
            validateattributes(value, {'single', 'double'}, ...
                               {'real', 'nonnan', 'nonempty', ...
                                'finite', 'square', '2d', ...
                                'nonsparse', ...
                                'size', [obj.CurStateLen, obj.CurStateLen]}, ...
                               'ekfSLAM', ...
                               'StateCovariance');
        end

        function validateProcessNoise(obj, value)
            if coder.internal.is_defined(obj.ProcessNoiseLen)
                validateattributes(value, {'single', 'double'}, ...
                                   {'real', 'nonnan', 'finite', 'nonempty', ...
                                    'square', '2d', 'nonsparse', ...
                                    'size', [obj.ProcessNoiseLen, obj.ProcessNoiseLen]}, ...
                                   'ekfSLAM', ...
                                   'ProcessNoise');
            elseif ~coder.internal.is_defined(obj.ProcessNoiseLen)
                validateattributes(value, {'single', 'double'}, ...
                                   {'real', 'nonnan', 'finite', 'nonempty', ...
                                    'square', '2d', 'nonsparse'}, ...
                                   'ekfSLAM', ...
                                   'ProcessNoise');
                obj.ProcessNoiseLen = size(value,1);
            end
        end
    end

    methods (Access = protected)
        function castedValue = castValue(obj, value)
            castedValue = cast(value, 'like', obj.State);
        end
    end

    methods (Access = private)
        % Put the measurement in appropriate form for correction
        function [meas, measCovar] = reshapeMeasurement(obj, associations, ...
                                                        measurement, ...
                                                        measurementCovariance)

            meas = cast(reshape(measurement(associations(:,2),:)',[],1),...
                        'like',obj.State);

            idxLMAssociated = associations(:,2);
            numMeas = size(associations(:,1),1);
            measCovar = zeros(2*numMeas,'like',obj.State);
            for i = 1:numMeas
                ii = 2*i + (-1:0);
                idx = (idxLMAssociated(i) * 2) + (-1:0);
                measCovar(ii, ii) = measurementCovariance(idx, idx);
            end
        end
    end
end
