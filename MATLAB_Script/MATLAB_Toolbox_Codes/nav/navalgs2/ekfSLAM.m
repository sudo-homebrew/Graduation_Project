classdef ekfSLAM < nav.algs.internal.ekfSLAM.EKF & ...
        fusion.internal.UnitDisplayer
%EKFSLAM Perform simultaneous localization and mapping using extended 
% Kalman filter
%   The EKFSLAM class performs simultaneous localization and mapping (SLAM)
%   using extended Kalman filter (EKF). It takes in observed landmarks from
%   the environment and associates it with the known landmarks to get the
%   associations and new landmarks. The associations are used to correct
%   the state and state covariance. The new landmarks are augmented in the 
%   state vector.
%
%   ekfSlam = EKFSLAM creates an EKF SLAM object, ekfSlam, with default
%   properties to implement Simultaneous Localization And Mapping using
%   Extended Kalman Filter.
%
%   ekfSlam = EKFSLAM('Name1', Value1, ..., 'NameN', ValueN) configures the
%   properties of the EKF SLAM object using one or more name-value pair
%   arguments. Unspecified properties have default values.
%
%   ekfSlam = EKFSLAM('MaxNumLandmark', N, 'Name1', Value1, ...,
%   'NameN', ValueN) specifies an upper bound on the number of landmarks in
%   the state vector while generating code. This limit is only required
%   when generating code. During MATLAB execution MAXNUMLANDMARK limit is
%   ignored.
%
%   ekfSlam = EKFSLAM('MaxNumLandmark', N, 'MaxNumPoseStored', M, ...
%   'Name1', Value1, ..., 'NameN', ValueN) specifies the maximum size of
%   pose history variable along with the maximum number of landmarks in the
%   state vector while generating code. These limits are only required
%   when generating code. During MATLAB execution MAXNUMLANDMARK limit and
%   MAXNUMPOSESTORED limit are ignored.
%
%   EKFSLAM methods:
%
%       predict        - Predict state and state error covariance
%       correct        - Correct state and state error covariance
%       removeLandmark - Remove landmark from the state vector
%       landmarkInfo   - Get landmark information
%       poseHistory    - Get corrected and predicted pose history
%       reset          - Reset state and state error covariance
%       copy           - Create deep copy of EKF SLAM object
%
%   EKFSLAM properties:
%
%       State                 - State vector
%       StateCovariance       - State estimation error covariance
%       MaxNumLandmark        - Maximum number of landmarks in State vector
%       StateTransitionFcn    - Propagate the state to next time step
%       ProcessNoise          - Process noise covariance
%       MeasurementFcn        - Calculate measurement from landmark
%                               position
%       InverseMeasurementFcn - Calculate landmark position from
%                               measurement
%       DataAssociationFcn    - Associate measurements to landmarks
%       MaxAssociationRange   - Maximum range up to which landmarks are
%                               checked for association
%
%   % Example:
%       % Load the dataset
%       load('racetrackDataset.mat', 'initialState', 'processNoise', ...
%            'initialStateCovariance', 'controllerInputs', 'timeStep', ...
%            'measurements', 'measCovar', 'validationGate');
%
%       % Initialize the ekfSLAM object
%       ekfSlamObj = ekfSLAM('State', initialState, ...
%                            'StateCovariance', initialStateCovariance, ...
%                            'ProcessNoise', processNoise);
%
%       % Initialize the variable to store the pose
%       storedPose = nan(size(controllerInputs,1)+1,3);
%       storedPose(1,:) = ekfSlamObj.State(1:3);
%
%       % Predict the state using the controller input.
%       % Then correct the state using the data of observed landmarks.
%       for count = 1:size(controllerInputs,1)
%           % Predict the state
%           predict(ekfSlamObj, controllerInputs(count,:), timeStep);
%
%           % Get the landmarks in the environment
%           observedLandmarks = measurements{count};
%
%           % Correct the state
%           if ~isempty(observedLandmarks)
%               correct(ekfSlamObj, observedLandmarks, measCovar, ...
%                       validationGate);
%           end
% 
%           % Log estimated pose
%           storedPose(count+1,:) = ekfSlamObj.State(1:3);
%       end
%
%       % Visualize the created map
%       fg = figure;
%       figAx = axes(fg);
%       axis equal;
%       grid minor;
%       hold on;
%       plot(figAx, storedPose(:,1), storedPose(:,2), 'g.-');
%       landmarks = reshape(ekfSlamObj.State(4:end),2,[])';
%       plot(figAx, landmarks(:,1), landmarks(:,2), 'm+');
%       plot(figAx, storedPose(1,1), storedPose(1,2),'k*');
%       plot(figAx, storedPose(end,1), storedPose(end,2),'rd');
%       legend('Robot trajectory', 'Landmarks', 'Start', 'End');
%
%   See also lidarSLAM

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties(Access = public)
        %DataAssociationFcn Associate measurements to landmarks
        %   Data association function, specified as a function handle. This
        %   function associates the measurements with the landmarks already
        %   available in state vector. The function may take additional 
        %   input parameters.
        %
        %   The function should have following signature:
        %   [associations, newLandmarks] = DataAssociationFcn(...
        %               knownLandmarks, knownLandmarksCovariance, ...
        %               observedLandmarks, observedLandmarksCovariance, ...
        %               parameters)
        %   where:
        %       'knownLandmarks' are known landmarks in the map.
        %       'knownLandmarksCovariance' is the covariance of knownLandmarks.
        %       'observedLandmarks' are observed landmarks in the environment.
        %       'observedLandmarksCovariance' is the covariance of observedLandmarks.
        %       'parameters' are any additional arguments that are needed.
        %       'associations' are list of associations of knownLandmarks to 
        %       observedLandmarks.
        %       'newLandmarks' are indices of observedLandmarks that
        %       qualified as new landmark.
        %
        %   Default: nav.algs.associateMaxLikelihood
        DataAssociationFcn = [];
    end

    properties(Access = public)
        %InverseMeasurementFcn Calculate landmark position from measurement
        %   Inverse measurement function, specified as a function handle. 
        %   This function calculates the landmark position as an M-element
        %   state vector for an N-element measurement vector. The function
        %   also calculates the Jacobians with respect to current pose and
        %   measurement.
        %   If not specified, the Jacobians are computed using numerical 
        %   differencing at each call to the correct function. This 
        %   computation can increase processing time and numerical 
        %   inaccuracy.
        %
        %   The function should have following signature:
        %       [landmarks(k),jacPose,jacMeasurements] = ...
        %                   InverseMeasurementFcn(pose(k),measurements)
        %   where:
        %       'pose(k)' is the (estimated) pose at time k.
        %       'landmarks(k)' is the landmark position at time k.
        %       'measurements' are observed landmarks at time k.
        %       'jacPose' is Jacobian of InverseMeasurementFcn with respect
        %       to pose(k).
        %       'jacMeasurements' is Jacobian of InverseMeasurementFcn with
        %       respect to measurements.
        %
        %   Default: nav.algs.rangeBearingInverseMeasurement
        InverseMeasurementFcn;
    end

    properties(Access = public)
        %MaxAssociationRange Maximum range up to which landmarks are checked for association
        %   Maximum range for the landmarks to be checked for association,
        %   specified as a numeric scalar.
        %
        %   Default: inf
        MaxAssociationRange;
    end

    properties(SetAccess = protected, GetAccess = public)
        %MaxNumLandmark Maximum number of landmarks in State vector
        %   Maximum number of landmarks in the State vector, specified as a 
        %   numeric scalar.
        %
        %   Default: inf
        MaxNumLandmark = inf;
    end

    properties(Access = protected, Hidden)
        % True if MaxNumLandmark is set
        IsMaxNumLandmarkDefined = false;
    end

    methods
        % Constructor
        function obj = ekfSLAM(varargin)
        % ekfSLAM Construct an instance of the ekfSLAM

            % Check1: odd number of inputs to constructor
            if (nargin > 0) && rem(nargin,2) ~= 0
                coder.internal.error( ...
                    'nav:navalgs:ekfslam:InvalidInputsToConstructor');
            end

            % Check2: codegen path and first NV pair is not MaxNumLandmark
            if ~coder.target('MATLAB')
                if nargin == 0
                coder.internal.assert(~(nargin == 0), ...
                    'nav:navalgs:ekfslam:MaxNumLandmarkRequiredForCodegen');
                elseif ~strcmp(varargin{1}, 'MaxNumLandmark')
                coder.internal.assert(strcmp(varargin{1}, 'MaxNumLandmark'), ...
                    'nav:navalgs:ekfslam:MaxNumLandmarkRequiredForCodegen');
                end
            end

            if ~coder.target('MATLAB')
                % Set MaxNumLandmark
                obj.MaxNumLandmark = varargin{2};
                obj.IsMaxNumLandmarkDefined = true;

                % Check and set MaxNumPoseStored
                if (nargin > 2) && strcmp(varargin{3}, 'MaxNumPoseStored')
                    obj.MaxNumPoseStored = varargin{4};
                    % Pass the remaining PV pairs for parsing and setting
                    parseSetPV(obj,varargin{5:end});
                    obj.IsMaxNumPoseStoredDefined = true;
                else
                    % If MaxNumPoseStored is not passed then pass all the 
                    % PV pairs for parsing and setting
                    parseSetPV(obj,varargin{3:end});
                    obj.IsMaxNumPoseStoredDefined = false;
                end
            else
                % parse all the PV pairs and set it accordingly
                parseSetPV(obj,varargin{:});
            end

            % initialize the variables to store predicted and corrected poses
            obj.initializePoseHistoryVariables();
        end

        %% Correct
        function [associations, newLandmarks] = correct(obj, measurement,...
                                                       measurementCovariance, varargin)
            %correct Correct state and state error covariance
            %   [ASSOCIATIONS, NEWLANDMARKS] = correct(SLAMOBJ, MEASUREMENT,
            %   MEASUREMENTCOVARIANCE) corrects the state and its
            %   associated state covariance based on the measurement and
            %   measurementCovariance at the current time step.
            %   The correct method uses the data association function
            %   specified in the DataAssociationFcn property of SLAMOBJ.
            %   The function associates the measurement to landmarks, and
            %   extract new landmarks from the measurement. The function
            %   then uses the associations to correct the state and
            %   associated state covariance. It then augments the state
            %   with new landmarks.
            %
            %   [ASSOCIATIONS, NEWLANDMARKS] = correct(SLAMOBJ, MEASUREMENT,
            %   MEASUREMENTCOVARIANCE, VARARGIN) specifies additional
            %   arguments supplied in VARARGIN to the DataAssociationFcn
            %   property of SLAMOBJ. The first four inputs to
            %   DataAssociationFcn are landmark positions, landmark
            %   positions covariance, MEASUREMENT and MEASUREMENTCOVARIANCE
            %   followed by user defined arguments.
            %
            %   Input Arguments:
            %
            %     MEASUREMENT           - Measurement of landmarks in the
            %                             environment, specified as an
            %                             N-by-K matrix, where K is the
            %                             dimension of the measurement and
            %                             N is the number of measurements.
            %
            %     MEASUREMENTCOVARIANCE - Covariance of measurements
            %                             specified as a K-element vector
            %                             or as an N*K-by-N*K matrix, where
            %                             K is the dimension of the
            %                             landmarks and N is the number of
            %                             measurements. If specified as a
            %                             K-element vector, then same
            %                             measurement covariance matrix is
            %                             assumed for all the measurements.
            %
            %   Output Arguments:
            %
            %     ASSOCIATIONS          - List of associations of landmarks
            %                             to measurements, returned as a
            %                             P-by-2 matrix. P is the number of
            %                             associations. The first column of
            %                             the matrix contains the indices
            %                             of associated landmarks and the
            %                             second column contains the
            %                             associated measurement indices.
            %
            %     NEWLANDMARKS          - List of indices of measurements
            %                             that qualified as new landmark,
            %                             returned as a Q-element vector.
            %                             Q is the number of measurements
            %                             that qualified as a new landmark.
            %
            %   Example:
            %       % Specify the Robot pose
            %       robotPose = [1; -2; 0.1];
            %
            %       % Specify initial landmark positions
            %       landmarkPosition = [15.8495; -12.9496; 25.2455; -15.4705;
            %                           37.5880;   3.1023; 16.5690;   2.7466];
            %
            %       % Specify initial covariance
            %       initialStateCovar = diag([0.1*ones(1,3) 1.1*ones(1,8)]);
            %
            %       % Specify measurement and measurement covariance
            %       measRB = [18.4500, -0.7354; 27.7362, -0.6071;
            %                 36.9421,  0.0386; 16.2765,  0.1959];
            %       measCovar = [0.1^2, (1.0*pi/180)^2];
            %
            %       % Create a new ekfSLAM object with initial state and
            %       % initial state covariance
            %       ekfSlamObj = ekfSLAM('State', [robotPose; landmarkPosition], ...
            %                            'StateCovariance', initialStateCovar);
            %
            %       % Call correct
            %       correct(ekfSlamObj, measRB, measCovar, 5.991);
            %
            %   See also ekfSLAM, predict

            % Check that number of arguments is atleast 3
            narginchk(3,inf);

            % validate measurement and measurementCovariance
            measCovar = obj.validateExpandMeasurement(measurement, ...
                                                      measurementCovariance);
            measurementCasted = cast(measurement,'like',obj.State);

            % call associate data with raw measurement and
            % measurementCovariance
            [associations, newLandmarks] = obj.associateData( ...
                measurementCasted, measurementCovariance, varargin{:});

            % call correct if there is any association
            if ~isempty(associations) && ...
                    ~(obj.CurStateLen <= obj.RobotStateSize)
                correct@nav.algs.internal.ekfSLAM.EKF( ...
                    obj, associations, measurementCasted, measCovar);
            end

            % call augment state if there is any new landmark
            if ~isempty(newLandmarks)
                obj.augmentState(newLandmarks, measurementCasted, measCovar);
            end
        end

        %% Get Stored States
        function [correctedPose, predictedPose] = poseHistory(obj)
        %poseHistory Retrieve corrected and predicted pose history
        %   [CORRECTEDPOSE, PREDICTEDPOSE] = poseHistory(SLAMOBJ) retrieves
        %   the corrected and predicted pose till the current pose.
        %   The function throws an error if MaxNumPoseStored is not 
        %   specified when the function is called during code generation.
        %
        %       CORRECTEDPOSE - All the corrected pose returned as an M-by-3
        %                       matrix of form [X Y Yaw]. X and Y specify
        %                       the position in meters. Yaw specifies the
        %                       orientation in radians.
        %
        %       PREDICTEDPOSE - All the predicted pose returned as an M-by-3
        %                       matrix of form [X Y Yaw]. X and Y specify
        %                       the position in meters. Yaw specifies the
        %                       orientation in radians.
        %
        %   Example:
        %       % Specify the Robot pose
        %       robotPose = [1; -2; 0.1];
        %
        %       % Specify initial landmark positions
        %       landmarkPosition = [15.8495; -12.9496; 25.2455; -15.4705;
        %                           37.5880;   3.1023; 16.5690;   2.7466];
        %
        %       % Specify initial covariance
        %       initialStateCovar = diag([0.1*ones(1,3) 1.1*ones(1,8)]);
        %
        %       % Specify input arguments to StateTransitionFcn
        %       velocity = [1 0];
        %       timeStep = 0.25;
        %
        %       % Specify measurement and measurement covariance
        %       measRB = [18.4500, -0.7354; 27.7362, -0.6071;
        %                 36.9421,  0.0386; 16.2765,  0.1959];
        %       measCovar = [0.1^2, (1.0*pi/180)^2];
        %
        %       % Create a new ekfSLAM object with initial state and
        %       % initial state covariance
        %       ekfSlamObj = ekfSLAM('State', [robotPose; landmarkPosition], ...
        %                            'StateCovariance', initialStateCovar);
        %
        %       % Run one prediction step
        %       predict(ekfSlamObj, velocity, timeStep);
        %
        %       % Call correct
        %       correct(ekfSlamObj, measRB, measCovar, 5.991);
        %
        %       % Get the pose history
        %       [corrPose, predPose] = poseHistory(ekfSlamObj);
        %
        %   See also ekfSLAM, correct, landmarkInfo

            % Error out if MaxNumPoseStored is not defined and poseHistory
            % is called on codegen path
            if ~coder.target('MATLAB') && ~obj.IsMaxNumPoseStoredDefined
                coder.internal.error( ...
                    'nav:navalgs:ekfslam:MaxNumPoseNeedToBeDefinedForCG');
            end
            % return the stored corrected and predicted pose
            correctedPose = obj.StoredCorrectedPose;
            predictedPose = obj.StoredPredictedPose;
        end

        %% Get landmark
        function info = landmarkInfo(obj, varargin)
        %LANDMARKINFO Retrieve landmark information
        %   INFO = LANDMARKINFO(SLAMOBJ) returns a table containing the
        %   landmark number along with position and state index of all the
        %   landmarks.
        %
        %   INFO = LANDMARKINFO(SLAMOBJ, LANDMARKINDEX) returns a table
        %   containing the landmark number along with position and state
        %   index for the landmarks specified by LANDMARKINDEX.
        %
        %       LANDMARKINDEX - Indices of landmarks for which information
        %                       is required specified as an N-element column
        %                       vector of landmarks number or N-by-2 matrix
        %                       of exact position of landmarks in the state
        %                       vector. N is the number of landmarks.
        %
        %   Example:
        %       % Specify the Robot pose
        %       robotPose = [1; -2; 0.1];
        %
        %       % Specify initial landmark positions
        %       landmarkPosition = [15.8495; -12.9496; 25.2455; -15.4705;
        %                           37.5880;   3.1023; 16.5690;   2.7466];
        %
        %       % Specify initial covariance
        %       initialStateCovar = diag([0.1*ones(1,3) 1.1*ones(1,8)]);
        %
        %       % Create a new ekfSLAM object with initial state and
        %       % initial state covariance
        %       ekfSlamObj = ekfSLAM('State', [robotPose; landmarkPosition], ...
        %                            'StateCovariance', initialStateCovar);
        %
        %       % Get the information for all the landmarks
        %       landmarkInfo(ekfSlamObj);
        %
        %   See also ekfSLAM, correct, poseHistory

            % Check that number of arguments is either 1 or 2
            narginchk(1, 2);

            % Error out if there is no landmark and info is queried
            if obj.CurStateLen == obj.RobotStateSize
                coder.internal.error('nav:navalgs:ekfslam:Nolandmark');
            end

            % get the landmarks present in State vector
            LMs = reshape(obj.State(obj.RobotStateSize+1:end),2,[])';
            % check if landmarkIndex is passed
            if nargin == 2
                % get the landmark index
                landmarkIndex = varargin{1};
                % validate common attributes for both landmark number and
                % state index
                validateattributes(landmarkIndex, {'double', 'single'}, ...
                                   {'real', 'nonnan', 'nonempty', 'finite', ...
                                    '2d', 'positive', 'integer'}, ...
                                   'ekfSLAM', ...
                                   'landmarkIndex');
                if size(landmarkIndex,2) == 1 % landmark number is given
                                              % validate that landmark number is a valid number based
                                              % on the number of landmarks in the State vector
                    validateattributes(landmarkIndex, ...
                                       {'double', 'single'}, ...
                                       {'<=', (obj.CurStateLen-obj.RobotStateSize)/2}, ...
                                       'ekfSLAM', ...
                                       'landmarkIndex');
                    % take out unique landmark numbers
                    landmarkNum = unique(landmarkIndex, 'stable');
                elseif size(landmarkIndex,2) == 2 % state index is given
                                                  % check that state index is lesser than the size of the
                                                  % State vector
                    validateattributes(landmarkIndex, ...
                                       {'double', 'single'}, ...
                                       {'>', obj.RobotStateSize, '<=', obj.CurStateLen}, ...
                                       'ekfSLAM', ...
                                       'landmarkIndex');
                    % check that first state index is even
                    validateattributes(landmarkIndex(:,1), {'double', 'single'}, ...
                                       {'even'}, ...
                                       'ekfSLAM', ...
                                       'first landmarkIndex');

                    % check values in row are consecutive else error out
                    diffLMIdx = diff(landmarkIndex,1,2);
                    errorCond = any(diffLMIdx ~= 1);
                    coder.internal.errorIf(errorCond, ...
                                           'nav:navalgs:ekfslam:InvalidLandmarkIndex', ...
                                           'landmarkIndex');

                    % take out unique state index
                    landmarkIndexUnique = ...
                        unique(landmarkIndex, 'stable', 'rows');
                    % take out landmark number from state index
                    landmarkNum = (landmarkIndexUnique(:,1))/2-1;
                else
                    % error out if landmark index passed has any other size
                    coder.internal.error( ...
                        'nav:navalgs:ekfslam:InvalidSize', ...
                        'landmarkIndex');
                end
            else
                % if landmark index is not passed then give info of all the
                % landmarks
                landmarkNum = (1:size(LMs,1))';
            end
            % cast all the output value to class of State
            landmarkNum = cast(landmarkNum,'like',obj.State);
            landmarkPos = cast(LMs(landmarkNum,:),'like',obj.State);
            landmarkStateIndex = cast((repmat(landmarkNum,1,2)*2 + ...
                                       repmat([2 3],size(landmarkNum,1),1)), ...
                                       'like',obj.State);
            % create the table
            info = table(landmarkNum, landmarkStateIndex, landmarkPos, ...
                         'VariableNames',{'landmark number', ...
                                          'landmark state index', ...
                                          'landmark position'});
        end

        %% Remove Landmark
        function removeLandmark(obj, landmarkIndex)
        %removeLandmark Remove landmark from state vector
        %   removeLandmark(SLAMOBJ, LANDMARKINDEX) removes landmark from
        %   the state vector along with associated covariance from the
        %   state covariance matrix. The landmarks are specified by
        %   LANDMARKINDEX.
        %
        %       LANDMARKINDEX - Indices of landmark to be removed, specified
        %                       as an N-by-1 vector of landmark number in
        %                       the state vector or N-by-2 matrix of exact
        %                       position of landmarks in the state vector.
        %                       N is the number of landmarks to be removed.
        %
        %   Example:
        %       % Specify the Robot pose
        %       robotPose = [1; -2; 0.1];
        %
        %       % Specify initial landmark positions
        %       landmarkPosition = [15.8495; -12.9496; 25.2455; -15.4705;
        %                           37.5880;   3.1023; 16.5690;   2.7466];
        %
        %       % Specify initial covariance
        %       initialStateCovar = diag([0.1*ones(1,3) 1.1*ones(1,8)]);
        %
        %       % Create a new ekfSLAM object with initial state and
        %       % initial state covariance
        %       ekfSlamObj = ekfSLAM('State', [robotPose; landmarkPosition], ...
        %                            'StateCovariance', initialStateCovar);
        %
        %       % Specify the landmark number to be removed
        %       removeLandmark(ekfSlamObj,3);
        %
        %   See also ekfSLAM, correct

            % error out if there is no landmark in the State vector and
            % remove is called
            if obj.CurStateLen == obj.RobotStateSize && ...
                    ~isempty(landmarkIndex)
                coder.internal.error('nav:navalgs:ekfslam:Nolandmark');
            end
            % validate common attributes for both landmark number and
            % state index
            validateattributes(landmarkIndex, {'double', 'single'}, ...
                               {'real', 'nonnan', 'nonempty', 'finite', ...
                                '2d', 'positive', 'integer'}, ...
                               'ekfSLAM', ...
                               'landmarkIndex');
            % validate specified attributes
            if size(landmarkIndex,2) == 1 % landmark number is given
                                          % validate that landmark number is a valid number based
                                          % on the number of landmarks in the State vector
                validateattributes(landmarkIndex, {'double', 'single'}, ...
                                   {'<=', (obj.CurStateLen-obj.RobotStateSize)/2}, ...
                                   'ekfSLAM', ...
                                   'landmarkIndex');

                % take out unique landmark numbers
                landmarkIndex = unique(landmarkIndex, 'stable');
                % number of landmarks to remove
                numLMToRemove = size(landmarkIndex,1);
                % convert landmark index in a linear state index
                landmarkIndexLinear = reshape(((landmarkIndex*2)+(2:3))',[],1);
            elseif size(landmarkIndex,2) == 2 % state index is given
                % check that state index is lesser than the size of the
                % State vector
                validateattributes(landmarkIndex, {'double', 'single'}, ...
                                   {'>', obj.RobotStateSize, '<=', obj.CurStateLen}, ...
                                   'ekfSLAM', ...
                                   'landmarkIndex');
                % check that first state index is even
                validateattributes(landmarkIndex(:,1), {'double', 'single'}, ...
                                   {'even'}, ...
                                   'ekfSLAM', ...
                                   'first landmarkIndex');

                % check values in row are consecutive else error out
                diffLMIdx = diff(landmarkIndex,1,2);
                errorCond = any(diffLMIdx ~= 1);
                coder.internal.errorIf(errorCond, ...
                                       'nav:navalgs:ekfslam:InvalidLandmarkIndex', ...
                                       'landmarkIndex');

                % take out unique values
                landmarkIndex = unique(landmarkIndex, 'stable', 'rows');
                % number of landmarks to remove
                numLMToRemove = size(landmarkIndex, 1);
                % convert landmark index in a linear state index
                landmarkIndexLinear = reshape(landmarkIndex',[],1);
            else
                % error out if landmark index passed has any other size
                coder.internal.error( ...
                    'nav:navalgs:ekfslam:InvalidSize', ...
                    'landmarkIndex');
            end
            % reduce the current state length
            obj.CurStateLen = obj.CurStateLen-numLMToRemove*2;
            % remove the landmarks from the State vector
            obj.State(landmarkIndexLinear) = [];

            % remove respective values from State Covariance matrix
            covar = obj.StateCovariance;
            covar(landmarkIndexLinear,:) = [];
            covar(:,landmarkIndexLinear) = [];
            obj.StateCovariance = covar;
        end

        %% Reset
        function reset(obj)
        % reset Reset state and state estimation error covariance
        %   reset(OBJ) resets the state and state error covariance to
        %   their default values and resets the internal states.
        %
        %   Example
        %       % Specify the Robot pose
        %       robotPose = [1; -2; 0.1];
        %
        %       % Specify Robot pose covariance
        %       robotPoseCovar = 0.1*eye(3);
        %
        %       % Create a new ekfSLAM object with initial robot pose and
        %       % robot pose covariance
        %       ekfSlamObj = ekfSLAM('State', robotPose, ...
        %                         'StateCovariance', robotPoseCovar);
        %
        %       % Call reset
        %       reset(ekfSlamObj);
        %
        %   See also ekfSLAM, removeLandmark

            % Get the length of state on codegen and MATLAB path
            if ~coder.target('MATLAB') && obj.IsMaxNumLandmarkDefined
                stateLen = obj.MaxStateLen;
            else
                stateLen = obj.RobotStateSize;
            end
            % Define current state length
            obj.CurStateLen = obj.RobotStateSize;
            % Set the state based on length of the state, stateLen
            obj.State = zeros(stateLen,1,'like',obj.State);
            obj.StateCovariance = eye(stateLen,'like',obj.State);

            % re-initialize pose history variables
            obj.initializePoseHistoryVariables();
        end

        %% Copy
        function newObj = copy(obj)
        %copy Create deep copy of EKF SLAM object
        %   NEWOBJ = COPY(OBJ) creates a deep copy of EKF SLAM object with
        %   same properties.

            % Construct a new object
            if coder.target('MATLAB')
                newObj = ekfSLAM('State', obj.State, ...
                                 'StateCovariance', obj.StateCovariance, ...
                                 'StateTransitionFcn', obj.StateTransitionFcn);
            elseif obj.IsMaxNumLandmarkDefined && obj.IsMaxNumPoseStoredDefined
                newObj = ekfSLAM('MaxNumLandmark', obj.MaxNumLandmark, ...
                                 'MaxNumPoseStored', obj.MaxNumPoseStored, ...
                                 'State', obj.State, ...
                                 'StateCovariance', obj.StateCovariance, ...
                                 'StateTransitionFcn', obj.StateTransitionFcn);
            else
                newObj = ekfSLAM('MaxNumLandmark', obj.MaxNumLandmark, ...
                                 'State', obj.State, ...
                                 'StateCovariance', obj.StateCovariance, ...
                                 'StateTransitionFcn', obj.StateTransitionFcn);
            end

            % Copy remaining public properties
            newObj.MeasurementFcn        = obj.MeasurementFcn;
            newObj.InverseMeasurementFcn = obj.InverseMeasurementFcn;
            newObj.DataAssociationFcn    = obj.DataAssociationFcn;
            newObj.ProcessNoise          = obj.ProcessNoise;
            newObj.MaxAssociationRange   = obj.MaxAssociationRange;

            % Copy all internal properties
            newObj.CurStateLen               = obj.CurStateLen;
            newObj.MaxStateLen               = obj.MaxStateLen;
            newObj.ProcessNoiseLen           = obj.ProcessNoiseLen;
            newObj.PredictedPoseCounter      = obj.PredictedPoseCounter;
            newObj.CorrectedPoseCounter      = obj.CorrectedPoseCounter;
            newObj.StoredPredictedPose       = obj.StoredPredictedPose;
            newObj.StoredCorrectedPose       = obj.StoredCorrectedPose;
            newObj.IsMaxNumLandmarkDefined   = obj.IsMaxNumLandmarkDefined;
            newObj.IsMaxNumPoseStoredDefined = obj.IsMaxNumPoseStoredDefined;
        end
    end

    methods (Access = protected)
        %% Data Association
        function [associations, newLandmark] = associateData(obj, varargin)
        %   Note: No validation is done at this level. Any additional
        %   input validation should be done in a function or object
        %   that uses this function.

            if (obj.CurStateLen > obj.RobotStateSize)
                % get the distance of each landmark from the vehicle
                landmarks = reshape(obj.State(obj.RobotStateSize+1:obj.CurStateLen),...
                    obj.LandmarkStateSize,[])';
                dist = vecnorm(landmarks - ...
                    repmat(obj.State(1:2)', size(landmarks,1), 1),2,2);
                % get indices of landmarks that are within the Range
                idxWithinRange = find(dist <= obj.MaxAssociationRange);
                if ~isempty(idxWithinRange)
                    % get the transformed landmarks and landmarksCovariance
                    landmarksWithinRange = landmarks(idxWithinRange,:);
                    numLM = size(landmarksWithinRange,1);
                    [landmarksMeasSpace, jacPose, jacCart] = ...
                        predictedMeasJacobian(obj, landmarksWithinRange, ...
                                              size(varargin{1},2));

                    % preallocate covariance matrix
                    covarLM = zeros(numLM*2,numLM*2);
                    % get the transformed covariance
                    for k = 1:numLM
                        landmarkPos = idxWithinRange(k)*2 + [2,3];
                        index = [1:3 landmarkPos];
                        covarSub = obj.StateCovariance(index,index);
                        idx2 = k*2 + [-1,0];
                        H = [jacPose(idx2,:), jacCart(idx2,:)];
                        covarLM(idx2,idx2) = H*covarSub*H';
                    end

                    % call the DataAssociationFcn with transformed landmarks
                    % and landmarksCovariance
                    [associations, newLandmark] = obj.DataAssociationFcn( ...
                        landmarksMeasSpace, covarLM, varargin{:});
                    associations(:,1) = idxWithinRange(associations(:,1));

                else    % isempty(idxWithinRange)
                    % if there is no landmark within the range then pass
                    % empty vector as landmark and landmark covariance
                    [associations, newLandmark] = ...
                        obj.DataAssociationFcn([], [], varargin{:});
                end
            else        % (obj.CurStateLen <= obj.RobotStateSize)
                % if there is no landmark in the state vector then pass
                % empty vector as landmark and landmark covariance
                [associations, newLandmark] = ...
                    obj.DataAssociationFcn([], [], varargin{:});
            end
        end

        %% State Augmentation
        function augmentState(obj, newLandmark, ...
                              measurement, measurementCovariance)
            %augmentState Augment state vector
            %   augmentState(SLAMOBJ, NEWLANDMARK, MEASUREMENT,
            %   MEASUREMENTCOVARIANCE) augments the state vector with new
            %   landmarks and updates the state covariance matrix with
            %   covariance of new landmarks.
            %
            %    NEWLANDMARK            - Indices new landmarks, specified
            %                             as an M-element vector. M is
            %                             the number of new landmarks.
            %
            %     MEASUREMENT           - Measurement of landmarks in the
            %                             environment, specified as an
            %                             N-by-K matrix, where K is the
            %                             dimension of the measurement and
            %                             N is the number of measurements.
            %
            %     MEASUREMENTCOVARIANCE - Covariance of measurements
            %                             specified as an N*K-by-N*K matrix,
            %                             where K is the dimension of the
            %                             landmarks and N is the number of
            %                             measurements. If specified as a
            %                             K-element vector, then same
            %                             measurement covariance matrix is
            %                             assumed for all the measurements.

            %   Note: Only newLandmark input is validated at this level.
            %   Any additional input validation should be done in a
            %   function or object that use this function.

            % validate new landmark
            validateattributes(newLandmark, ...
                               {class(obj.State)}, ...
                               {'real', 'nonnan', 'finite', 'nonempty', ...
                                'vector', 'positive', 'integer'}, ...
                               'ekfSLAM', ...
                               'newLandmark');

            % get the associated measurements
            meas = cast(measurement(newLandmark,:),'like',obj.State);

            % Augment state vector with 1 landmark at a time
            for i=1:size(meas,1)
                covarIndex = i*2 + [-1,0];
                obj.augmentStateOnce(meas(i,:), ...
                                     measurementCovariance(covarIndex, covarIndex));
            end
        end
    end

    methods (Access = protected)
        function displayScalarObject(obj)
            displayScalarObjectWithUnits(obj);
        end

        function groups = getPropertyGroups(obj)
        % Add section titles to property display.
            if ~isscalar(obj)
                groups = getPropertyGroups@matlab.mixin.CustomDisplay(obj);
            else
                stateList.State = obj.State;
                stateList.StateCovariance = obj.StateCovariance;
                stateList.MaxNumLandmark = obj.MaxNumLandmark;
                stateList.StateTransitionFcn = obj.StateTransitionFcn;
                stateList.ProcessNoise = obj.ProcessNoise;

                measList.MeasurementFcn = obj.MeasurementFcn;
                measList.InverseMeasurementFcn = ...
                    obj.InverseMeasurementFcn;

                dataAssocList.DataAssociationFcn = obj.DataAssociationFcn;
                dataAssocList.MaxAssociationRange = obj.MaxAssociationRange;

                stateGroup = matlab.mixin.util.PropertyGroup(stateList);
                measGroup = matlab.mixin.util.PropertyGroup(measList);
                dataAssocGroup = ...
                    matlab.mixin.util.PropertyGroup(dataAssocList);

                groups = [stateGroup, measGroup, dataAssocGroup];
            end
        end
    end

    methods (Access = protected)
        % validate measurement and measurementCovariance
        % expand measurementCovariance if it is a vector
        function measCovar = validateExpandMeasurement(obj, measurement,...
                                                    measurementCovariance)
            % validate measurement
            validateattributes(measurement, {'single', 'double'}, ...
                               {'real', 'nonnan','finite', ...
                                '2d', 'nonempty'}, ...
                               'ekfSLAM', ...
                               'measurement');

            [numMeas, dimension] = size(measurement);
            sizeCovar = numMeas*dimension;

            % Check the condition for size of covariance
            covSizeCond = ...
                (numel(measurementCovariance) == dimension) || ...
                all(size(measurementCovariance) == [sizeCovar, sizeCovar]);

            % Throw error if covariance is not in expected format
            coder.internal.errorIf(~covSizeCond, ...
                                   'nav:navalgs:ekfslam:InvalidCovarianceMatrix', ...
                                   'measurementCovariance', dimension, sizeCovar);

            % Validate measurementCovariance
            if numel(measurementCovariance) == dimension
                validateattributes(measurementCovariance, ...
                                   {'single', 'double'}, ...
                                   {'real', 'nonnan', 'finite', 'nonempty', ...
                                    '2d', 'nonsparse'}, ...
                                   'ekfSLAM', ...
                                   'measurementCovariance');
                measCovar = diag(repmat(cast( ...
                    measurementCovariance,'like',obj.State), 1,numMeas));
            elseif all(size(measurementCovariance) == [sizeCovar, sizeCovar])
                validateattributes(measurementCovariance, ...
                                   {'single', 'double'}, ...
                                   {'real', 'nonnan', 'finite', 'nonempty', ...
                                    '2d', 'size', [sizeCovar, sizeCovar], ...
                                    'nonsparse'}, ...
                                   'ekfSLAM', ...
                                   'measurementCovariance');
                measCovar = cast(measurementCovariance,'like',obj.State);
            else
                measCovar = cast(measurementCovariance,'like',obj.State);
            end
        end
    end

    methods % Set methods
        function set.DataAssociationFcn(obj, value)
            validateattributes(value, {'function_handle'}, ...
                               {'nonempty'}, ...
                               'ekfSLAM', ...
                               'DataAssociationFcn');
            obj.DataAssociationFcn = value;
        end

        function set.InverseMeasurementFcn(obj, value)
            validateattributes(value, {'function_handle'}, ...
                               {'nonempty'}, ...
                               'ekfSLAM', ...
                               'InverseMeasurementFcn');

            obj.InverseMeasurementFcn = value;
        end

        function set.MaxNumLandmark(obj, value)
            validateattributes(value, {'single', 'double'}, ...
                               {'real', 'nonnan', 'finite', 'nonempty', ...
                                'positive', 'integer', 'scalar'}, ...
                               'ekfSLAM', ...
                               'MaxNumLandmark');
            obj.MaxNumLandmark = value;
        end

        function set.MaxAssociationRange(obj, value)
        % validate MaxAssociationRange
            validateattributes(value, {'single', 'double'}, ...
                               {'real', 'nonnan', 'nonempty', 'positive', 'scalar'}, ...
                               'ekfSLAM', ...
                               'MaxAssociationRange');
            % cast the value to class of State
            castedValue = castValue(obj, value);
            % set the value
            obj.MaxAssociationRange = castedValue;
        end
    end

    methods (Access = private)
        function augmentStateOnce(obj, landmark, landmarkCovariance)
            % Transform landmark from sensor space to Cartesian coordinate
            % and get the Jacobians for covariance transformation
            landmarkCart = zeros(0,2,'like',obj.State);
            dGdx = zeros(obj.LandmarkStateSize,obj.RobotStateSize,'like',obj.State);
            dGdz = zeros(obj.LandmarkStateSize,obj.LandmarkStateSize,'like',obj.State);
            if nargout(obj.InverseMeasurementFcn) == 3
                [landmarkCart, dGdx, dGdz] = obj.InverseMeasurementFcn( ...
                    obj.State(1:obj.RobotStateSize), ...
                    landmark);
            elseif nargout(obj.InverseMeasurementFcn) == 1
                landmarkCart = obj.InverseMeasurementFcn( ...
                    obj.State(1:obj.RobotStateSize), ...
                    landmark);
                % Use numerical perturbation to get Jacobians
                dGdx = matlabshared.tracking.internal.numericJacobian( ...
                    obj.InverseMeasurementFcn, ...
                    {obj.State(1:obj.RobotStateSize), ...
                     landmark}, 1);
                dGdz = matlabshared.tracking.internal.numericJacobian( ...
                    obj.InverseMeasurementFcn, ...
                    {obj.State(1:obj.RobotStateSize), ...
                     landmark}, 2);
            else
                coder.internal.error( ...
                    'nav:navalgs:ekfslam:invalidNumOutputsFromFcn', ...
                    'InverseMeasurementFcn');
            end

            prevStateLen = obj.CurStateLen;
            idx = prevStateLen + (1:obj.LandmarkStateSize);
            obj.CurStateLen = obj.CurStateLen + obj.LandmarkStateSize;
            if ~coder.target('MATLAB') && ...
                    (obj.CurStateLen > obj.MaxStateLen)
                coder.internal.error( ...
                    'nav:navalgs:ekfslam:MaxStateSizeReached');
            end

            % augment state
            obj.State(idx) = landmarkCart;

            % augment stateCovariance
            P = obj.StateCovariance;
            % landmark covariance
            P(idx,idx)= dGdx*P(1:obj.RobotStateSize,1:obj.RobotStateSize)*dGdx' + ...
                dGdz*landmarkCovariance*dGdz';
            % pose to landmark covariance correlation
            P(idx,1:obj.RobotStateSize)= dGdx*P(1:obj.RobotStateSize,1:obj.RobotStateSize);
            P(1:obj.RobotStateSize,idx)= P(idx,1:obj.RobotStateSize)';
            % map to landmark covariance correlation
            if prevStateLen>obj.RobotStateSize
                idxrem= (obj.RobotStateSize+1):prevStateLen;
                P(idx,idxrem)= dGdx*P(1:obj.RobotStateSize,idxrem);
                P(idxrem,idx)= P(idx,idxrem)';
            end
            obj.StateCovariance = P;
        end

        % Parse and Set Property-Value pair
        function parseSetPV(obj, varargin)
            defaultConstructorNames = {'State', ...
                                       'StateCovariance', ...
                                       'MaxNumLandmark', ...
                                       'StateTransitionFcn', ...
                                       'ProcessNoise', ...
                                       'MeasurementFcn', ...
                                       'InverseMeasurementFcn', ...
                                       'DataAssociationFcn', ...
                                       'MaxAssociationRange', ...
                                       'MaxNumPoseStored'};

            defaultConstructorValues  = {zeros(obj.RobotStateSize,1), ...
                                         [], ...
                                         inf, ...
                                         @nav.algs.velocityMotionModel, ...
                                         eye(2), ...
                                         @nav.algs.rangeBearingMeasurement, ...
                                         @nav.algs.rangeBearingInverseMeasurement, ...
                                         @nav.algs.associateMaxLikelihood, ...
                                         inf, ...
                                         inf};

            % Create property-value parser
            parser = robotics.core.internal.NameValueParser(...
                defaultConstructorNames, ...
                defaultConstructorValues);
            parse(parser, varargin{:});

            % Assign the properties based on the parsed values
            stateValue = parameterValue(parser, 'State');
            % validate the state
            validateState(obj,stateValue);
            % Set the datatype to be used
            classToUse = class(stateValue);

            % allocate State and StateCovariance
            if obj.IsMaxNumLandmarkDefined
                % pre-allocate State and StateCovariance if MaxNumLandmark
                % is defined
                obj.MaxStateLen = obj.RobotStateSize + ...
                    cast(obj.MaxNumLandmark,'like',obj.RobotStateSize)* ...
                    obj.LandmarkStateSize;

                obj.State = zeros(obj.MaxStateLen,1,classToUse);
                obj.StateCovariance = eye(obj.MaxStateLen,classToUse);
                % Define current state length
                obj.CurStateLen = size(stateValue(:),1);
            else
                % Define current state length
                obj.CurStateLen = size(stateValue(:),1);
                % pre-allocate State and StateCovariance
                obj.State = zeros(obj.CurStateLen,1,classToUse);
                obj.StateCovariance = eye(obj.CurStateLen,classToUse);
            end

            % Assign State
            obj.State(1:obj.CurStateLen) = cast(stateValue(:),classToUse);

            % Assign State Covariance
            stateCovarPassed = parameterValue(parser, 'StateCovariance');
            if ~isempty(stateCovarPassed)
                validateStateCovariance(obj, stateCovarPassed);
                obj.StateCovariance(1:obj.CurStateLen, ...
                                    1:obj.CurStateLen) = ...
                    cast(stateCovarPassed,classToUse);
            end

            % Assign State Transition Function
            obj.StateTransitionFcn = ...
                parameterValue(parser, 'StateTransitionFcn');
            % Assign Process Noise
            obj.ProcessNoise = parameterValue(parser, 'ProcessNoise');
            obj.ProcessNoiseLen = size(obj.ProcessNoise,1);
            % Assign Measurement Function
            obj.MeasurementFcn = parameterValue(parser, 'MeasurementFcn');
            % Assign Measurement Function
            obj.InverseMeasurementFcn = ...
                parameterValue(parser, 'InverseMeasurementFcn');
            % Assign Data Association Function
            obj.DataAssociationFcn = ...
                parameterValue(parser, 'DataAssociationFcn');
            % Assign Maximum Association range
            obj.MaxAssociationRange = ...
                parameterValue(parser, 'MaxAssociationRange');
        end

        % initialize poseHistory variables
        function initializePoseHistoryVariables(obj)
            % Reset the counters
            obj.PredictedPoseCounter = 0;
            obj.CorrectedPoseCounter = 0;

            if obj.IsMaxNumPoseStoredDefined
                % If MaxNumPoseStored is defined then create the vector of
                % respective size
                obj.StoredPredictedPose = ...
                    NaN(obj.MaxNumPoseStored, obj.RobotStateSize, ...
                    'like',obj.State);
                obj.StoredCorrectedPose = ...
                    NaN(obj.MaxNumPoseStored, obj.RobotStateSize, ...
                    'like',obj.State);
            else
                % If MaxNumPoseStored is not defined then create an empty
                % vector
                obj.StoredPredictedPose = NaN(0,3,'like',obj.State);
                obj.StoredCorrectedPose = NaN(0,3,'like',obj.State);
            end
        end

        function [predLM, jacPose, jacCart] = predictedMeasJacobian(obj,...
                                    landmarksWithinRange, measDimension)
            % get number of landmarks
            numLM = size(landmarksWithinRange,1);
            if nargout(obj.MeasurementFcn) == 3
                % if the measurementFcn gives Jacobians then directly
                % call the MeasurementJacobian to get all the values
                [predLM, jacPose, jacCart] = ...
                    obj.MeasurementFcn( ...
                    obj.State(1:obj.RobotStateSize), ...
                    landmarksWithinRange);
            elseif nargout(obj.MeasurementFcn) == 1
                % if the measurementFcn doesn't give Jacobians then
                % use numerical perturbations to get Jacobians
                predLM = obj.MeasurementFcn( ...
                    obj.State(1:obj.RobotStateSize), ...
                    landmarksWithinRange);
                % Use numerical perturbation to get Jacobians
                jacPose = zeros(obj.LandmarkStateSize*numLM, ...
                    obj.RobotStateSize);
                jacCart = zeros(obj.LandmarkStateSize*numLM, measDimension);
                for i = 1:numLM
                    index = 2*i + [-1,0];
                    jacPose(index,:) = matlabshared.tracking.internal.numericJacobian( ...
                        obj.MeasurementFcn, ...
                        {obj.State(1:obj.RobotStateSize), ...
                        landmarksWithinRange(i,:)}, 1);
                    jacCart(index,:) = matlabshared.tracking.internal.numericJacobian( ...
                        obj.MeasurementFcn, ...
                        {obj.State(1:obj.RobotStateSize), ...
                        landmarksWithinRange(i,:)}, 2);
                end
            else
                coder.internal.error( ...
                    'nav:navalgs:ekfslam:invalidNumOutputsFromFcn', ...
                    'MeasurementFcn');
            end
        end
    end
end
