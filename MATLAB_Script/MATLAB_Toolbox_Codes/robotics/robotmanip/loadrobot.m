function [robot,robotData] = loadrobot(robotName,varargin)
%loadrobot Load robot model as rigidBodyTree.
%   ROBOT = loadrobot(ROBOTNAME) loads a robot model as a
%   rigidBodyTree object based on the specified ROBOTNAME.
%
%   [ROBOT, ROBOTDATA] = loadrobot(ROBOTNAME) returns additional
%   information about the source of the robot model. ROBOTDATA is a struct
%   with the fields:
%
%   RobotName - Name of the returned robot model.
%
%   FilePath  - File path of Unified Robot Description Format (URDF) file
%               that is used to create the RBT of the specified robot.
%
%   Source    - Robot description files are downloaded from this URL.
%
%   Version   - Version number of the robot model.
%
%   The following properties are relevant to only some robots. Non-relevant
%   properties for a robot are empty.
%
%   TrackWidth - Distance between wheels on axle specified in meters
%
%   WheelRadius - Vehicle wheel radius specified in meters
%
%   WheelBase - Distance between front and rear axles specified in meters
%
%   MaxTranslationalVelocity - Vehicle maximum linear velocity specified in
%   meters/s
%
%   MaxRotationalVelocity - Vehicle maximum angular velocity specified in
%   rad/s
%
%   DriveType - All robots are modeled as having fixed base, but this property
%   describes the actual motion type of the robot base.
%               FixedBase - robots with fixed base
%               Differential-Drive - robots with differential drive mobile base
%               Omni-Wheel - robots with omni wheel mobile base
%
%   MobileBaseMotionModel - kinematic motion model of the mobile base
%
%   ManipulatorMotionModel - Joint-Space Motion Model of the Manipulator
%
%   [ROBOT, ROBOTDATA] = loadrobot(______, Name, Value) provides additional
%   options specified by Name-Value pair arguments. Available parameter
%   names:
%
%   "DataFormat" - Data format for the RigidBodyTree object, specified as
%                  "struct", "row", or "column". To use dynamics methods,
%                  you must use either "row" or "column". The default data
%                  format is "struct".
%
%   "Gravity"    - Gravity for the RigidBodyTree object. The default value
%                  is [0 0 0](m/s^2)
%
%   "Version"    - Version number of the robot model, specified as a numeric
%                  value. The default value is 1.
%
%   Example:
%
%      % Load the kinova Gen3 robot version 1 as a rigid body tree.
%      [gen3, gen3RobotData] = loadrobot("kinovaGen3", "Version", 1);
%
%      %Visualize the rigid body tree.
%      show(gen3);
%
%  See also importrobot, rigidBodyTree

% Copyright 2019-2021 The MathWorks, Inc.

%#codegen

    narginchk(1,7);

    % Double validation of robotName for clear error message
    validateattributes(robotName,{'char','string'},{'nonempty','scalartext'},'loadrobot','robotName');

    if isempty(coder.target)
        robotName = validatestring(robotName, robotics.manip.internal.RobotList.allRobots, 'loadrobot', 'robotName');

        parser = inputParser;
        addParameter(parser, 'Gravity', [0 0 0], @(x)validateattributes(x,{'numeric'}, {'real', 'nonsparse',...
                                                                                        'finite','nonempty','numel',3,'vector'}, 'loadrobot', 'Gravity'));
        addParameter(parser, 'DataFormat', 'struct');
        addParameter(parser, 'Version', 1);

        parse(parser, varargin{:});
        userInput = parser.Results;

        % Double validation of DataFormat for clear error message
        validateattributes(userInput.DataFormat,{'char','string'},{'nonempty','scalartext'},'loadrobot','DataFormat');
        userInput.DataFormat = validatestring(userInput.DataFormat, {'struct', 'row', 'column'}, 'loadrobot', 'DataFormat');

        % Load the pre-generated RBTs
        % Fetch the number of versions and the corresponding MAT file of
        % the robot
        [~, matFileName] = robotics.manip.internal.RobotList.getNumVersionsAndMATFileName(robotName, userInput.Version);

        % Load the MAT file corresponding to the version
        fileToLoad = matFileName;
        S=load(strcat(fileToLoad,'.mat'), 'robot', 'robotData', 'inertialFlag');

        robot = S.robot;
        robotData = S.robotData;

        % Set the DataFormat parameter
        robot.DataFormat = userInput.DataFormat;

        hasInertia = ~S.inertialFlag;
        robotData.HasBodyInertias = hasInertia;

        if (robotData.Manipulator)&&(hasInertia)
            robotics.manip.internal.JointSpaceMotionModelGravityAndDataFormatUpdator.update(...
                robotData.ManipulatorMotionModel,userInput.Gravity,userInput.DataFormat);
        end
        robotData.FilePath = which(strcat(fileToLoad,'.urdf'));
        robotData = rmfield(robotData,'Mobile');
        robotData = rmfield(robotData,'Manipulator');

        % Set the gravity parameter
        robot.Gravity = userInput.Gravity;
    else
        % Throw an error if the user expects output of robotdata
        coder.internal.assert(nargout <= 1, 'robotics:robotmanip:rigidbodytree:CodegenRobotData');

        % Parse inputs
        params = struct('Gravity', uint32([0, 0, 0]), ...
                        'DataFormat', char(0), ...
                        'Version', uint32(1));

        pstruct = coder.internal.parseParameterInputs(params, [], varargin{:});
        Gravity = coder.internal.getParameterValue(pstruct.Gravity, [0, 0, 0], varargin{:});

        DataFormat = coder.internal.getParameterValue(pstruct.DataFormat, 'struct', varargin{:});

        Version = coder.internal.getParameterValue(pstruct.Version, 1, varargin{:});

        % For coder, make an extrinsic call to internal extractStructFromRBT that returns a
        % codegen compatible RBT struct
        rbt = coder.const(@feval, 'robotics.manip.internal.populateRBTStructFromFunction', ...
                          'loadrobot', robotName, 'DataFormat', DataFormat, 'Version', Version);

        % Convert RBT struct back to rigidBodyTree object
        robot = rigidBodyTree(robotics.manip.internal.RigidBodyTree(rbt.NumBodies, rbt));

        % Set the gravity value
        robot.Gravity = Gravity;

        % Output empty robotdata
        robotData=[];
    end
end
