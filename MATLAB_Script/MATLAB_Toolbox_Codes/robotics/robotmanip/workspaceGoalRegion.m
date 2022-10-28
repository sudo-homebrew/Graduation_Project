classdef workspaceGoalRegion
%WORKSPACEGOALREGION Define workspace region of end-effector goal poses
%   The WORKSPACEGOALREGION object defines a region for valid end-effector 
%   goal poses. To sample poses within the bounds of the goal region, use 
%   the SAMPLE function. You can also visualize the bounds you define using 
%   the SHOW function.
%
%   The object is typically used with rapidly-exploring random tree (RRT) 
%   planners like the manipulatorRRT object. 
%
%  The key elements of the workspace goal region are defined as properties:
%
%   - ReferencePose:            Pose of the reference frame 
%                               in the world frame. The bounds and offset
%                               pose are relative to this frame.
%
%   - EndEffectorOffsetPose:    Offset pose applied to any pose sampled in 
%                               the reference frame as a 4-by-4 homogeneous
%                               transformation matrix.
%
%   - Bounds:                   Bounds of the region as a 6-by-2 matrix, 
%                               [min max], where the first three rows are 
%                               the bounds on XYZ-position, and the last
%                               three are the bounds on ZYX Euler angle
%                               orientation, respectively.
%
%   WGR = workspaceGoalRegion(ENDEFFECTORNAME) creates a default workspace 
%   goal region object for the specified end-effector name.
%
%   WGR = workspaceGoalRegion(___,Name,Value) sets properties using
%   name-value pairs. For example, 
%   workSpaceGoalRegion("endEffector","Bounds",LIMITS) creates a workspace 
%   goal region with the bounds specified in LIMITS.
%
%   WORKSPACEGOALREGION Properties:
%      EndEffectorName       - Name of the end effector
%      ReferencePose         - Pose of the reference frame
%      EndEffectorOffsetPose - End-effector offset pose applied to poses 
%                               sampled in reference frame
%      Bounds                - Position and orientation bounds
%
%   WORKSPACEGOALREGION Methods:
%       sample              - Sample an end-effector pose in world frame
%       show                - Visualize workspace bounds, reference frame,
%                               and offset frame
%   Example:
%       % Initialize the workspace goal region for the "sampleEE" end-effector. 
%       wgr = workspaceGoalRegion("sampleEE");
%
%       % Define the bounds with respect to the reference frame.
%       wgr.Bounds = [-0.2 0.2; -0.1 0.1; -0.3 0.3; ...  % XYZ position
%                     -pi/2 pi/2;, 0 0; 0 0];          % ZYX Euler angle
%
%       % Visualize the workspaceGoalRegion.
%       show(wgr)
%
%   References:
%           [1]D. Berenson, S. Srinivasa, D. Ferguson, A. Collet and J. Kuffner,
%           "Manipulation planning with Workspace Goal Regions," 2009 IEEE
%           International Conference on Robotics and Automation, Kobe, 2009, pp.
%           618-624
%
%   See also manipulatorRRT

%   Copyright 2020 The MathWorks, Inc.

%#codegen


    properties(SetAccess=immutable)
        %EndEffectorName Name of the end effector
        %   Name of the end effector, specified as a string
        %   scalar.
        %
        %   The EndEffectorName property is immutable and must be set on
        %   construction.
        EndEffectorName
    end

    properties
        %ReferencePose Pose of the reference frame
        %   Pose of the reference frame in the world frame, specifed as a 
        %   homogeneous transform with respect to the world frame. Poses
        %   are sampled around this pose based on the specified bounds.
        %
        %   Default: eye(4)
        ReferencePose = eye(4);

        %EndEffectorOffsetPose End-effector offset pose applied to poses sampled in reference frame
        %   End-effector offset pose applied to poses sampled in reference 
        %   frame, specifed as a homogeneous transform with respect to the 
        %   reference frame. This offset is applied to all poses sampled
        %   and typically accounts for a grasping offset.
        %
        %   Default: eye(4)
        EndEffectorOffsetPose = eye(4);

        %Bounds Position and orientation bounds
        %   Position and orientation bounds on pose samples, specified in the
        %   reference frame as [min max] column vectors.  The first three rows
        %   are the XYZ-position bounds. The last three rows are the orientation
        %   bounds, which are specified as intrinsic ZYX Euler angles,
        %   respectively. Orientation is based on the right-hand rule with
        %   counter-clockwise rotations about the respective axes being
        %   positive. During sampling, a value is uniformly sampled within these
        %   bounds to obtain a sample pose in the reference frame.
        %
        %   Default: zeros(6, 2)
        Bounds = zeros(6 ,2);
    end

    properties(Access=private, Constant)
        %EndEffectorNamePropertyName
        EndEffectorNameCtorArgName = 'endEffectorName';

        %ReferencePosePropertyName
        ReferencePosePropertyName = 'ReferencePose';

        %EndEffectorOffsetPosePropertyName
        EndEffectorOffsetPosePropertyName = 'EndEffectorOffsetPose';

        %BoundsPropertyName
        BoundsPropertyName= 'Bounds';

        %EulerAngleConvention Euler angle convention used to obtain rotation transforms
        %   The convention is used in sampling a random pose vector within the
        %   bounds
        EulerAngleConvention = "ZYX";

        %BoundsSize
        BoundsSize = [6, 2];

        %ClassName
        ClassName = "workspaceGoalRegion";
    end

    properties(Access=private)
        %Visual Visual object of workspace goal region
        Visual
    end

    methods
        function obj = workspaceGoalRegion(endEffectorName, varargin)
        %WORKSPACEGOALREGION Constructor
        % Validate EndEffectorName as it is immutable
            obj.validateEndEffectorName(endEffectorName);
            obj.EndEffectorName = endEffectorName;

            %Construct a name-value pair parser for the properties
            names = {
                obj.ReferencePosePropertyName, ...
                obj.EndEffectorOffsetPosePropertyName, ...
                obj.BoundsPropertyName ...
                    };
            defaults = {
                obj.ReferencePose, ...
                obj.EndEffectorOffsetPose, ...
                obj.Bounds ...
                       };
            parser = robotics.core.internal.NameValueParser(names, defaults);

            % Parse the input and assign properties
            parser.parse(varargin{:});
            obj.ReferencePose = ...
                parser.parameterValue(obj.ReferencePosePropertyName);
            obj.EndEffectorOffsetPose = ...
                parser.parameterValue(obj.EndEffectorOffsetPosePropertyName);
            obj.Bounds = parser.parameterValue(obj.BoundsPropertyName);

            % Construct the visual
            obj.Visual = ...
                robotics.manip.internal.workspaceGoalRegion.WGRVisual();
        end

        function eePose = sample(obj, varargin)
        %sample Sample end-effector poses in world frame
        %   EEPOSE = sample(WGR) samples an end-effector pose in the world
        %   frame as a homogeneous transformation matrix.
        %
        %   EEPOSE = sample(WGR, NUMSAMPLES) samples multiple poses based 
        %   on the NUMSAMPLES input. EEPOSE is returned as a 3-D array of 
        %   homogeneous transforms (4-by-4-by-NUMSAMPLES).
        %
        %   The function returns a pose uniformly-sampled within the Bounds 
        %   property and applies the following transformations based on the 
        %   ReferencePose and EndEffectorOffsetPose properties:
        %
        %       EEPOSE = ReferencePose * TSAMPLE * EndEffectorOffsetPose;

            numSamples = 1;
            if(nargin == 2)
                numSamples = varargin{1};
                robotics.internal.validation.validatePositiveIntegerScalar(...
                    numSamples, ...
                    'sample', ...
                    'numSamples');
            end
            samplePoseVector = obj.sampleRandomPoseVectorWithinBounds(numSamples);
            samplePose = obj.convertPoseVectorToTransform(samplePoseVector);

            % Transform the samplePose which is in the reference frame to
            % the world frame and apply the end-effector offset pose
            for i = 1:numSamples
                samplePose(:, :, i) = ...
                    obj.ReferencePose * samplePose(:, :, i) * obj.EndEffectorOffsetPose;
            end
            eePose = samplePose;
        end

        function ax = show(obj, varargin)
        %show Visualize workspace bounds, reference frame, and offset frame
        %   AX = show(WGR) plots the position and orientation bounds of the
        %   workspace goal region. The reference frame and end-effector offset
        %   frame are also shown. 
        %
        %   AX = show(WGR,"Parent",axesHandle) specifies the parent axes
        %   to plot the workspace goal region.

        % show is not supported in codegen
            if(~coder.target('MATLAB'))
                coder.internal.errorIf(~coder.target('MATLAB'),...
                                       'shared_robotics:robotcore:utils:CodegenNotSupported', 'show');
            end
            p = inputParser;
            parentAx = [];
            p.addParameter('Parent', parentAx, ...
                           @(x)validateattributes(x, {'matlab.graphics.axis.Axes'}, {'nonempty', 'scalar'}, ...
                                                  'show', 'Parent'))

            p.parse(varargin{:});
            parentAx = p.Results.Parent;
            if(isempty(p.Results.Parent))
                parentAx = newplot;
            end

            % Update the visual properties
            obj.Visual.Bounds = obj.Bounds;
            obj.Visual.EndEffectorOffsetPose = obj.EndEffectorOffsetPose;
            obj.Visual.ReferencePose = obj.ReferencePose;

            show(obj.Visual, parentAx);

            % Optionally, output the parent axes
            if(nargout == 1)
                ax = parentAx;
            end
        end
    end


    methods
        function obj = set.ReferencePose(obj, val)
        %set.ReferencePose
            obj.validateHomogeneousTransform(val, obj.ReferencePosePropertyName);
            obj.ReferencePose = val;
        end

        function obj = set.EndEffectorOffsetPose(obj, val)
        %set.EndEffectorOffsetPose
            obj.validateHomogeneousTransform(val, obj.EndEffectorOffsetPosePropertyName);
            obj.EndEffectorOffsetPose = val;
        end

        function obj = set.Bounds(obj, val)
        %set.Bounds
            validateattributes(val, {'double'}, ...
                               {'nonnan', 'real', 'nonempty', 'size', obj.BoundsSize, 'finite'},...
                               obj.ClassName, obj.BoundsPropertyName);

            % Validate that the bounds along each degree of freedom are
            % non-decreasing
            for i = 1 : obj.BoundsSize(1)
                validateattributes(val(i, :), {'double'}, ...
                                   {'nonnan', 'real', 'nonempty', 'finite', 'nondecreasing'}, ...
                                   obj.ClassName,  obj.BoundsPropertyName);

            end
            obj.Bounds = val;
        end

    end

    methods(Access=private)
        function samplePoseVector = sampleRandomPoseVectorWithinBounds(obj, numSamples)
        %sampleRandomPoseVector Uniformly sample a pose vector in the lower and upper bounds
        %   A pose vector is of the form [trX, trY, trZ, eulRotZ, eulRotY,
        %   eulRotX], where "tr" denotes translation and "eulRot" denotes an
        %   Euler angle rotation which follows the EulerAngleConvention
        %   property.  The sampling is based on the cited reference [1], wherein
        %   a pose vector is uniformly sampled within the bounds, and a
        %   corresponding transform is used to obtain the end-effector pose.
        %   Note that the resulting transform will be uniform in translation
        %   bounds, but not in rotation as sampling uniformly in euler angles
        %   doesn't result in uniform rotations.
            randomRatio = rand(obj.BoundsSize(1), numSamples);
            samplePoseVector = repmat(obj.Bounds(:, 1)', numSamples, 1) + ...
                randomRatio' .* repmat(obj.Bounds(:, 2)' - obj.Bounds(:, 1)', numSamples, 1);
        end

        function tform = convertPoseVectorToTransform(obj, poseVector)
        %convertPoseVectorToTransform
            tform = eul2tform(poseVector(:, 4:end),  obj.EulerAngleConvention);
            tform(1:3, end, :) = poseVector(:, 1:3)';
        end

        function validateHomogeneousTransform(obj, val, propertyName)
        %validateHomogeneousTransform
            robotics.internal.validation.validateHomogeneousTransform(...
                val, ...
                obj.ClassName, ...
                propertyName, ...
                'nonnan', 'real', 'nonempty', 'finite');
        end

        function validateEndEffectorName(obj, val)
        %set.EndEffectorName
            validateattributes(val, ...
                               {'string', 'char'}, ...
                               {'nonempty','scalartext'}, ...
                               obj.ClassName, ...
                               'endEffectorName');
        end
    end
end
