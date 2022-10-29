classdef (StrictDefaults) odometryMotionModel < matlab.System
%ODOMETRYMOTIONMODEL Create odometry motion model
%   ODOMETRYMOTIONMODEL creates an odometry motion model object for
%   differential drive robots. This object contains specific motion
%   model parameters. You can use this object to specify the motion
%   model parameters in the monteCarloLocalization object.
%
%   This motion model assumes that the robot makes pure rotation and
%   translation motions to travel from one location to the other. The
%   elements of noise parameter refers to weights of standard deviation
%   for the Gaussian noise applied to robot motion.
%
%   OMM = odometryMotionModel creates an odometry motion model
%   object for differential drive robots, with a default set of noise
%   parameters.
%
%   OMM = odometryMotionModel('PropertyName', PropertyValue)
%   creates an odometry motion model object for differential drive
%   robots with the specified property value.
%
%   STEP method syntax:
%
%   CURRENTPOSES = OMM(PREVIOUSPOSES, ODOMETRYPOSE) OR
%   CURRENTPOSES = STEP(OMM, PREVIOUSPOSES, ODOMETRYPOSE) returns the
%   CURRENTPOSES by propagating the incoming PREVIOUSPOSES, using a
%   sampling-based odometry motion model which uses the difference
%   between the specified ODOMETRYPOSE and the property value
%   LastOdometryPose on the OMM object.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj, x) and y = obj(x) are
%   equivalent.
%
%   odometryMotionModel properties:
%       Noise               - Noise specified as an array of weights of
%                             standard deviation
%       Type                - Odometry motion model type (Read-only)
%       LastOdometryPose    - Last odometry readings (Read-only)
%
%   odometryMotionModel methods:
%       step                     - Compute the next pose(s) for the
%                                  specified previous pose(s) for any
%                                  number of samples
%       showNoiseDistribution    - Display the affect of noise parameters
%
%   Example:
%
%       % Create odometry motion model object
%       motionModel = odometryMotionModel;
%
%       % To visualize the effect of existing noise parameters
%       showNoiseDistribution(motionModel);
%
%       % To visualize the effect of existing noise parameters, with
%       % custom odometry change and number of samples
%       showNoiseDistribution(motionModel, ...
%           'OdometryPoseChange', [0.25 0.25 0.5], ...
%           'NumSamples', 1000);
%
%       % Create inputs for STEP call
%       previousPoses =  rand(10, 3);
%       currentOdom = [ 0.1 0.1 0.1 ];
%
%       % First STEP call to instantiate the object, and returns same
%       % poses
%       currentPoses = motionModel(previousPoses, currentOdom);
%
%       % Second STEP call to with an updated odometry, updates the
%       % particle poses
%       currentOdom = currentOdom + [0.1 0.1 0.05];
%       currentPoses = motionModel(previousPoses, currentOdom);
%
%       % Another STEP call using object notation directly
%       currentOdom = currentOdom + [0.1 0.1 0.05];
%       currentPoses = motionModel(previousPoses, currentOdom);
%
%
%   See also monteCarloLocalization.

%   Copyright 2015-2021 The MathWorks, Inc.
%
%   References:
%
%   [1] S. Thrun, W. Burgard and D. Fox, Probabilistic Robotics.
%   Cambridge, MA: MIT Press, 2005.
%   [2] A. Howard (2003, Feb, 6) ROS-Planning/Navigation/AMCL Repository [Online]
%   Available: https://github.com/ros-planning/navigation/blob/1.13.0/amcl/src/amcl/sensors/amcl_odom.cpp

%#codegen

    properties
        %Noise Noise specified as an array of weights of standard deviation
        %   This property represents the weights of standard deviation for
        %   the Gaussian noise applied to robot motion. The Gaussian noise
        %   is a function of pose difference.
        %   Noise(1) - Rotational error due to rotational motion
        %   Noise(2) - Rotational error due to translation motion
        %   Noise(3) - Translation error due to translation motion
        %   Noise(4) - Translation error due to rotational motion
        %
        %   Default: [0.2 0.2 0.2 0.2]
        Noise = [0.2 0.2 0.2 0.2]
    end

    properties (SetAccess = private)
        %Type Type of the odometry motion model being used
        %   This read-only property indicates the type of odometry motion
        %   model being used by the object.
        %
        %   Default: 'DifferentialDrive'
        Type = 'DifferentialDrive'


        %LastOdometryPose - stores the last odometry values
        %
        % Default: [NaN, NaN, NaN]
        LastOdometryPose = [NaN NaN NaN];
    end

    methods
        function obj = odometryMotionModel(varargin)
        %odometryMotionModel constructor
            setProperties(obj,nargin,varargin{:}, 'Noise');
        end

        function set.Noise(obj, noise)
            validateattributes(noise, {'double'}, ...
                               {'real', 'nonnegative', 'finite', 'vector', 'numel', 4}, 'Noise', 'noise');
            obj.Noise = noise(:).';
        end
    end

    methods (Access = protected)

        function resetImpl(obj)
            obj.LastOdometryPose = [nan nan nan];
        end

        function releaseImpl(obj)
            obj.LastOdometryPose = [nan nan nan];
        end

        function currentPoses = stepImpl(obj, previousPoses, odometryPose)
        %STEP Move the samples with PREVIOUSPOSES, with the incoming
        %   ODOMETRYPOSE using the sampling-based DifferentialDrive
        %   odometry motion model. The STEP method outputs the
        %   CURRENTPOSES by computing the change in Odometry, using
        %   ODOMETRYPOSE and the value of the LastOdometryPose property
        %   on the object.

        % validate previousPoses
            validateattributes(previousPoses, {'double'}, ...
                               {'real', 'finite', '2d', 'ncols', 3, 'nonempty', 'nonnan'}, 'step', 'previousPoses');

            % validate odometry
            obj.validateOdometryPose(odometryPose, 'step', 'odometryPose');

            % Convert odometryPose into row vector
            odometryPose = odometryPose(:)';

            % First STEP call
            if any(isnan(obj.LastOdometryPose))

                % As the object is getting initialized in the first step
                % call, return the same set of previousPoses
                currentPoses = previousPoses;

            else
                currentPoses = nav.algs.internal.motionModelUpdate(...
                    previousPoses, obj.LastOdometryPose, odometryPose, obj.Noise);
            end

            % Update stored odometry
            obj.LastOdometryPose = odometryPose;
        end

        function num = getNumInputsImpl(~)
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 1;
        end

        function flag = isInputSizeMutableImpl(~, ~)
            flag = true;
        end
        
    end

    methods
        function ax = showNoiseDistribution(obj, varargin)
        %showNoiseDistribution - Shows the noise distribution in a figure
        %   showNoiseDistribution(MOTIONMODEL) shows the noise
        %   distribution for a default odometry pose update, number of
        %   samples and the current noise parameters on the object.
        %
        %   AX = showNoiseDistribution(MOTIONMODEL) returns handle of
        %   the axes used by the show method.
        %
        %   showNoiseDistribution(MOTIONMODEL, Name, Value) provides
        %   additional options specified by one or more Name-Value pair
        %   arguments. Name must appear inside single quotes (''). You
        %   can specify several Name-Value pair arguments in any order:
        %
        %       'OdometryPoseChange'    - A numeric vector of 1x3
        %                               element (i.e., x, ,y, theta
        %                               positions), describing the
        %                               odometry update (i.e., change
        %                               in odometry) of the robot.
        %       'NumSamples'            - A number specifying the
        %                               number of particles to use for
        %                               displaying
        %       'Parent'                - Axes handle to display the
        %                               plot on an existing figure
        %
        %   Example:
        %
        %       % Create odometry motion model object
        %       motionModel = odometryMotionModel;
        %
        %       % To visualize the effect of existing noise parameters
        %       showNoiseDistribution(motionModel);
        %
        %       % To visualize the effect of existing noise parameters, with
        %       % custom odometry change and number of samples
        %       showNoiseDistribution(motionModel, ...
        %           'OdometryPoseChange', [0.25 0.25 0.5], ...
        %           'NumSamples', 1000);

            if ~coder.target('MATLAB')
                %  Code generation

                % Always throw error when calling showNoiseDistribution in
                % generated code
                coder.internal.errorIf(~coder.target('MATLAB'),...
                                       'nav:navalgs:motionModel:GraphicsSupportCodegen','showNoiseDistribution');
            end

            % Define defaults
            startPose = [0 0 0];
            odometryPoseChange = [.3 .25 1];
            numSamples = 1000;
            noiseParameters = obj.Noise;

            % Parse optional inputs
            p = inputParser;
            p.addParameter('OdometryPoseChange', odometryPoseChange, ...
                           @(x)obj.validateOdometryPose(x, 'showNoiseDistribution', 'OdometryPoseChange'));
            p.addParameter('NumSamples', numSamples, @obj.validateNumSamples);
            p.addParameter('Parent', [], @robotics.internal.validation.validateAxesUIAxesHandle);
            p.parse(varargin{:});

            % Re-assign optional inputs
            odometryPoseChange = p.Results.OdometryPoseChange;
            numSamples = p.Results.NumSamples;
            parentAxHandle = p.Results.Parent;

            % Preserve the hold state of axes
            holdState = false;
            if ~isempty(parentAxHandle)
                holdState = ishold(parentAxHandle);
            end

            % Create particles
            oldP = repmat(startPose, numSamples, 1);
            oldOdom = [0 0 0];

            % update motion
            endPose = startPose + odometryPoseChange;
            newOdom = oldOdom + odometryPoseChange;

            % Create figure OR use the existing figure
            if isempty(parentAxHandle)
                axHandle = newplot;
            else
                axHandle = newplot(parentAxHandle);
            end
            axHandle.Parent.NumberTitle = 'off';
            axHandle.Parent.Name = message('nav:navalgs:motionModel:FigureName').getString;
            hold(axHandle, 'on');

            % Create circular robots on the figure window
            robotPoses = [startPose; endPose];
            robotHandle = obj.plotRobot(axHandle, robotPoses(:,1), robotPoses(:,2), robotPoses(:,3));

            % Compute new set of particles and plot them
            newP = nav.algs.internal.motionModelUpdate(oldP, ...
                                                       oldOdom, ...
                                                       newOdom, ...
                                                       noiseParameters);
            hparticles = plot(axHandle, newP(:,1), newP(:,2), '.r');

            % Set figure properties
            title(axHandle, ...
                  message('nav:navalgs:motionModel:FigureTitle', num2str(noiseParameters)).getString);
            xlabel(axHandle, message('nav:navalgs:motionModel:FigureXLabel').getString);
            ylabel(axHandle, message('nav:navalgs:motionModel:FigureYLabel').getString);
            axis(axHandle, 'equal');
            legend(axHandle, [robotHandle(1) robotHandle(2) hparticles],...
                   {message('nav:navalgs:motionModel:FigureLegendInitialPose').getString, ...
                    message('nav:navalgs:motionModel:FigureLegendFinalPose').getString, ...
                    message('nav:navalgs:motionModel:FigureLegendSamples').getString}, ...
                   'Location', 'northwest');

            % Reset hold state
            if holdState
                toggleStr = 'on';
            else
                toggleStr = 'off';
            end
            hold(axHandle, toggleStr);

            % Return axes handle is queried for
            if nargout > 0
                ax = axHandle;
            end

        end
    end

    methods (Static, Access = private)

        function validateOdometryPose(odomPose, methodNameStr, variableNameStr)
        %validateOdometryPose - Validate input for
        % step and showNoiseDistribution method
            validateattributes(odomPose, {'double'}, ...
                               {'real', 'nonnan', 'finite', 'vector', 'numel', 3}, methodNameStr, variableNameStr);
        end

        function validateNumSamples(numSamples)
        %validateNumSamples - Validate input for showNoiseDistribution
        %method
            validateattributes(numSamples, {'numeric'}, ...
                               {'scalar', 'nonnan', 'finite','nonnegative', 'real', 'nonempty'}, 'showNoiseDistribution', 'numSamples');
        end

        function robotHandle = plotRobot(ax, x, y, th)
        %plotRobot - This method plots the robot on the figure. The
        %robot consists of a circle with an orientation.

        % define robot handle info
            robotHandle = zeros(1,2);
            % robot info
            rad = sqrt((x(1)-x(2))^2 + (y(1)-y(2))^2)/5;
            angles = 0:0.1:2*pi;

            % robot 1
            % circle
            xcirc1 = x(1) + rad * cos(angles);
            ycirc1 = y(1) + rad * sin(angles);
            % steering
            xsteer1 = x(1) + rad * cos(th(1));
            ysteer1 = y(1) + rad * sin(th(1));

            % robot 2
            % circle
            xcirc2 = x(2) + rad * cos(angles);
            ycirc2 = y(2) + rad * sin(angles);
            % steering
            xsteer2 = x(2) + rad * cos(th(2));
            ysteer2 = y(2) + rad * sin(th(2));

            % plotting
            hold(ax, 'on');

            % plot start robot with steering
            robotHandle(1) = plot(ax, ...
                                  [x(1) xsteer1 nan xcirc1 xcirc1(1)], ...
                                  [y(1) ysteer1 nan ycirc1 ycirc1(1)], ...
                                  'Color', 'b','LineWidth', 2);

            % plot end robot with steering
            robotHandle(2) = plot(ax, ...
                                  [x(2) xsteer2 nan xcirc2 xcirc2(1)], ...
                                  [y(2) ysteer2 nan ycirc2 ycirc2(1)], ...
                                  'Color', 'm', 'LineWidth', 2);

            % compute connection
            xsteer1 = x(1) + (2*rad) * cos(th(1));
            ysteer1 = y(1) + (2*rad) * sin(th(1));

            xsteer2 = x(2) + (2*rad) * cos(th(2));
            ysteer2 = y(2) + (2*rad) * sin(th(2));

            % plot connection line between robot with arcs
            robotHandle(3) = plot(ax, ...
                                  [xsteer1 x(1)  x(2) xsteer2], ...
                                  [ysteer1 y(1) y(2) ysteer2]);

            axis(ax, 'equal');
        end
    end

end
