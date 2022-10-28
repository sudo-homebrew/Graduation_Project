function [updatedPose, jacPose, jacVel] = velocityMotionModel(pose, velocity, timeStep)
%VELOCITYMOTIONMODEL Create constant velocity motion model
%
%   This motion model assumes that the robot moves at a constant linear
%   velocity and rotates with a constant angular velocity for a time-step.
%
%   UPDATEDPOSE = VELOCITYMOTIONMODEL(POSE, VELOCITY, TIMESTEP) creates a
%   constant velocity motion model to calculate the pose at the next
%   time-step based on the current pose, POSE, the velocity, VELOCITY,
%   and the time interval, TIMESTEP.
%
%   [UPDATEDPOSE, JACPOSE, JACVEL] = VELOCITYMOTIONMODEL(POSE, VELOCITY, TIMESTEP)
%   returns the Jacobian matrices of the velocity motion model with respect
%   to current pose, JACPOSE, and with respect to velocity, JACVEL, along
%   with the updated pose at the next time-step.
%
%   Input Arguments:
%
%   POSE        - Current pose of the vehicle in the local navigation
%                 coordinate system specified as a three-element vector of
%                 real values. The vector is of form [X Y Yaw]. X and Y
%                 specify the position in meters. Yaw specifies the
%                 orientation in radians.
%
%   VELOCITY    - Current command velocity of the vehicle in the vehicle
%                 body coordinate system specified as a two-element vector
%                 of real values. The vector is of the form [V W]. V
%                 specifies the linear velocity of the vehicle in meters
%                 per second. W specifies the angular velocity in radians
%                 per second.
%
%   TIMESTEP    - Time step in which vehicle moves specified as a scalar of
%                 real values.
%
%   Output Arguments:
%
%   UPDATEDPOSE - Updated pose of the vehicle in the local navigation
%                 coordinate system returned as a three-element vector of
%                 real values. The vector is of form [X Y Yaw]. X and Y
%                 specify the position in meters. Yaw specifies the
%                 orientation in radians.
%
%   JACPOSE     - Jacobian of the velocity motion model with respect to
%                 current pose, returned as a 3-by-3 matrix.
%
%   JACVEL      - Jacobian of the velocity motion model with respect to
%                 velocity, returned as a 3-by-2 matrix.
%
%   Either single or double datatypes are supported for the inputs to
%   VELOCITYMOTIONMODEL. Outputs have the same datatype as the pose.
%
%   % Example: Calculate the pose and Jacobians at the next time step
%
%   % Create input arguments
%   pose     = [1; -2; 0.1];          % current pose
%   velocity = [1, 0];                % velocity
%   timeStep = 0.25;                  % time step
%
%   % Calculate the pose and Jacobians
%   [updatedPose, jacPose, jacVel] = nav.algs.velocityMotionModel(pose, ...
%                                                   velocity, timeStep);
%
%   See also ekfSLAM

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    % Check that number of arguments is exactly 3
    narginchk(3, 3);

    validateattributes(pose, {'single', 'double'}, ...
                       {'real', 'nonnan', 'nonempty','finite', ...
                       'vector', 'numel', 3}, ...
                       'nav.algs.velocityMotionModel', ...
                       'pose');

    validateattributes(velocity, {'single', 'double'}, ...
                       {'real', 'nonnan', 'nonempty', 'finite', ...
                       'vector', 'numel', 2}, ...
                       'nav.algs.velocityMotionModel', ...
                       'velocity');

    validateattributes(timeStep, {'single', 'double'}, ...
                       {'real', 'nonnan', 'nonempty', 'finite', ...
                       'scalar', 'nonnegative'}, ...
                       'nav.algs.velocityMotionModel', ...
                       'timeStep');

    % Data type would be determined by the pose argument.
    % Rules for data classes:
    % * The input arguments can be single or double. Mixture of the data
    %   types is allowed, but all the values will be converted to the
    %   class of pose.
    % * The output has the same class as pose.

    % preallocate updatedPose variable
    updatedPose = zeros(size(pose),'like', pose);

    % cast velocity and timeStep into class of pose
    velocity = cast(velocity, 'like', pose);
    timeStep = cast(timeStep, 'like', pose);

    % extract linear and angular velocity
    v = velocity(1);
    w = velocity(2);

    % compute sine and cosine of current orientation
    sinTheta = sin(pose(3));
    cosTheta = cos(pose(3));

    % define the tolerance limit for data class
    tolerance = 2*sqrt(eps(class(pose)));

    if abs(w) < tolerance %When angular velocity is within tolerance limit
        % calculate intermediate values
        interVal1 = 0.5 * v * timeStep^2 * sinTheta;
        interVal2 = 0.5 * v * timeStep^2 * cosTheta;

        % update the pose using the following equations:
        % x = xpose + v*delT*cos(thetapose) - 0.5*v*w*delT^2*sin(thetapose)
        % y = ypose + v*delT*sin(thetapose) + 0.5*v*w*delT^2*cos(thetapose)
        % theta = thetapose + w*delT;
        updatedPose(1) = pose(1) + v * timeStep * cosTheta - interVal1 * w;
        updatedPose(2) = pose(2) + v * timeStep * sinTheta + interVal2 * w;
        updatedPose(3) = pose(3) + w*timeStep;

        % Go for Jacobians computation only if it is asked
        if nargout > 1
            % jacPose & jacVel are governed by following equations:
            % jacPose = [1, 0, -v*delT*sin(thetapose);
            %            0, 1,  v*delT*cos(thetapose);
            %            0, 0,    delT]
            % jacVel  = [delT*cos(thetapose), -0.5*v*delT^2*sin(thetapose);
            %            delT*sin(thetapose),  0.5*v*delT^2*cos(thetapose);
            %                              0,        delT]
            jacPose = [1, 0, -v * timeStep * sinTheta;
                       0, 1,  v * timeStep * cosTheta;
                       0, 0,                       1];
            jacVel = [timeStep * cosTheta, -interVal1;
                      timeStep * sinTheta,  interVal2;
                      0, timeStep];
        end
    else % When angular velocity is not within tolerance limit
        % update the pose using the following equations:
        % x = xpose - v/w*sin(thetapose) + v/w*sin(thetapose+w*delT)
        % y = ypose + v/w*cos(thetapose) - v/w*cos(thetapose+w*delT)
        % theta = thetapose + w*delT;

        % update the orientation
        updatedPose(3) = robotics.internal.wrapToPi(pose(3) + w * timeStep);

        % calculate intermediate values
        velbyom = v/w;
        sinThetaN = sin(updatedPose(3));
        cosThetaN = cos(updatedPose(3));
        delSin = sinThetaN - sinTheta;
        delCos = cosThetaN - cosTheta;

        % update the position
        updatedPose(1) = pose(1) + velbyom * delSin;
        updatedPose(2) = pose(2) - velbyom * delCos;

        % Go for Jacobians computation only if it is asked
        if nargout > 1
            % jacPose & jacVel are governed by following equations
            % jacPose = [1, 0, v/w*(cos(thetaPose+w*delT)-cos(thetapose));
            %            0, 1, v/w*(sin(thetaPose+w*delT)-sin(thetapose));
            %            0, 0, 1];
            % jacVel = [(sin(thetaPose+w*delT) - sin(thetapose))/w,
            %            v/w^2*(sin(thetaPose) - sin(thetapose+w*delT))...
            %            + v/w*delT*cos(thetapose+w*delT),
            %            0;
            %           (cos(thetaPose) - cos(thetapose+w*delT))/w,
            %            v/w^2*(cos(thetaPose+w*delT) - cos(thetapose))...
            %            + v/w*delT*sin(thetapose+w*delT),
            %            delT;
            jacPose = [1, 0, velbyom*delCos;
                       0, 1, velbyom*delSin;
                       0, 0,             1];
            jacVel = [delSin/w,-v*delSin/w^2 + velbyom*timeStep*cosThetaN;
                      -delCos/w, v*delCos/w^2 + velbyom*timeStep*sinThetaN;
                      0,                                 timeStep];
        end
    end
end
