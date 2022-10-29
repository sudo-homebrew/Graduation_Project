function [rangeBearingReading, jacPose, jacLandmarkPosition] = ...
        rangeBearingMeasurement(pose, landmarkPosition)
    %RANGEBEARINGMEASUREMENT Measurement function for range-bearing readings
    %   RANGEBEARINGREADING = RANGEBEARINGMEASUREMENT(POSE, LANDMARKPOSITION)
    %   transforms the position of landmarks, LANDMARKPOSITION, stored in
    %   Cartesian coordinates to range-bearing readings, RANGEBEARINGREADING,
    %   relative to the current pose, POSE.
    %
    %   [RANGEBEARINGREADING, JACPOSE, JACLANDMARKPOSITION] = ...
    %                      RANGEBEARINGMEASUREMENT(POSE, LANDMARKPOSITION)
    %   returns the Jacobian matrices of the range-bearing measurement
    %   function with respect to the landmark position and the current
    %   pose, along with range-bearing readings.
    %
    %   Input Arguments:
    %
    %   POSE                - Current pose of the vehicle in the local
    %                         navigation coordinate system specified as a
    %                         three-element vector of real values. The 
    %                         vector is of form [X Y Yaw]. X and Y specify
    %                         the position in meters. Yaw specifies the
    %                         orientation in radians.
    %
    %   LANDMARKPOSITION    - Position of landmarks in the local navigation
    %                         coordinate system specified as an N-by-2
    %                         matrix, where N is the number of landmarks.
    %                         The first column of the matrix contains the
    %                         X-coordinate values and the second column
    %                         contains the Y-coordinate values.
    %
    %   Output Arguments:
    %
    %   RANGEBEARINGREADING - Range-bearing readings relative to the pose
    %                         of the vehicle returned as an N-by-2 matrix,
    %                         where N is the number of readings. The first
    %                         column of the matrix contains Range (radial
    %                         coordinate) and the second column contains
    %                         Bearing (angular coordinate).
    %
    %   JACPOSE             - Jacobian of the range bearing measurement
    %                         function with respect to pose, returned as an
    %                         N*2-by-3 matrix, where N is the number of
    %                         landmarks.
    %
    %   JACLANDMARKPOSITION - Jacobian of the range bearing measurement
    %                         function with respect to landmark position,
    %                         returned as an N*2-by-2 matrix, where N is
    %                         the number of landmarks.
    %
    %   Either single or double datatypes are supported for the inputs to
    %   RANGEBEARINGMEASUREMENT. Outputs have the same datatype as the pose.
    %
    %   % Example: Get the range-bearing reading and Jacobians
    %
    %   % create input arguments
    %   pose  = [1; -2; 0.1];
    %   lmPos = [3, 4];
    %
    %   % calculate the range-bearing reading along with Jacobians
    %   [rb, jacPose, jacLM] = nav.algs.rangeBearingMeasurement(pose, lmPos);
    %
    %   See also nav.algs.rangeBearingInverseMeasurement, ekfSLAM

    %   Copyright 2021 The MathWorks, Inc.

    %#codegen

    % Check that number of arguments is exactly 2
    narginchk(2,2);

    % validate pose
    validateattributes(pose, {'single', 'double'}, ...
                       {'real', 'nonnan', 'finite', 'nonempty', ...
                        'vector', 'numel', 3}, ...
                       'nav.algs.rangeBearingMeasurement', ...
                       'pose');

    % validate landmarkPosition
    validateattributes(landmarkPosition, {'single', 'double'}, ...
                       {'real', 'nonnan','finite', '2d', ...
                        'nonempty', 'ncols', 2}, ...
                       'nav.algs.rangeBearingMeasurement', ...
                       'landmarkPosition');

    % Data type would be determined by the pose argument.
    % Rules for data classes:
    % * The input arguments can be single or double. Mixture of the data
    %   types is allowed, but all the values will be converted to the
    %   class of pose.
    % * The outputs have the same class as pose.

    % cast landmarkPosition into class of pose
    landmarkPosition = cast(landmarkPosition, 'like', pose);

    % calculate the range-bearing readings using the following equation:
    % range = sqrt((xLM - xpose)^2 + (yLM - ypose)^2)
    % bearing = atan((yLM - ypose), (xLM - xpose)) - thetapose
    del = landmarkPosition - ...
          repmat([pose(1), pose(2)], size(landmarkPosition,1), 1);
    range = hypot(del(:,1), del(:,2));
    bearing = ...
        robotics.internal.wrapToPi(atan2(del(:,2),del(:,1))-pose(3));
    rangeBearingReading = [range bearing];

    % Go for Jacobians computation only if it is asked
    if nargout > 1
        % calculate the intermediate values for vectorized computation
        rangeSq = range.^2;
        delByRange = del./repmat(range,1,2);
        delByRangeSq = del./repmat(rangeSq,1,2);
        delByRangeSq(:,[1,2]) = delByRangeSq(:,[2,1]);

        % jacPose & jacLandmarkPosition are governed by following equations
        % jacPose = [(xpose - xLM)/range,   (ypose - yLM)/range,    0;
        %            (yLM - ypose)/range^2, (xpose - xLM)/range^2, -1]
        % jacLandmarkPosition = [(xLM - xpose)/range,   (yLM - ypose)/range;
        %                      (ypose - yLM)/range^2, (xLM - xpose)/range^2]

        % number of landmarks
        numLM = size(landmarkPosition,1);

        % initialize the variables for Jacobians
        jacPose = cast(repmat([zeros(2),[0;-1]],numLM,1), 'like', pose);
        jacLandmarkPosition = ...
            zeros(2*numLM, size(landmarkPosition,2), 'like', pose);

        % Indices where values are to be substituted
        indices = repmat(2*(1:numLM)',1,2) + repmat([-1,0],numLM,1);

        % fill in the Jacobian variables with values
        jacPose(indices,1:2) = ...
            [-delByRange(:,:); repmat([1,-1],numLM,1).*delByRangeSq(:,:)];
        jacLandmarkPosition(indices,1:2) = ...
            [ delByRange(:,:); repmat([-1,1],numLM,1).*delByRangeSq(:,:)];
    end
end
