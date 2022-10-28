function [landmarkPosition, jacPose, jacRangeBearingReading] = ...
        rangeBearingInverseMeasurement(pose, rangeBearingReading)
    %RANGEBEARINGINVERSEMEASUREMENT Inverse measurement function for range-bearing readings
    %   LANDMARKPOSITION = RANGEBEARINGINVERSEMEASUREMENT(POSE, RANGEBEARINGREADING)
    %   transforms range-bearing readings, RANGEBEARINGREADING, to the
    %   position of landmarks in Cartesian coordinates in the POSE frame.
    %
    %   [LANDMARKPOSITION, JACPOSE, JACRANGEBEARINGREADING] = ...
    %                   RANGEBEARINGMEASUREMENT(POSE, RANGEBEARINGREADING)
    %   returns the Jacobian matrices of the range-bearing inverse
    %   measurement function with respect to the current pose and the
    %   range-bearing readings, along with the position of landmarks.
    %
    %   Input Arguments:
    %
    %   POSE                   - Current pose of the vehicle in the local
    %                            navigation coordinate system specified as
    %                            a three-element vector of real values. The
    %                            vector is of form [X Y Yaw]. X and Y
    %                            specify the position in meters. Yaw
    %                            specifies the orientation in radians.
    %
    %   RANGEBEARINGREADING    - Range-bearing readings relative to the
    %                            pose of the vehicle specified as an
    %                            N-by-2 matrix, where N is the number of
    %                            readings. The first column contains Range
    %                            (radial coordinate) and second column
    %                            contains Bearing (angular coordinate).
    %
    %   Output Arguments:
    %
    %   LANDMARKPOSITION       - Position of landmarks in the local
    %                            navigation coordinate system returned as
    %                            an N-by-2 matrix, where N is the number of
    %                            landmarks. The first column of the matrix
    %                            contains the X-coordinate values and the
    %                            second column contains the Y-coordinate 
    %                            values.
    %
    %   JACPOSE                - Jacobian of the range bearing inverse
    %                            measurement function with respect to pose,
    %                            returned as an N*2-by-3 matrix, where
    %                            N is the number of range-bearing readings.
    %
    %   JACRANGEBEARINGREADING - Jacobian of the range bearing inverse
    %                            measurement function with respect to
    %                            range-bearing readings, returned as an
    %                            N*2-by-2 matrix, where N is the number of
    %                            range-bearing readings.
    %
    %   Either single or double datatypes are supported for the inputs to
    %   RANGEBEARINGINVERSEMEASUREMENT. Outputs have the same datatype as
    %   the pose.
    %
    %   % Example: Get the landmark position and Jacobians
    %
    %   % create input arguments
    %   pose = [1; -2; 0.1];
    %   rb = [5, 0.1];
    %
    %   % calculate the landmark position along with Jacobians
    %   [lmPos, jacPose, jacRB] = ...
    %                   nav.algs.rangeBearingInverseMeasurement(pose, rb);
    %
    %   See also nav.algs.rangeBearingMeasurement, ekfSLAM

    %   Copyright 2021 The MathWorks, Inc.

    %#codegen

    % Check that number of arguments is exactly 2
    narginchk(2,2);

    % validate pose
    validateattributes(pose, {'single', 'double'}, ...
                       {'real', 'nonnan', 'finite', 'nonempty', ...
                        'vector', 'numel', 3}, ...
                       'nav.algs.rangeBearingInverseMeasurement', ...
                       'pose');

    % validate rangeBearingReading
    validateattributes(rangeBearingReading, {'single', 'double'}, ...
                       {'real', 'nonnan','finite', 'nonempty', ...
                        '2d', 'ncols', 2}, ...
                       'nav.algs.rangeBearingInverseMeasurement', ...
                       'rangeBearingReading');
    validateattributes(rangeBearingReading(:,1), {'double', 'single'}, ...
                       {'nonnegative'}, ...
                       'nav.algs.rangeBearingInverseMeasurement', ...
                       'rangeBearingReading');
    % Data type would be determined by the pose argument.
    % Rules for data classes:
    % * The input arguments can be single or double. Mixture of the data
    %   types is allowed, but all the values will be converted to the
    %   class of pose.
    % * The outputs have the same class as pose.
    rangeBearingReading = cast(rangeBearingReading, 'like', pose);

    % calculate the landmark position using the following equations:
    % xLM = xPose + range*cos(thetaPose+bearing)
    % yLM = yPose + range*sin(thetaPose+bearing)
    landmarkPosition = ...
        [pose(1)+ rangeBearingReading(:,1).*cos(pose(3)+rangeBearingReading(:,2)), ...
         pose(2)+ rangeBearingReading(:,1).*sin(pose(3)+rangeBearingReading(:,2))];

    % Go for Jacobians computation only if it is asked
    if nargout > 1
        % calculate the intermediate values for vectorized computation
        thetaN = pose(3) + rangeBearingReading(:,2);
        cosThN = cos(thetaN);
        sinThN = sin(thetaN);
        RcosThN = rangeBearingReading(:,1).*cosThN;
        RsinThN = rangeBearingReading(:,1).*sinThN;

        % jacPose and jacRangeBearingReading are governed by following
        % equations:
        % jacPose  = [1, 0, -range*sin(thetaPose+bearing);
        %             0, 1,  range*cos(thetaPose+bearing)]
        % jacRangeBearingReading =
        %          [cos(thetaPose+bearing), -range*sin(thetaPose+bearing)
        %           sin(thetaPose+bearing),  range*cos(thetaPose+bearing)]

        % number of range-bearing readings
        numRB = size(rangeBearingReading,1);

        % initialize the variables for Jacobians
        jacPose = cast(repmat([eye(2), [0; 0]],numRB,1), 'like', pose);
        jacRangeBearingReading = cast(repmat(eye(2), numRB,1), 'like', pose);

        % Indices where values are to be substituted
        indices = repmat(2*(1:numRB)',1,2) + repmat([-1,0],numRB,1);

        % fill in the Jacobian variables with values
        jacPose(indices,3) =   [-RsinThN(:); RcosThN(:)];
        jacRangeBearingReading(indices,1:2) = [cosThN(:), -RsinThN(:);
                            sinThN(:),  RcosThN(:)];
    end
end
