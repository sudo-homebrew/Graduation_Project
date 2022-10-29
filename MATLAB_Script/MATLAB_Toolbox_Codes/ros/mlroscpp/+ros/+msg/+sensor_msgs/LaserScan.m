classdef LaserScan < ros.msggen.sensor_msgs.LaserScan
%LaserScan Custom MATLAB implementation of sensor_msgs/LaserScan type
%   This class provides a built-in visualization and conversion from
%   polar to Cartesian coordinates.
%
%   LaserScan methods:
%      readCartesian  - Return ranges in Cartesian coordinates
%      readScanAngles - Return scan angles for the range readings
%      lidarScan      - Return lidarScan object containing LaserScan msg data
%      plot           - Plot lidar scan data

%   Copyright 2014-2020 The MathWorks, Inc.

    methods
        function obj = LaserScan(varargin)
        %LaserScan Constructor
        %   The arguments feed straight into the generated LaserScan class.

            obj@ros.msggen.sensor_msgs.LaserScan(varargin{:});
        end

        function plotHandles = plot(obj, varargin)
        %PLOT Plot lidar scan data
        %   PLOT(OBJ) creates a point plot of the laser scan in the
        %   current axes (or creates a new one). The laser scan is
        %   shown in Cartesian (XY) coordinates and the axes are
        %   automatically scaled to the maximum range that the laser
        %   scanner supports.
        %
        %   The plot is oriented to follow the standard ROS convention of: x
        %   axis forward and y axis left.
        %
        %   PLOT(___, Name, Value) allows the specification of
        %   optional name/value pairs to control the plotting.
        %   Potential name/value pairs are:
        %      'Parent'       -  specifies the parent axes in which the laser scan
        %                        should be drawn. By default, the laser scan will be
        %                        plotted in the currently active axes.
        %      'MaximumRange' -  sets the maximum plot range to a fixed value (in meters).
        %                        This will set the minimum and maximum X axis limits and
        %                        the maximum Y axis limit to the specified value. The
        %                        minimum Y axis limit is automatically determined by the
        %                        opening angle of the laser scanner. You can use this
        %                        name-value pair if the automatic scaling does not give the
        %                        desired result.
        %                        Default: Value of RangeMax message property
        %
        %   PLOTHANDLES = PLOT(___) returns a column vector of line
        %   handles, PLOTHANDLES, for the laser scan. Use PLOTHANDLES to
        %   modify properties of the line after it is created.
        %
        %   This function will plot the Cartesian (XY) coordinates of the
        %   laser scan readings and scale the plot to fit the data.

        % Get Cartesian values
            [cart, cartAngles] = obj.readCartesian;

            % Parse input arguments
            % By default, use the maximum range for scaling
            defaults.Parent = [];

            % Only take the maximum range as default, if it is finite and
            % positive. Otherwise, use the maximum measured range as default.
            if isfinite(obj.RangeMax) && obj.RangeMax > 0
                defaults.MaximumRange = obj.RangeMax;
            else
                defaults.MaximumRange = max(obj.Ranges);
            end

            lineHandles = robotics.utils.internal.plotScan(metaclass(obj), cart, cartAngles, defaults, varargin{:});

            % Return the line handle if requested by the user
            if nargout > 0
                plotHandles = lineHandles;
            end
        end

        function lidarScanObj = lidarScan(obj)
        %lidarScan Return an object for storing 2D lidar scan from scan msg
        %   This function reads the Cartesian output of the laser scan msg
        %   and creates a lidarScan object.
        %   The ranges and angles in the lidarScan object will be
        %   stored in double-precision.
        %
        %   Example:
        %       % Load LaserScan message
        %       scanMsg = exampleHelperROSLoadRanges;
        %
        %       % Get a lidarScan object out of ROS LaserScan msg
        %       scan = lidarScan(scanMsg);
        %
        %       % Perform few operations on the lidarScan object
        %       minRange = 0.1;
        %       maxRange = 8.0;
        %       validScan = removeInvalidData(scan, 'RangeLimits', [minRange, maxRange])
        %
        %       % Visualize scan
        %       figure
        %       plot(validScan);
        %
        %       % Extract ranges and angles as single-precision data
        %       scanSingle = lidarScan(scanMsg.Ranges, scanMsg.readScanAngles)

            ranges = obj.Ranges;
            angles = obj.readScanAngles;
            lidarScanObj = lidarScan(double(ranges), angles);
        end

        function angles = readScanAngles(obj)
        %readScanAngles Return the scan angles for the range readings
        %   ANGLES = readScanAngles(OBJ) calculates the scan angles,
        %   ANGLES, corresponding to the range readings in the message OBJ.
        %
        %   Angles are measured counter-clockwise around the positive z
        %   axis, with the zero angle forward along the x axis. The
        %   angles will be returned in radians and are wrapped to the
        %   (-pi,pi] interval.
        %
        %   ANGLES will be returned as an N-by-1 vector, where N is
        %   equal to the number of range readings.

        % The message specifies the scanning angles through the
        % AngleMin, AngleMax and AngleIncrement properties. For some
        % sensor messages, these could be in conflict. For example,
        % commonly only the AngleMin and AngleIncrement properties
        % are used to generate the angle vector. See
        % http://docs.ros.org/melodic/api/laser_geometry/html/laser__geometry_8cpp_source.html
        % for the ROS implementation. Alternatively, scan angles could be calculated as a
        % linearly spaced vector between AngleMin and AngleMax. The
        % results of the two different ways of calculating the scan angles
        % might differ, but based on existing precedents we adhere to
        % the former convention.
            numReadings = numel(obj.Ranges);
            rawAngles = obj.AngleMin + (0:numReadings-1)' * obj.AngleIncrement;

            % Wrap the angles to the (-pi,pi] interval.
            angles = robotics.internal.wrapToPi(double(rawAngles));
        end

        function [cart, cartAngles] = readCartesian(obj, varargin)
        %readCartesian Return Cartesian (XY) coordinates for the scan
        %   CART = readCartesian(OBJ) converts the polar measurements of
        %   the laser scan into Cartesian coordinates CART. This
        %   function uses the meta data in the message, such as the
        %   angular resolution and opening angle of the laser scanner.
        %   The coordinates are returned in meters.
        %
        %   The coordinates follow the standard ROS convention of: x
        %   axis forward and y axis left.
        %
        %   CART = readCartesian(___, Name, Value) allows the specification of
        %   optional name/value pairs:
        %      'RangeLimits'  -  specifies the minimum and maximum
        %                     values of valid ranges as a [min max]
        %                     vector. All ranges smaller than min or
        %                     larger than max will be ignored during the
        %                     conversion to Cartesian coordinates.
        %                     Default: [OBJ.RangeMin OBJ.RangeMax]
        %
        %   [CART, CARTANGLES] = readCartesian(___) also returns the
        %   scan angles CARTANGLES that are associated with each of the
        %   Cartesian coordinates. The angles are measured counter-clockwise
        %   around the positive z axis and will be in radians.
        %   CARTANGLES will be returned as an N-by-1 vector and CART
        %   will be an N-by-2 matrix.
        %
        %   The XY coordinates will be returned as an N-by-2 matrix,
        %   where N is less than or equal to the number of range
        %   readings. All NaN or infinite ranges will be ignored for this
        %   conversion. All range values outside of the laser scanner's
        %   defined [RangeMin RangeMax] limits will also be ignored.

            defaults.RangeLimits = [obj.RangeMin, obj.RangeMax];
            args = obj.parseReadCartesianArguments(defaults, varargin{:});

            R = obj.Ranges;

            % If no ranges are available, return right away
            if isempty(R)
                cart = single.empty(0,2);
                return;
            end

            % Discard all values that are below lower range limit or above
            % the upper range limit. Also discard infinite or NaN values
            validIdx = isfinite(R) & R >= args.RangeLimits(1) & R <= args.RangeLimits(2);
            R = R(validIdx);

            % Filter scan angles for all valid range readings
            angles = obj.readScanAngles();
            cartAngles = angles(validIdx);
            x = cos(cartAngles) .* R;
            y = sin(cartAngles) .* R;
            cart = double([x,y]);
        end

    end

    methods (Static, Access = private)
        function args = parseReadCartesianArguments(defaults, varargin)
        %parseReadCartesianArguments Parse arguments for readCartesian function

        % Return right away if there is nothing to be parsed
            if isempty(varargin)
                args = defaults;
            end

            % Parse input
            parser = inputParser;

            % Parse the range limits
            % Needs to be a 2-vector with non-nan values and max >= min
            addParameter(parser, 'RangeLimits', defaults.RangeLimits, ...
                         @(x) validateattributes(x, {'numeric'}, ...
                                                 {'vector', 'numel', 2, 'nonnan', 'nondecreasing'}, ...
                                                 'LaserScan', 'RangeLimits'));

            % Return data
            parse(parser, varargin{:});
            args.RangeLimits = parser.Results.RangeLimits;

        end
    end

    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file

        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msg.sensor_msgs.LaserScan.empty(0,1);
                return
            end

            % Create an empty message object
            obj = ros.msg.sensor_msgs.LaserScan(strObj);
        end
    end
end
