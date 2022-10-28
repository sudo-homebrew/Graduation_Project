classdef OccupancyGrid < ros.msggen.nav_msgs.OccupancyGrid
%OccupancyGrid Custom MATLAB implementation of nav_msgs/OccupancyGrid type
%   This class provides conversion to and from the binaryOccupancyMap
%   and the occupancyMap classes.
%
%   OccupancyGrid methods:
%      readBinaryOccupancyGrid  - Returns a binaryOccupancyMap object
%      writeBinaryOccupancyGrid - Writes values from binaryOccupancyMap
%      readOccupancyGrid        - Returns a OccupancyMap object
%      writeOccupancyGrid       - Writes values from occupancyMap
%
%   See also binaryOccupancyMap, occupancyMap

%   Copyright 2014-2020 The MathWorks, Inc.

    properties (Access = private, Constant)
        %OccupancyThreshold Default occupancy threshold
        %   Default threshold above which all the values are considered
        %   occupied.
        OccupancyThreshold = 50;

        %ValueForUnknown Default value to replace unknown values
        %   Default value that will replace unknown values (-1) in the
        %   occupancy grid message. The value 0 means unoccupied (free).
        ValueForUnknown = 0;

        %UnknownValueInMsg The value used to represent unknown
        %   This value is used by ROS nav_msgs/OccupancyGrid to represent
        %   the unknown probability of occupancy
        UnknownValueInMsg = -1;

        %MaxValueInMsg The maximum value in ROS nav_msgs/OccupancyGrid message
        MaxValueInMsg = 100;
    end

    methods
        function obj = OccupancyGrid(varargin)
        %OccupancyGrid Constructor
        %   The arguments feed straight into the generated OccupancyGrid class.

            obj@ros.msggen.nav_msgs.OccupancyGrid(varargin{:});
        end

        function writeBinaryOccupancyGrid(obj, map)
        %writeBinaryOccupancyGrid Write values from robotics.BinaryOccupancyGrid
        %   writeBinaryOccupancyGrid(MSG, MAP) will write occupancy
        %   values and other meta information to the MSG from the
        %   binaryOccupancyMap object MAP. The meta information
        %   includes resolution, origin, width and height.
        %
        %   Example:
        %       % Create a binary occupancy map
        %       map = binaryOccupancyMap(randi([0,1], 10));
        %
        %       % Create a nav_msgs/OccupancyGrid message
        %       msg = rosmessage('nav_msgs/OccupancyGrid');
        %
        %       % Write the map data to the msg
        %       writeBinaryOccupancyGrid(msg, map);
        %

        % Validate input
            validateattributes(map,{'binaryOccupancyMap'}, ...
                               {'nonempty', 'scalar'});

            % Fill in the meta information
            obj.assignPropertiesToMsg(map);

            % Get grid values from the BinaryOccupancyGrid
            values = obj.readFromBOG(map);
            obj.Data = int8(values(:));
        end

        function writeOccupancyGrid(obj, map)
        %writeOccupancyGrid Write values from occupancyMap
        %   writeOccupancyGrid(MSG, MAP) will write occupancy
        %   values and other meta information to the MSG from the
        %   occupancyMap object MAP. The meta information
        %   includes resolution, origin, width and height.
        %
        %   Example:
        %       % Create an occupancy grid
        %       map = occupancyMap(rand(10));
        %
        %       % Create a nav_msgs/OccupancyGrid message
        %       msg = rosmessage('nav_msgs/OccupancyGrid');
        %
        %       % Write the map data to the msg
        %       writeOccupancyGrid(msg, map);
        %

        % Validate input
            validateattributes(map,{'occupancyMap'}, ...
                               {'nonempty', 'scalar'});

            % Fill in the meta information
            obj.assignPropertiesToMsg(map);

            % Get grid values from the OccupancyGrid
            values = obj.readFromOG(map);
            obj.Data = int8(values(:));
        end

        function map = readBinaryOccupancyGrid(obj, varargin)
        %readBinaryOccupancyGrid Returns a binaryOccupancyMap object
        %   MAP = readBinaryOccupancyGrid(MSG) returns a
        %   binaryOccupancyMap object MAP, reading the data from the MSG.
        %   The default occupancy threshold is 50. All values in the MSG
        %   data equal or greater than occupancy threshold are set as
        %   occupied (1) in the MAP and less than occupancy threshold are
        %   set as unoccupied (0) in the MAP. By default, the unknown
        %   values (-1) in the MSG data are set as unoccupied (0) in the MAP.
        %
        %   MAP = readBinaryOccupancyGrid(MSG, OT) returns a
        %   binaryOccupancyMap object MAP using the occupancy
        %   threshold OT and using the default for unknown values.
        %
        %   MAP = readBinaryOccupancyGrid(MSG, OT, VAL) returns a
        %   binaryOccupancyMap object MAP using the occupancy
        %   threshold OT and using VAL in place of the unknown
        %   values (-1).
        %
        %   Example:
        %       % Create a nav_msgs/OccupancyGrid message
        %       msg = rosmessage('nav_msgs/OccupancyGrid');
        %
        %       % Populate the ROS occupancy grid message
        %       msg.Info.Height = 10;
        %       msg.Info.Width = 10;
        %       msg.Info.Resolution = 0.1;
        %       msg.Data = 100*rand(100,1);
        %
        %       % Read the msg data and convert to binaryOccupancyMap
        %       map = readBinaryOccupancyGrid(msg);
        %
        %       % Read the msg data with threshold
        %       map = readBinaryOccupancyGrid(msg, 65);
        %
        %       % Read the msg data with threshold and replacement for
        %       % unknown value
        %       map = readBinaryOccupancyGrid(msg, 65, 1);

            opts.OccupancyThreshold = obj.OccupancyThreshold;
            opts.ValueForUnknown = obj.ValueForUnknown;

            % Parse optional input arguments if specified
            if ~isempty(varargin)
                opts = obj.parseReadBOGInputs(varargin{:});
            end

            % Validate message values
            validateMessageValues(obj);

            resolution = 1/double(obj.Info.Resolution);

            % Handle eps errors in resolution due to single to double
            % conversion. Using 1e-5 as a practical maximum EPS of a single
            % resolution
            if abs(round(resolution) - resolution) < 1e-5
                resolution = round(resolution);
            end

            map = binaryOccupancyMap(double(obj.Info.Height), ...
                                     double(obj.Info.Width), resolution, 'grid');

            % Fill in the meta information
            map.GridLocationInWorld = double([obj.Info.Origin.Position.X, ...
                                obj.Info.Origin.Position.Y]);

            % Set grid values in the binaryOccupancyMap
            values = double(obj.Data);
            values(obj.Data == obj.UnknownValueInMsg) = opts.ValueForUnknown;
            values(obj.Data < opts.OccupancyThreshold & obj.Data ~= obj.UnknownValueInMsg) = 0;
            values(obj.Data >= opts.OccupancyThreshold) = 1;

            % Write the values in the binaryOccupancyMap
            obj.writeToOG(map, values)
        end

        function og = readOccupancyGrid(obj)
        %readOccupancyGrid Returns an occupancyMap object
        %   MAP = readOccupancyGrid(MSG) returns a
        %   occupancyMap object MAP reading the data from the MSG.
        %   All values in the MSG are converted to probabilities
        %   between 0 and 1. The unknown values (-1) in the MSG data
        %   are set as 0.5 in the MAP.
        %
        %   Example:
        %       % Create a nav_msgs/OccupancyGrid message
        %       msg = rosmessage('nav_msgs/OccupancyGrid');
        %
        %       % Populate the ROS occupancy grid message
        %       msg.Info.Height = 10;
        %       msg.Info.Width = 10;
        %       msg.Info.Resolution = 0.1;
        %       msg.Data = 100*rand(100,1);
        %
        %       % Read the msg data and convert to occupancyMap
        %       map = readOccupancyGrid(msg);

        % Validate message values
            obj.validateMessageValues();

            resolution = 1/double(obj.Info.Resolution);

            % Handle eps errors in resolution due to single to double
            % conversion. Using 1e-5 as a practical maximum EPS of a single
            % resolution
            if abs(round(resolution) - resolution) < 1e-5
                resolution = round(resolution);
            end

            og = occupancyMap(double(obj.Info.Height), ...
                              double(obj.Info.Width), resolution, 'grid');

            % Fill in the meta information
            og.GridLocationInWorld = double([obj.Info.Origin.Position.X, ...
                                obj.Info.Origin.Position.Y]);

            % Set grid values in the occupancyMap
            values = double(obj.Data);
            values(obj.Data == obj.UnknownValueInMsg) = obj.MaxValueInMsg/2;
            values = values/obj.MaxValueInMsg;

            % Write the values in the occupancyMap
            obj.writeToOG(og, values)
        end
    end

    methods (Access = private)
        function assignPropertiesToMsg(msg, og)
        %assignPropertiesToMsg Assign occupancy grid properties to msg

        % Fill in the meta information
            msg.Info.Resolution = 1/og.Resolution;
            msg.Info.Origin.Position.X = og.GridLocationInWorld(1);
            msg.Info.Origin.Position.Y = og.GridLocationInWorld(2);
            msg.Info.Height = og.GridSize(1);
            msg.Info.Width = og.GridSize(2);
        end

        function validateMessageValues(obj)
        %validateMessageValues Validate message values

        % Error if height and width do not match number of elements in
        % the data
            if numel(obj.Data) ~= obj.Info.Height*obj.Info.Width
                error(message('ros:mlroscpp:occgrid:InvalidDataSize', ...
                              num2str(numel(obj.Data)), num2str(obj.Info.Height), ...
                              num2str(obj.Info.Width)));
            end

            % Error if resolution is zero
            validateattributes(obj.Info.Resolution, {'numeric'}, ...
                               {'nonempty', 'nonnan', 'positive'}, ...
                               'readOccupancyGrid', 'Resolution');

            % Validate that Data is between [0, 100] or -1
            validateattributes(obj.Data, {'numeric'}, ...
                               {'nonempty', '>=' obj.UnknownValueInMsg, '<=', obj.MaxValueInMsg}, ...
                               'readOccupancyGrid', 'Data');

            % Validate origin
            origin = [obj.Info.Origin.Position.X, obj.Info.Origin.Position.Y];
            validateattributes(origin, {'numeric'}, {'nonnan', 'finite'}, ...
                               'readOccupancyGrid', 'Origin');
        end

        function opts  = parseReadBOGInputs(obj, varargin)
        %parseReadBOGInputs Input parser for read method

            p = inputParser;
            addOptional(p,'OccupancyThreshold',obj.OccupancyThreshold, ...
                        @(x)validateattributes(x,{'double'}, ...
                                               {'nonnegative', 'scalar', 'finite'}));
            addOptional(p,'ValueForUnknown',obj.ValueForUnknown, ...
                        @(x)validateattributes(x,{'numeric','logical'}, ...
                                               {'nonnegative', 'scalar', 'binary'}));

            parse(p,varargin{:});
            opts.OccupancyThreshold = p.Results.OccupancyThreshold;

            % This is used with double data
            opts.ValueForUnknown = double(p.Results.ValueForUnknown);
        end

        function values = readFromBOG(obj, map)
        %readFromBOG Read values from binaryOccupancyMap

            occgrid = map.occupancyMatrix;

            % The data needs to be flipped and transposed to be compatible
            % with ROS
            values = (flip(occgrid))';
            values = double(values);
            values(values == 1) = obj.MaxValueInMsg;
        end

        function values = readFromOG(obj, map)
        %readFromOG Read values from occupancyMap

            occgrid = map.occupancyMatrix;

            % The data needs to be flipped and transposed to be compatible
            % with ROS
            values = (flip(occgrid))';
            values = values*obj.MaxValueInMsg;
        end

        function writeToOG(~, og, values)
        %writeToOG Write values to binaryOccupancyMap or occupancyMap

        % Use reverse GridSize values because data will be transposed
        % in the next step
            matrix = reshape(values, og.GridSize(2), og.GridSize(1));

            % While writing the data needs to be transposed and flipped to
            % be compatible with ROS
            matrix = flip(matrix');
            [x, y] =  ndgrid(1:og.GridSize(1), 1:og.GridSize(2));
            og.setOccupancy([x(:) y(:)], matrix(:), 'grid');
        end
    end

    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file

        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msg.nav_msgs.OccupancyGrid.empty(0,1);
                return
            end

            % Create an empty message object
            obj = ros.msg.nav_msgs.OccupancyGrid(strObj);
        end
    end
end
