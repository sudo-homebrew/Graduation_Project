classdef Octomap < ros.msggen.octomap_msgs.Octomap
%Octomap Custom MATLAB implementation of octomap_msgs/Octomap type
%
%   Octomap methods:
%      readOccupancyMap3D  - Return a OccupancyMap3D object

%   Copyright 2017-2020 The MathWorks, Inc.

    methods
        function obj = Octomap(varargin)
        %Octomap Constructor
        %   The arguments feed straight into the generated Octomap class.

            obj@ros.msggen.octomap_msgs.Octomap(varargin{:});
        end

        function map = readOccupancyMap3D(obj)
        %readOccupancyMap3D - Return an occupancyMap3D object
        %   MAP = readOccupancyMap3D(MSG) returns an occupancyMap3D object
        %   MAP, reading the data from the MSG. Any binary or full octomap
        %   message can be converted into a MAP containing probabilities as
        %   per binary/full data on the MSG.
        %   NOTE: ColorOcTree messages are not supported.

        %   Example:
        %       % Create a octomap_msgs/Octomap message
        %       msg = rosmessage('octomap_msgs/Octomap');
        %
        %       % Populate the ROS octomap message
        %       msg.Id = 'OcTree';
        %       msg.Resolution = 0.1;
        %       msg.Data = int8(100*rand(100,1));
        %
        %       % Read the msg data and convert to occupancyMap3D
        %       map = readOccupancyMap3D(msg);

        % Error out for non OcTree Id (i.e., for ColorOcTree)
            if ~strcmp('OcTree', obj.Id)
                error(message('ros:mlroscpp:octomap:UnSupportedOctreeFormat', obj.Id));
            end

            % Construct the occupancyMap3D object and populate the data
            try
                % readOccupancyMap3D requires both ROS Toolbox and NAV
                % Toolbox. Since MapIO comes from backend, this call to
                % occupancyMap3D ensures error message containing
                % information about the Navigation Toolbox.
                map = nav.algs.internal.MapIO.deserializeROSMsgData(obj.Binary, obj.Resolution, obj.Data);
            catch ex
                % Do license check by instantiating basic Navigation
                % Toolbox class
                occupancyMap3D;
                % If not a license issue, propagate exception
                rethrow(ex)
            end
        end
    end

    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file

        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msg.octomap_msgs.Octomap.empty(0,1);
                return
            end

            % Create an empty message object
            obj = ros.msg.octomap_msgs.Octomap(strObj);
        end
    end
end
