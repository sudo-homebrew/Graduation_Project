
classdef SparseOccupancyGrid < ros.Message
    %SparseOccupancyGrid MATLAB implementation of jsk_pcl_ros/SparseOccupancyGrid
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'jsk_pcl_ros/SparseOccupancyGrid' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '497269ddab6058d0d4860f25dc49448f' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'OriginPose' 'Columns' 'Resolution' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'origin_pose' 'columns' 'resolution' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.geometry_msgs.Pose' ...
            'ros.msggen.jsk_pcl_ros.SparseOccupancyGridColumn' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        OriginPose
        Columns
        Resolution
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'SparseOccupancyGrid', 'Header')
            obj.Header = val;
        end
        function set.OriginPose(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Pose'};
            validateattributes(val, validClasses, validAttributes, 'SparseOccupancyGrid', 'OriginPose')
            obj.OriginPose = val;
        end
        function set.Columns(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.jsk_pcl_ros.SparseOccupancyGridColumn.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.jsk_pcl_ros.SparseOccupancyGridColumn'};
            validateattributes(val, validClasses, validAttributes, 'SparseOccupancyGrid', 'Columns')
            obj.Columns = val;
        end
        function set.Resolution(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SparseOccupancyGrid', 'Resolution');
            obj.Resolution = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.jsk_pcl_ros.SparseOccupancyGrid.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.jsk_pcl_ros.SparseOccupancyGrid(strObj);
        end
    end
end