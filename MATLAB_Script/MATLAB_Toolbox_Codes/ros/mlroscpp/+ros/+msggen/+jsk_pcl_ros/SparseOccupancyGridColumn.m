
classdef SparseOccupancyGridColumn < ros.Message
    %SparseOccupancyGridColumn MATLAB implementation of jsk_pcl_ros/SparseOccupancyGridColumn
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'jsk_pcl_ros/SparseOccupancyGridColumn' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '55074b193e722d5ead092ffe27f06522' % The MD5 Checksum of the message definition
        PropertyList = { 'Cells' 'ColumnIndex' } % List of non-constant message properties
        ROSPropertyList = { 'cells' 'column_index' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.jsk_pcl_ros.SparseOccupancyGridCell' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Cells
        ColumnIndex
    end
    methods
        function set.Cells(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.jsk_pcl_ros.SparseOccupancyGridCell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.jsk_pcl_ros.SparseOccupancyGridCell'};
            validateattributes(val, validClasses, validAttributes, 'SparseOccupancyGridColumn', 'Cells')
            obj.Cells = val;
        end
        function set.ColumnIndex(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SparseOccupancyGridColumn', 'ColumnIndex');
            obj.ColumnIndex = int32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.jsk_pcl_ros.SparseOccupancyGridColumn.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.jsk_pcl_ros.SparseOccupancyGridColumn(strObj);
        end
    end
end
