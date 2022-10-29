classdef PointCloud2BusWrapper < ros.msg.sensor_msgs.internal.PointCloudInterface
%This class is for internal use only. It may be removed in the future.

%PointCloud2BusWrapper Implements PointCloudInterface for busstruct
%   This class is an implementation of a busstruct corresponding to a
%   sensor_msgs/PointCloud2 message.
%
%   See also ros.slros2.internal.block.ReadPointCloud

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties (Dependent)
        Width
        Height
        PointStep
        Data
    end

    properties
        PreserveStructureOnRead
    end

    methods
        function fieldNames = readAllFieldNames(obj)
            fieldNames = cell(1, obj.NumFields);
            for i = 1:obj.NumFields
                fieldNames{i} = obj.getFieldName(i);
            end
        end

        function fieldIdx = getFieldIndex(obj, fieldName)
            fieldIdx = obj.getFieldNameIndex(fieldName);
        end
    end

    methods (Access = {...
                        ?ros.msg.sensor_msgs.internal.PointCloudInterface, ...
                        ?ros.msg.sensor_msgs.internal.PointCloud2Reader, ...
                        ?matlab.unittest.TestCase})
        function fieldIdx = getFieldNameIndex(obj, fieldName)
        %getFieldNameIndex Get index of field in PointField array
        %   Returns 0 if the field is not present
            fieldIdx = uint32(0);
            for i = 1:obj.NumFields
                if obj.matchesFieldName(i, fieldName)
                    fieldIdx = i;
                    return
                end
            end
        end

        function offset = getFieldOffset(obj, fieldIdx)
            offset = obj.BusStruct.fields(fieldIdx).offset;
        end

        function datatype = getFieldDatatype(obj, fieldIdx)
            datatype = obj.BusStruct.fields(fieldIdx).datatype;
        end

        function count = getFieldCount(obj, fieldIdx)
            count = obj.BusStruct.fields(fieldIdx).count;
        end
    end

    properties (Access = private)
        %BusStruct
        BusStruct

        %NumFields
        NumFields
    end

    %% Constructor
    methods
        function obj = PointCloud2BusWrapper(busstruct)
            obj.BusStruct = busstruct;

            % Set the FieldNames
            obj.NumFields = busstruct.fields_SL_Info.CurrentLength;

            if isfield(busstruct,'PreserveStructureOnRead')
                obj.PreserveStructureOnRead = busstruct.PreserveStructureOnRead;
            end
        end
    end

    %% Property access methods
    methods
        function val = get.PreserveStructureOnRead(obj)
            val = obj.PreserveStructureOnRead;
        end

        function set.PreserveStructureOnRead(obj,val)
            obj.PreserveStructureOnRead = val;
        end

        function val = get.Width(obj)
            val = obj.BusStruct.width;
        end

        function set.Width(obj,val)
            obj.BusStruct.width = val;
        end

        function val = get.Height(obj)
            val = obj.BusStruct.height;
        end

        function set.Height(obj,val)
            obj.BusStruct.height = val;
        end

        function val = get.PointStep(obj)
            val = obj.BusStruct.point_step;
        end

        function val = get.Data(obj)
            val = obj.BusStruct.data(1:obj.BusStruct.data_SL_Info.CurrentLength);
        end
    end

    %% Additional methods
    methods (Access = private)
        function name = getFieldName(obj, fieldIdx)
            indices = 1:obj.BusStruct.fields(fieldIdx).name_SL_Info.CurrentLength;
            name = char(obj.BusStruct.fields(fieldIdx).name(indices))';
        end
        function out = matchesFieldName(obj, fieldIdx, fieldName)
            numChar = coder.const(numel(fieldName));
            numActual = length(obj.BusStruct.fields(fieldIdx).name);
            out = strcmp(fieldName, ...
                         char(obj.BusStruct.fields(fieldIdx).name(1:min(numActual,numChar)))');
        end
    end
end
