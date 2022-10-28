classdef PointCloud2BusWrapper < ros.msg.sensor_msgs.internal.PointCloudInterface
%This class is for internal use only. It may be removed in the future.

%PointCloud2BusWrapper Implements PointCloudInterface for busstruct
%   This class is an implementation of
%   ros.msg.sensor_msgs.internal.PointCloudInterface where the
%   underlying storage is a busstruct corresponding to a
%   sensor_msgs/PointCloud2 message.
%
%   See also ros.slros.internal.block.ReadPointCloud

%   Copyright 2017-2021 The MathWorks, Inc.

%#codegen

%% Implemented for ros.msg.sensor_msgs.internal.PointCloudInterface
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
            offset = obj.BusStruct.Fields(fieldIdx).Offset;
        end

        function datatype = getFieldDatatype(obj, fieldIdx)
            datatype = obj.BusStruct.Fields(fieldIdx).Datatype;
        end

        function count = getFieldCount(obj, fieldIdx)
            count = obj.BusStruct.Fields(fieldIdx).Count;
        end
    end

    %% Additional properties
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
            obj.NumFields = busstruct.Fields_SL_Info.CurrentLength;

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
            val = obj.BusStruct.Width;
        end

        function set.Width(obj,val)
            obj.BusStruct.Width = val;
        end

        function val = get.Height(obj)
            val = obj.BusStruct.Height;
        end

        function set.Height(obj,val)
            obj.BusStruct.Height = val;
        end

        function val = get.PointStep(obj)
            val = obj.BusStruct.PointStep;
        end

        function val = get.Data(obj)
            val = obj.BusStruct.Data(1:obj.BusStruct.Data_SL_Info.CurrentLength);
        end
    end

    %% Additional methods
    methods (Access = private)
        function name = getFieldName(obj, fieldIdx)
            indices = 1:obj.BusStruct.Fields(fieldIdx).Name_SL_Info.CurrentLength;
            name = char(obj.BusStruct.Fields(fieldIdx).Name(indices))';
        end
        function out = matchesFieldName(obj, fieldIdx, fieldName)
            numChar = coder.const(numel(fieldName));
            numActual = length(obj.BusStruct.Fields(fieldIdx).Name);
            out = strcmp(fieldName, ...
                         char(obj.BusStruct.Fields(fieldIdx).Name(1:min(numActual,numChar)))');
        end
    end
end
