classdef VarLenArrayInfo < handle
%This class is for internal use only. It may be removed in the future.

%VarLenArrayInfo Data object with information about variable-length array properties in ROS messages
%   VarLenArrayInfo holds information about all variable-length
%   properties for a given message type (equivalently, a Simulink bus).
%   It is used when saving and retrieving info about maximum sizes of
%   variable-length arrays. This class is aware of Simulink buses and
%   ROS messages but does not deal with any model-related information.
%
%   Note: The VarLenArrayInfo object is initialized either using a
%   Simulink bus or using a msgType & modelName (the latter is used to
%   retrieve the corresponding Simulink bus).
%
%   Sample invocation:
%    msginfo = ros.slros.internal.bus.VarLenArrayInfo(bus)
%    msginfo = ros.slros.internal.bus.VarLenArrayInfo('geographic_msgs/WayPoint', gcs)
%    msginfo.getMaxLength('MyArrayProp')
%    msginfo.setMaxLength('MyArrayProp', 256)
%    msginfo.setMaxLength({'MyArrayProp', 'MyArrayProp2}, [256 512])
%
%   See also: bus.VarlenArraySizeStore

%   Copyright 2014-2019 The MathWorks, Inc.


    properties (SetAccess=immutable)
        %MessageType - Message type associated with this data structure
        MessageType
    end

    properties(SetAccess=immutable, GetAccess=private)
        %ArrayPropMap - Map of array properties to associated array info
        ArrayPropMap
    end

    properties(Access=private)
        %ArrayProps - Array properties data structure
        ArrayProps = struct('PropertyName', {}, 'DataType', {}, 'MaxLength', {})
    end

    properties(Dependent, SetAccess=private)
        %PropertyNames - Property names for this message types that have var-size arrays
        PropertyNames
    end

    methods
        function obj = VarLenArrayInfo(varargin)
        %VarLenArrayInfo Constructor
            narginchk(1,4);
            if nargin == 1
                bus = varargin{1};
            else
                p = inputParser();
                addRequired(p, 'msgType', @(x)validateattributes(x, {'char'}, {'nonempty'}));
                addRequired(p, 'modelName', @(x)validateattributes(x, {'char'}, {'nonempty'}));
                addParameter(p, 'BusUtilityObject', ros.slros.internal.bus.Util, @(x)isa(x, 'ros.slros.internal.bus.Util'));
                parse(p, varargin{:});
                msgType = p.Results.msgType;
                modelName = p.Results.modelName;
                busUtilObj = p.Results.BusUtilityObject;
                validateattributes(msgType, {'char'}, {'nonempty'});
                validateattributes(modelName, {'char'}, {'nonempty'});
                bus = busUtilObj.getBusObjectFromMsgType(msgType, modelName);
            end
            validateattributes(bus, {'Simulink.Bus'}, {'scalar'});
            businfo = ros.slros.internal.bus.BusItemInfo( bus.Description );
            obj.MessageType = businfo.MsgType;

            obj.ArrayPropMap = containers.Map;

            for j = 1:numel(bus.Elements)
                elem = bus.Elements(j);
                elemInfo = ros.slros.internal.bus.BusItemInfo( elem.Description );

                if elemInfo.isVarLenDataElement()
                    s.PropertyName = elem.Name;
                    if ~isempty(elemInfo.MsgType)
                        s.DataType = elemInfo.MsgType;
                    else % primitive type
                        s.DataType = elem.DataType;
                    end
                    s.MaxLength = elem.Dimensions;

                    obj.ArrayPropMap(s.PropertyName) = s;
                end
            end

        end

        function out = hasVarLenArrayProperties(obj)
        %hasVarLenArrayProperties Does message have var-size array properties
            out = obj.ArrayPropMap.Count > 0;
        end

        function out = get.PropertyNames(obj)
            out = obj.ArrayPropMap.keys;
        end

        function maxLengths = getMaxLength(obj, propertyNames)
        %getMaxLength Get maximum array lengths for given property names

            propertyNames = cellstr(propertyNames);
            maxLengths = zeros(1,numel(propertyNames));
            for i=1:numel(propertyNames)
                maxLengths(i) = obj.ArrayPropMap(propertyNames{i}).MaxLength;
            end
        end

        function [dataType, isROSMsgType] = getDataType(obj, propertyName)
        %getDataType Get data type for property name

            dataType = obj.ArrayPropMap(propertyName).DataType;
            isROSMsgType = contains(dataType, '/');
        end

        function setMaxLength(obj, propertyNames, maxLengths)
        %setMaxLength Set maximum lengths for var-size arrays associated with property names

            propertyNames = cellstr(propertyNames);
            validateattributes(maxLengths, {'numeric'}, {'positive', 'integer', 'numel', numel(propertyNames)});
            for i=1:numel(propertyNames)
                s = obj.ArrayPropMap(propertyNames{i});
                s.MaxLength = maxLengths(i);
                obj.ArrayPropMap(propertyNames{i}) = s;
            end
        end
    end

    %%
    methods(Access = {?ros.slros.internal.bus.VarlenArraySizeStore, ?matlab.unittest.TestCase})
        % convertToStruct and initFromStruct are used to "serialize" and
        % deserialize the array information into structs for saving in the
        % model workspace. Consequently, they are only used by VarlenArraySizeStore

        function propStruct = convertToStruct(obj)
        %convertToStruct Convert array information into struct

            propertyNames = obj.ArrayPropMap.keys;
            values = obj.ArrayPropMap.values(propertyNames);
            dataTypes = cellfun(@(x) x.DataType, values, 'UniformOutput', false);
            maxLengths = cellfun(@(x) x.MaxLength, values, 'UniformOutput', false);
            propStruct = struct('PropertyName', propertyNames, 'DataType', dataTypes, 'MaxLength', maxLengths);
        end

        function initFromStruct(obj, propStruct)
        %initFromStruct Initialize array information from struct

            validateattributes(propStruct, {'struct'}, {});
            for i=1:numel(propStruct)
                propName = propStruct(i).PropertyName;
                existingInfo = obj.ArrayPropMap(propName);
                assert(strcmp(existingInfo.DataType, propStruct(i).DataType));
                obj.setMaxLength(propName, propStruct(i).MaxLength);
            end
        end
    end

end
