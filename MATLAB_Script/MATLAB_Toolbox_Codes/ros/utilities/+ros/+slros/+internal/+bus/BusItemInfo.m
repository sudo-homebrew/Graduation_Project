classdef BusItemInfo
%This class is for internal use only. It may be removed in the future.

%  BUSITEMINFO is an interface to save and retrieve structured
%  information from the 'Description' field of a Simulink.BusElement or
%  Simulink.Bus object. This allows ROS-specific metadata to stored
%  with the Bus object.
%
%  Examples encoded Description:
%    'MsgType=geographic_msgs/KeyValue:IsVarLen=1:VarLenCategory=data:VarLenElem=Props_SL_Info:TruncateAction=warn'
%    'IsVarLen=1:VarLenCategory=length:VarLenElem=Props'
%    'PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=Key_SL_Info:TruncateAction=warn'

%   Copyright 2014-2019 The MathWorks, Inc.


    properties
        % ROS message type
        MsgType = ''

        % ROSPropName - Name of the ROS property (needed when the name is
        % on the Simulink reserved list).
        ROSPropName = ''

        % PrimitiveROSType - indicates a string or string[] primitive ROS
        % type (if empty, consult Simulink BusElement.DataType).
        % Valid values are 'string' | 'string[]' | ''
        PrimitiveROSType = ''

        % IsVarLen -  whether this a variable Length element
        IsVarLen = false

        % VarLenCategory - the category of variable-length information
        % Valid values are 'data' | 'length' | ''
        VarLenCategory = ''

        % VarLenElem - The name of the element that has the associated
        % Variable-length information.
        % if VarLenCategory=data, VarLenElem is the name of the element that has the length
        % if VarLenCategory=length, VarLenElem is the name of element that has the data
        VarLenElem = ''

        % Int64Type - Indicates whether this was a u/int64 ROS datatype
        % Valid values are 'uint64' | 'int64' | ''
        Int64Type = ''

        % TruncateAction - Indicates the truncation action (if this is a
        % variable-length array)
        % Valid values are 'warn' | 'none' | ''
        TruncateAction = ''
    end

    methods
        function obj = BusItemInfo(desc)
            if ~exist('desc', 'var')
                desc = '';
            end
            assert(ischar(desc));
            items = regexp(desc, ':', 'split');
            for i=1:numel(items)
                if isempty(items{i})
                    continue;
                end
                tokens = regexp(items{i}, '=', 'split');
                assert(numel(tokens)==2, 'Tokenizing problem in %s', desc);
                if strcmp(tokens{1}, 'IsVarLen')
                    obj.IsVarLen = (tokens{2} == '1');
                else
                    obj.(tokens{1}) = tokens{2};
                end
            end
        end

        function out = isVarLenDataElement(obj)
            out = obj.IsVarLen && strcmpi(obj.VarLenCategory, 'data');
        end

        function obj = set.IsVarLen(obj, val)
            assert(islogical(val) && isscalar(val));
            obj.IsVarLen = val;
        end

        function obj = set.PrimitiveROSType(obj, val)
            validatestring(lower(val), {'string', 'string[]', '', 'char'});
            obj.PrimitiveROSType = lower(val);
        end

        function obj = set.Int64Type(obj, val)
            validatestring(lower(val), {'int64', 'uint64', ''});
            obj.Int64Type = lower(val);
        end

        function obj = set.TruncateAction(obj, val)
            validatestring(lower(val), {'warn', 'none', ''});
            obj.TruncateAction = lower(val);
        end

        function obj = set.VarLenElem(obj, val)
            assert(ischar(val));
            obj.VarLenElem = val;
        end

        function obj = set.VarLenCategory(obj, val)
            validatestring(lower(val), {'', 'data', 'length'});
            obj.VarLenCategory = lower(val);
        end

        function desc = toDescription(obj)
            fldnames = fieldnames(obj);
            info = {};
            for i=1:numel(fldnames)
                if strcmp(fldnames{i}, 'IsVarLen')
                    if  obj.(fldnames{i})
                        info{end+1} = [fldnames{i} '=' '1']; %#ok<AGROW>
                    end
                elseif ~isempty(obj.(fldnames{i}))
                    info{end+1} = [fldnames{i} '=' obj.(fldnames{i})]; %#ok<AGROW>
                end
            end
            desc = strjoin(info, ':');
        end

    end

end
