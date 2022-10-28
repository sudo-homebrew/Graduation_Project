classdef (Abstract) DataFormatBase < handle
%This class is for internal use only. It may be removed in the future.

%DataFormatBase Base class for all ROS entities that use DataFormat
%   Several ROS entities allow for ROS message structs or objects to be
%   used depending on the setting for the DataFormat property. This class
%   manages that setting.

%   Copyright 2020 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %DataFormat - Message format to be used
        %   When 'object', refers to an object subclassed from ros.Message.
        %   When 'struct', refers to a struct with the same fields.
        %   This property determines which format of message will be
        %   required to be provided to APIs that accept ROS messages as
        %   input, or will be returned by APIs that output ROS messages.
        DataFormat
    end

    properties (Access = protected)
        %UseObjectMsg - Indicates if object message format is to be used
        %   This is pre-computed when DataFormat is set to avoid multiple
        %   unnecessary string comparisons in frequently-called APIs.
        UseObjectMsg
    end

    properties (Constant, Access = private)
        %DataFormatValues - Acceptable values for the DataFormat property
        DataFormatValues = {'object', 'struct'}

        %DefaultDataFormat - Message format used by default
        DefaultDataFormat = 'object';
    end

    methods
        function dataFormat = get.DataFormat(obj)
        %get.DataFormat Custom getter for DataFormat property

            if obj.UseObjectMsg
                dataFormat = 'object';
            else
                dataFormat = 'struct';
            end
        end
    end

    methods (Access = protected)
        function setDataFormat(obj, dataFormat)
        %setDataFormat Set underlying property from provided data format
        %   Accepts parser-provided DataFormat value, including partial
        %   matches, and sets the underlying UseObjectMsg property that
        %   tracks the behavior of the object

            obj.UseObjectMsg = strncmpi(dataFormat, 'object', strlength(dataFormat));
        end

        function addDataFormatToParser(obj, parser, className)
        %addDataFormatToParser Add DataFormat parameter to input parser
        %   className is the name to be shown in error messages if the
        %   arguments parsed are invalid.
        %   inputParser is a handle, so no need for return value.

            addParameter(parser, 'DataFormat', obj.DefaultDataFormat, ...
                         @(x) ~isempty(validatestring(x, ...
                                                      obj.DataFormatValues, ...
                                                      className, ...
                                                      'DataFormat')))
        end

        function validateDataFormatROSMessage(obj, varargin)
        %validateDataFormatROSMessage Check parameters of rosmessage method
        %   When rosmessage(OBJ) syntax is used, rather than
        %   rosmessage("TYPE"), if the DataFormat parameter is provided, it
        %   needs to match with the DataFormat property of the input OBJ.
        %   This function does that validation, only checking if there are
        %   any inputs.

        % Check name-value pair inputs only if necessary
            if nargin > 1
                % Set up name-value pair, using current value as default
                parser = inputParser;
                addParameter(parser, 'DataFormat', obj.DataFormat, ...
                             @(x) ~isempty(validatestring(x, ...
                                                          obj.DataFormatValues, ...
                                                          'rosmessage', ...
                                                          'DataFormat')))
                parse(parser, varargin{:})
                if ~strncmpi(parser.Results.DataFormat, obj.DataFormat, ...
                             strlength(parser.Results.DataFormat))
                    className = strsplit(class(obj));
                    className = className{end}; % Use unpackaged class name
                    error(message('ros:mlroscpp:message:ROSMessageDFMismatch', className))
                end
            end
        end

        function validateInputMessage(obj, msg, msgType, className, fcnName)
        %validateInputMessage Do error checking on message format and type
        %   This is intended for use on user-provided messages within a
        %   try-catch block (as the validation may be too slow for checking
        %   the message under normal functionality).
        %   msgType may be a char array or cellstr of acceptable message
        %   type(s).

            if obj.UseObjectMsg
                if isstruct(msg)
                    error(message('ros:mlroscpp:message:InputDFMismatch', className))
                end
                validateattributes(msg, {'ros.Message'}, {'scalar'}, fcnName, 'msg')
                if ~any(strcmp(msg.MessageType, msgType))
                    if iscell(msgType)
                        msgType = strjoin(msgType, ', ');
                    end
                    error(message('ros:mlroscpp:message:InputTypeMismatch', msgType))
                end
            else
                if isa(msg, 'ros.Message')
                    error(message('ros:mlroscpp:message:InputDFMismatch', className))
                end
                validateattributes(msg, {'struct'}, {'scalar'}, fcnName, 'msg')
                if ~isfield(msg, 'MessageType') || ~any(strcmp(msg.MessageType, msgType))
                    if iscell(msgType)
                        msgType = strjoin(msgType, ', ');
                    end
                    error(message('ros:mlroscpp:message:InputTypeMismatch', msgType))
                end
            end
        end
    end
end
