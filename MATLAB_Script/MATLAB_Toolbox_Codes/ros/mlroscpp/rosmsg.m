function output = rosmsg( op, varargin )
%ROSMSG Retrieve information about messages and message types
%   MSGINFO = ROSMSG('show', 'TYPE') prints out the definition of the
%   message with TYPE. If the output argument MSGINFO is defined, return
%   the information as a string. Simplified form: ROSMSG show TYPE
%
%   MSGMD5 = ROSMSG('md5', 'TYPE') prints out the MD5 checksum of the
%   message with TYPE. If the output argument MSGMD5 is defined, return the
%   checksum as a string. Simplified form: ROSMSG md5 TYPE
%
%   MSGLIST = ROSMSG('list') lists all available message types
%   that can be used in MATLAB. If the output argument MSGLIST is defined,
%   return the list of message types as a cell array of strings.
%   Simplified form: ROSMSG list
%
%
%   Example:
%      % List all available message types
%      ROSMSG list
%
%      % Retrieve message definition of the sensor_msgs/Image type
%      msgInfo = ROSMSG('show', 'sensor_msgs/Image')

%   Copyright 2020 The MathWorks, Inc.

try
    if nargout == 0
        rosmsgImpl(op, varargin{:});
    else
        output = rosmsgImpl(op, varargin{:});
    end
catch ex
    % Save stack traces and exception causes internally, but do not
    % print them to the console
    rosex = ros.internal.ROSException.fromException(ex);
    throwAsCaller(rosex);
end
end

function output = rosmsgImpl(op, varargin)
%rosmsgImpl Actual implementation of rosmsg functionality

% Convert to char to ensure that "" is flagged as empty
op = convertStringsToChars(op);
if nargin > 1
    [varargin{:}] = convertStringsToChars(varargin{:});
end

persistent parser
if isempty(parser)
    parser = inputParser;
    addRequired(parser, 'op', @(x) validateattributes(x, {'char','string'}, {'nonempty'}, 'rosmsg', 'op'));
    addOptional(parser, 'type', '', @(x) validateattributes(x, {'char','string'}, {'nonempty'}, 'rosmsg', 'type'));
end
parse(parser, op, varargin{:});

operation = parser.Results.op;
messageType = parser.Results.type;

switch(lower(operation))
    case {'info', 'show'}
        % Print out or return message definition information.
        msgDef = rosmsgShow( messageType );
        if nargout == 0
            disp(msgDef);
        else
            output = msgDef;
        end
        
    case 'list'
        % List of messages is retrieved from available message types in the
        % rostype class.
        
        msgs = rosmsgList;
        if nargout == 0
            cellStringPrint(msgs);
        else
            output = msgs;
        end
        
    case 'md5'
        % Print out or return message MD5 checksum
        msgMD5 = rosmsgMD5( messageType );
        if nargout == 0
            disp(msgMD5);
        else
            output = msgMD5;
        end
        
    otherwise
        error(message('ros:mlros:node:OperationNotSupported', ...
            operation));
end
end

function msgDef = rosmsgShow(messageType)
%rosmsgShow Retrieve the message definition for a type

% Display message information
try
    [msgPkgName,msgFileName] = fileparts(messageType);
    
    msgPath = fullfile(matlabroot, 'sys', 'ros1', computer('arch'), 'ros1', 'share');
    customMsgPath = fullfile(matlabroot, 'toolbox', 'ros', 'mlroscpp', ...
        'custom_messages', 'share');
    reg = ros.internal.CustomMessageRegistry.getInstance('ros');
    customMsgInfo = reg.getMessageInfo(messageType);
    if isempty(customMsgInfo)
        if isequal(endsWith(messageType,'Request'),1)
            dummyMessageType = messageType(1:end-7);
            if ismember(dummyMessageType,rostype.getServiceList)
                type = 'Request';
                [msgPkgName,msgFileName] = fileparts(dummyMessageType);
                location = ros.internal.utilities.locateMessage(msgPkgName,msgFileName,{msgPath, customMsgPath},'srv');
            else
                type = 'msg';
                [msgPkgName,msgFileName] = fileparts(messageType);
                location = ros.internal.utilities.locateMessage(msgPkgName,msgFileName,{msgPath, customMsgPath},'msg');
            end
        elseif isequal(endsWith(messageType,'Response'),1)
            dummyMessageType = messageType(1:end-8);
            if ismember(dummyMessageType,rostype.getServiceList)
                type = 'Response';
                [msgPkgName,msgFileName] = fileparts(dummyMessageType);
                location = ros.internal.utilities.locateMessage(msgPkgName,msgFileName,{msgPath, customMsgPath},'srv');
            else
                type = 'msg';
                [msgPkgName,msgFileName] = fileparts(messageType);
                location = ros.internal.utilities.locateMessage(msgPkgName,msgFileName,{msgPath, customMsgPath},'msg');
            end
        else
            type = 'msg';
            location = ros.internal.utilities.locateMessage(msgPkgName,msgFileName,{msgPath, customMsgPath},'msg');
        end
    else
        type = 'msg';
        location = customMsgInfo.srcPath;
    end
    msgDef = fileread(location);
    if isequal(type,'Request') || isequal(type,'Response')
        if contains(msgDef,'---')
            split = strsplit(msgDef,'---');
            [msgDefRequest,msgDefResponse] = split{:};
        else
            msgDefRequest = msgDef;
            msgDefResponse = '';
        end
    end
    
    % Replace all property names with their MATLAB property equivalent
    msgInfo = ros.internal.ros.getMessageInfo(messageType);
    if msgInfo.custom
        curpath = path;
        clnup = onCleanup(@(x)path(curpath));
        addpath(fullfile(msgInfo.installDir,'m'));
    end
    msgClass = msgInfo.msgClassGen;
    propList = eval([msgClass '.PropertyList']);
    rosPropList = eval([msgClass '.ROSPropertyList']);
    
    % Convert message definition to MATLAB standards
    if isequal(type,'Request')
        msgDef = ros.msg.internal.formatMessageDefinition(msgDefRequest, rosPropList, propList);
    elseif isequal(type,'Response')
        msgDef = ros.msg.internal.formatMessageDefinition(msgDefResponse, rosPropList, propList);
    else
        msgDef = ros.msg.internal.formatMessageDefinition(msgDef, rosPropList, propList);
    end
    
    
catch ex
    newEx = ros.internal.ROSException( ...
        message('ros:mlros:message:DefinitionNotFound', messageType));
    throw(newEx.addCustomCause(ex));
end

end

function msgMD5 = rosmsgMD5(messageType)
%rosmsgMD5 Retrieve the message MD5 checksum for a type

try
    % Instantiate message class and return MD5 hash
    msgInfo = ros.internal.ros.getMessageInfo(messageType);
    msgMD5 = eval([msgInfo.msgClassGen '.MD5Checksum']);
catch ex
    newEx = ros.internal.ROSException( message( ...
        'ros:mlros:message:DefinitionNotFound', messageType));
    throw(newEx.addCustomCause(ex));
end
end

function msgList = rosmsgList
%rosmsgList Get a list of all available message types

msgList = rostype.getMessageList;
end

function cellStringPrint(list)
%cellStringPrint Print a cell array of strings to the console

for item = list
    disp(char(item));
end
end

% LocalWords:  MSGINFO MSGMD MSGLIST
