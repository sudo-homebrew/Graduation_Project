function msg = rosmessage(messageType, varargin)
%ROSMESSAGE Create ROS messages
%   MSG = ROSMESSAGE(MESSAGETYPE) creates a message compatible with
%   ROS messages of type MESSAGETYPE. MESSAGETYPE is a string scalar.
%
%   MSG = ROSMESSAGE(OBJ) creates and returns an empty message MSG
%   compatible with OBJ, which can be a ROS Publisher, Subscriber,
%   ServiceClient, ServiceServer, or SimpleActionClient. The output
%   message type and data format matches the input object.
%
%   MSG = ROSMESSAGE(MESSAGETYPE,"DataFormat",FORMAT) determines the format
%   of the returned message, specified as "object" or "struct". The "struct"
%   format is typically faster, but does not validate message field data
%   when set. FORMAT must match the DataFormat property of objects that
%   send or receive the message on the ROS network. (Default: "object").
%
%
%   Example:
%
%      % Create a pose message object
%      poseMsg = ROSMESSAGE("geometry_msgs/Pose2D")
%
%      % Create a path message struct
%      pathMsg = ROSMESSAGE("nav_msgs/Path","DataFormat","struct")
%
%      % Connect to ROS network
%      rosinit
%
%      % Create publisher and compatible message struct
%      % Use struct message format for better performance
%      % Message will match the supplied publisher's format
%      chatPub = rospublisher("/chatter","std_msgs/String","DataFormat","struct");
%      chatMsg = ROSMESSAGE(chatPub)
%
%      % Create subscriber and compatible message object
%      laserSub = rossubscriber("/scan","sensor_msgs/LaserScan","DataFormat","object");
%      laserMsg = ROSMESSAGE(laserSub)
%
%      % Disconnect from the network and remove all publishers and subscribers
%      rosshutdown
%
%   See also ROSMSG.

%   Copyright 2014-2020 The MathWorks, Inc.
%#codegen
    coder.extrinsic(...
        'ros.codertarget.internal.getEmptyCodegenMsg',...
        'rostype.getServiceList');
    coder.internal.narginchk(1, 3, nargin)
    messageType = convertStringsToChars(messageType);
    validateattributes(messageType, {'char', 'string'}, {'nonempty'}, ...
                       'rosmessage', 'messageType');

    % Parse NV pairs
    nvPairs = struct('DataFormat', uint32(0));
    pOpts = struct('PartialMatching', true, 'CaseSensitivity', false);
    pStruct = coder.internal.parseParameterInputs(nvPairs, pOpts, varargin{:});
    dataFormat = coder.internal.getParameterValue(pStruct.DataFormat, 'object', varargin{:});
    dataFormat = validatestring(dataFormat, {'object', 'struct'}, 'rosmessage', 'DataFormat');

    if isempty(coder.target)
        % Interpreted execution in MATLAB
        try
            if ismember(messageType, rostype.getServiceList)
                messageType = [messageType 'Request'];
            end
            [msg, ~, msgInfo] = ...
                ros.internal.getEmptyMessage(messageType, 'ros');
            if ~strcmp(dataFormat, 'struct')
                msg = feval(msgInfo.msgClassGen, msg);
            end
        catch ex
            if strcmp(ex.identifier, 'MATLAB:undefinedVarOrClass')
                error(message('ros:utilities:message:MessageNotFoundError', ...
                              messageType, 'rosmsg list'))
            else
                rethrow(ex)
            end
        end
    else
        % Codegen
        coder.internal.assert(strcmp(dataFormat, 'struct'),...
                              'ros:mlroscpp:codegen:InvalidDataFormat', 'rosmessage');
        msgStructGenFcnName = coder.const(@ros.codertarget.internal.getEmptyCodegenMsg,messageType,'ros');
        msgStructGenFcn = str2func(msgStructGenFcnName);
        msg = msgStructGenFcn();
    end
end
