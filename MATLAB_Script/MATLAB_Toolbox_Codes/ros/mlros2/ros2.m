function varargout = ros2(varargin)
% ros2 Retrieve information about ROS 2 network
%
%   MSGLIST = ros2("msg","list") lists all available ROS 2 message types that
%   can be used in MATLAB. Simplified form:
%   ros2 msg list
%
%   MSGINFO = ros2("msg","show","TYPE") provides the definition of the ROS 2
%   message TYPE. Simplified form:
%   ros2 msg show TYPE
%
%   NODELIST = ros2("node","list") lists nodes on the ROS 2 network.
%   Simplified form:
%   ros2 node list
%
%   TOPICLIST = ros2("topic","list") lists topic names that are currently
%   registered on ROS 2 network through either publishers or subscribers.
%   Simplified form:
%   ros2 topic list
%
%   SVCLIST = ros2("service","list") lists service names that are currently
%   registered on the ROS 2 network through either servers or clients.
%   Simplified form:
%   ros2 service list
%
%   SERVICETYPES = ros2("service","type",SVCNAME) lists service types that
%   are currently registered on the ROS 2 network for the provided SVCNAME.
%   Simplified form:
%   ros2 service type SVCNAME
%
%   NODELIST = ros2("node","list","DomainID",ID) provided introspection
%   information for the specified network domain ID. By default, the
%   domain ID is 0 unless otherwise specified by the ROS_DOMAIN_ID
%   environment variable.
%
%   OUTPUT = ros2(____,"DomainID",ID) the requested information for the
%   specified network domain ID.
%
%   The DomainID name-value pair applies only to information gathered from the
%   the active network, such as the node, topic, and service lists, and not
%   to static ROS 2 data such as message information.
%
%   The first time ros2 is called for a specific domain ID, not all
%   information on the network may be immediately available. If incomplete
%   network information is returned from ros2, wait for a short time before
%   trying again.
%
%   BAG2INFO = ros2("bag","info",FOLDERPATH) returns information about the
%   contents of the ros2bag FOLDERPATH as a structure. The information
%   includes the bag file path, storage identifier, bag size, number of
%   messages, start/end time, topics, serialization format, and message types.
%   Simplified form:
%   ros2 bag info FOLDERPATH
%
%   Examples:
%
%      % List all available topics
%      ros2 topic list
%
%      % Retrieve message definition of the sensor_msgs/LaserScan type
%      msgInfo = ros2("msg","show","sensor_msgs/LaserScan")
%
%      % Show the list of all nodes on the ROS 2 network on the default
%      % domain of 0
%      ros2 node list
%
%      % Show the list of all nodes on ROS 2 network with a DomainID of 2
%      nodeList = ros2("node","list","DomainID",2)
%
%      % Create a node with a DomainID of 2
%      node2 = ros2node("/node2",2)
%
%      % The node list now shows the new node
%      nodeList = ros2("node","list","DomainID",2)
%
%   See also:
%   ros2node, ros2publisher, ros2subscriber

%   Copyright 2019-2021 The MathWorks, Inc.
    narginchk(2,nargin);
    try
        [cmd, subcmd, cmdoption, domainid] = getparameters(varargin{:});
        introspec = ros.ros2.internal.Introspection();
        result = introspec.([cmd,subcmd])(cmdoption, domainid);
        if nargout < 1
            if isequal(subcmd,'listtypes')
                disp(cell2table(result,'VariableNames',{'Topic','MessageType'}));
            elseif ~isempty(result)
                if iscell(result)
                    fprintf('%s\n', result{:,1});
                elseif isobject(result)
                    disp(result.info);
                else
                    fprintf('%s\n', result);
                end
            end
        else
            if isobject(result)
                varargout{1} = result.infoStruct;
            else
                varargout{1} = result;
            end
        end
    catch ex
        throw(ex);
    end
end

%-------------------------------------------------------------------
% Get parameters
function [cmd, subcmd, cmdoption, domainid] = getparameters(varargin)
    expectedCommands = {'node','topic','service','msg','bag','DomainID'};
    stringOfCommands = 'node, topic, service, msg, bag';
    expectedNodeCommands = {'list'};
    expectedTopicCommands = {'list'};
    expectedServiceCommands = {'list', 'type'};
    expectedMsgCommands = {'list','show'};
    expectedBagCommands = {'info'};

    cmd = [];
    subcmd = [];
    cmdoption = [];
    domainid = [];
    argsParsed = 0;
    while argsParsed < nargin
        narginchk(argsParsed+2,nargin);
        if ~argsParsed
            % Do not accept DomainID name-value pair as first argument
            opt = validatestring(varargin{argsParsed+1},expectedCommands(1:end-1),argsParsed+1);
        else
            opt = validatestring(varargin{argsParsed+1},expectedCommands,argsParsed+1);
        end
        switch opt
          case 'node'
            if ~isempty(cmd)
                error(message('ros:mlros2:message:MultipleCommands',...
                              cmd,stringOfCommands))
            end
            cmd = opt;
            subcmd = validatestring(varargin{argsParsed+2},expectedNodeCommands, argsParsed+2);
            if (nargin > argsParsed + 2) && isequal(varargin{argsParsed+3}, '-a') % i.e. we have more args and next is -a
                subcmd = 'listall';
                argsParsed = argsParsed+3;
            else
                argsParsed = argsParsed+2;
            end
          case 'topic'
            if ~isempty(cmd)
                error(message('ros:mlros2:message:MultipleCommands',...
                              cmd,stringOfCommands))
            end
            cmd = opt;
            subcmd = validatestring(varargin{argsParsed+2},expectedTopicCommands, argsParsed+2);
            if (nargin > argsParsed + 2) && isequal(varargin{argsParsed+3}, '-t') % i.e. we have more args and next is -t
                subcmd = 'listtypes';
                argsParsed = argsParsed+3;
            else
                argsParsed = argsParsed+2;
            end
          case 'service'
            if ~isempty(cmd)
                error(message('ros:mlros2:message:MultipleCommands',...
                              cmd,stringOfCommands))
            end
            cmd = opt;
            subcmd = validatestring(varargin{argsParsed+2},expectedServiceCommands, argsParsed+2);
            if (nargin > argsParsed + 2) && isequal(varargin{argsParsed+3}, '-t') % i.e. we have more args and next is -t
                subcmd = 'listtypes';
                argsParsed = argsParsed+3;
            elseif isequal(subcmd, 'type')
                narginchk(argsParsed+3,nargin)
                cmdoption = varargin{argsParsed+3};
                validateattributes(cmdoption,{'char','string'},{'nonempty'},'ros2','svctype',argsParsed+3)
                argsParsed = argsParsed+3;
            else
                argsParsed = argsParsed+2;
            end
          case 'msg'
            if ~isempty(cmd)
                error(message('ros:mlros2:message:MultipleCommands',...
                              cmd,stringOfCommands))
            end
            cmd = opt;
            subcmd = validatestring(varargin{argsParsed+2},expectedMsgCommands, argsParsed+2);
            if isequal(subcmd, 'show')
                narginchk(argsParsed+3,nargin);
                cmdoption = varargin{argsParsed+3};
                validateattributes(cmdoption,{'char','string'},{'nonempty'},'ros2','msg',argsParsed+3);
                argsParsed = argsParsed+3;
            else
                argsParsed = argsParsed+2;
            end
          case 'DomainID'
            % If cmd is not yet parsed, then we need at least two more
            if isempty(cmd)
                narginchk(argsParsed+4,nargin);
            end
            domainid = varargin{argsParsed+2};

            % Accept text input so command syntax can specify domain ID
            if ischar(domainid) || isstring(domainid)
                tempDomainID = str2double(domainid);
                if ~isnan(tempDomainID)
                    domainid = tempDomainID;
                end
            end
            validateattributes(domainid,{'numeric'}, {'scalar', 'integer', 'nonnegative'});
            argsParsed = argsParsed+2;
          case 'bag'
            if ~isempty(cmd)
                error(message('ros:mlros2:message:MultipleCommands',...
                              cmd,stringOfCommands))
            end
            cmd = opt;
            subcmd = validatestring(varargin{argsParsed+2},expectedBagCommands, argsParsed+2);
            if isequal(subcmd, 'info')
                narginchk(argsParsed+3,nargin);
                cmdoption = varargin{argsParsed+3};
                validateattributes(cmdoption,{'char','string'},{'nonempty'},'ros2','bag',argsParsed+3);
                argsParsed = argsParsed+3;
            else
                argsParsed = argsParsed+2;
            end
        end
    end
end
