function output = rosservice( op, varargin )
%ROSSERVICE Access information about services in the ROS network
%   LS = ROSSERVICE('list') returns a list of service names for all of
%   the active service servers on the ROS network. If specified, the
%   output argument, LS, contains a cell array of service names.
%   Simplified form: ROSSERVICE list
%
%   SVCINF = ROSSERVICE('info', 'SVCNAME') returns information about
%   the service whose name is SVCNAME. If specified, the output argument,
%   SVCINF, contains the service information as a structure.
%   Simplified form: ROSSERVICE info SVCNAME
%
%   SVCTYPE = ROSSERVICE('type', 'SVCNAME') returns the service type of
%   the service whose name is SVCNAME. If specified, the output argument,
%   SVCTYPE, contains the service type as a string.
%   Simplified form: ROSSERVICE type SVCNAME
%
%   SVCURI = ROSSERVICE('uri', 'SVCNAME') returns the URI of the service
%   whose name is SVCNAME. If specified, the output argument, SVCURI,
%   contains the service URI as a string.
%   Simplified form: ROSSERVICE uri SVCNAME
%
%   Use ROSSERVICE to get a list of services on the ROS network, or to
%   get information about a particular service.
%
%
%   Example:
%      % Show the list of all active services on the ROS network
%      ROSSERVICE list
%
%      % Get type and URI of service '/some_service'
%      ROSSERVICE type /some_service
%      ROSSERVICE uri /some_service
%
%      % Get more detailed information of the service in a structure
%      svcInfo = ROSSERVICE('info', '/some_other_service');

%   Copyright 2014-2020 The MathWorks, Inc.

    try
        if nargout == 0
            rosserviceImpl(op, varargin{:});
        else
            output = rosserviceImpl(op, varargin{:});
        end
    catch ex
        % Save stack traces and exception causes internally, but do not
        % print them to the console
        rosex = ros.internal.ROSException.fromException(ex);
        throwAsCaller(rosex);
    end
end

function output = rosserviceImpl(op, varargin)
%rosserviceImpl Actual implementation of rosservice functionality.

% Use validatestring to ensure string and char inputs to 'operation' are
% supported.
    try
        supportedOperations = {'list', 'info', 'type', 'uri'};
        operation = validatestring(op, supportedOperations, 'rosnode', 'operation');
    catch
        error(message('ros:mlros:service:OperationNotSupported', op));
    end

    if nargin > 1
        % Convert strings to characters to ensure that, together with
        % validateattributes, "" is flagged as invalid input.
        [varargin{:}] = convertStringsToChars(varargin{:});
    end

    % Parse the input to the function
    parser = getParser;
    parse(parser, varargin{:});
    argument = parser.Results.arg;

    switch lower(operation)
      case 'list'
        % Display list of available services
        serviceNames = rosserviceList;
        if nargout == 1
            output = serviceNames;
            return;
        end

        % If no output argument specified, print to terminal
        for i = 1:size(serviceNames, 1)
            disp(char(serviceNames{i,1}));
        end

      case 'info'
        % Retrieve information about a specific service

        serviceName = argument;

        % This will throw an error if service name doesn't exist
        info = ros.internal.NetworkIntrospection.getServiceInfo(serviceName, []);

        % If output argument specified, return
        if nargout == 1
            output = info;
            return;
        end

        % Display the info structure
        rosserviceInfoPrint( info );

      case 'type'
        % Retrieve the type of a particular service

        serviceName = argument;
        type = rosserviceType(serviceName);

        if nargout == 1
            output = type;
            return;
        end

        % If no output argument specified, print to terminal
        disp(type);

      case 'uri'
        % Retrieve the URI of a particular service

        serviceName = argument;
        uri = rosserviceURI(serviceName);

        if nargout == 1
            output = uri;
            return;
        end

        % If no output argument specified, print to terminal
        disp(uri);
    end

    function parser = getParser()
        persistent p;
        if isempty(p)
            p = inputParser;
            addOptional(p, 'arg', '/', @(x) validateattributes(x, {'char'}, {'nonempty'}, 'rosservice', 'arg'));
        end

        parser = p;
    end
end

function serviceNames = rosserviceList
%rosserviceList Retrieve a list of all available services

    serviceNames = ros.internal.NetworkIntrospection.getServiceList([]);
    serviceNames = sortrows(serviceNames, 1);
end

function type = rosserviceType(serviceName)
%rosserviceType Retrieve the type of a particular service by name
%   This function will throw an error if a service of that name does
%   not exist.

    type = ros.internal.NetworkIntrospection.getServiceType(serviceName, []);

    if isempty(type)
        error(message('ros:mlros:service:ServiceNameNotFound', serviceName));
    end
end

function uri = rosserviceURI(serviceName)
%rosserviceURI Retrieve the URI of a particular service by name
%   This function will throw an error if a service of that name does
%   not exist.

    uri = ros.internal.NetworkIntrospection.getServiceURI(serviceName, []);

    if isempty(uri)
        error(message('ros:mlros:service:ServiceNameNotFound', serviceName));
    end
end

function rosserviceInfoPrint(info)
%rosserviceInfoPrint Print the ROS service info structure to console

    disp([message('ros:mlros:service:RosserviceNode').getString ': ' info.Node]);
    disp([message('ros:mlros:service:RosserviceURI').getString ': ' info.URI]);
    disp([message('ros:mlros:service:RosserviceType').getString ': ' info.Type]);
    argsStr = [];
    for i = 1:length(info.Args)
        argsStr = [argsStr ' ' info.Args{i}]; %#ok<AGROW>
    end
    disp([message('ros:mlros:service:RosserviceArgs').getString ':' argsStr]);
end
