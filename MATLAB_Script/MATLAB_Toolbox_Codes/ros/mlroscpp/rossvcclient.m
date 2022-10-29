function [client, reqMsg] = rossvcclient(name, varargin)
%ROSSVCCLIENT Create a ROS service client
%   CLIENT = ROSSVCCLIENT(SVCNAME) creates a service client that
%   connects to, and gets its service type from, a service server.
%   This command syntax blocks the current MATLAB program from running
%   until it can connect to the service server. SVCNAME is a string scalar.
%
%   CLIENT = ROSSVCCLIENT(SVCNAME,SVCTYPE) creates a service
%   client of type SVCTYPE regardless of whether a service server offering
%   SVCNAME is available. If SVCNAME already exists in the network,
%   SVCTYPE must match the service type of SVCNAME. The service client
%   will be attached to the ros.Node object, NODE. SVCNAME is a string scalar.
%
%   CLIENT = ROSSVCCLIENT(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments. You can specify
%   several name-value pair arguments in any order as
%   Name1,Value1,...,NameN,ValueN:
%
%      "DataFormat" - Determines format of ROS message to be used by
%                     the service client, and returned from rosmessage.
%                     Using structs can be faster than using message
%                     objects.
%                     Options are:
%                        "object" - Message object of the specified type
%                        "struct" - Message struct with compatible fields
%                     Default: "object"
%
%      "Timeout"    - Specifies a timeout period, in seconds. If the
%                     service client does not connect to the service server
%                     and the timeout period elapses, ros.ServiceClient displays
%                     an error message and lets MATLAB continue running
%                     the current program. Otherwise, the default value is Inf,
%                     which blocks MATLAB from running the current
%                     program until the service client receives a
%                     service response.
%
%   [CLIENT,REQUEST] = ROSSVCCLIENT(___) returns a new service request
%   message in REQUEST. The message type of REQUEST is determined by the
%   service that CLIENT is connected to. The message will be initialized with
%   default values. The format of REQUEST is determined by the DataFormat
%   of the service client.
%
%   Use ROSSVCCLIENT to create a ROS service client. This service client
%   uses a persistent connection to send requests to, and receive
%   responses from, a ROS service server. The connection persists until
%   the service client is deleted or the service server becomes
%   unavailable.
%
%
%   Example:
%
%      % Create a service client and wait to connect to the service
%      % server (blocking). This assumes that there is an existing node object
%      % Use struct message format for better performance
%      client = ROSSVCCLIENT("/gazebo/get_model_state",...
%          "DataFormat","struct");
%
%      % Create the service request message
%      request = rosmessage(client);
%      request.ModelName = 'SomeModel';
%
%      % Send the service request and wait for a response (blocking)
%      response = call(client,request);
%
%      % Send the service request and wait five seconds for a response
%      % (blocking, with timeout).
%      response = call(client,request,"Timeout",5);
%
%      % Create and call with another service client that uses message objects
%      % Wait three seconds to connect to the service server (blocking, with timeout)
%      client2 = ROSSVCCLIENT("/gazebo/get_model_state",...
%          "Timeout",3,"DataFormat","object");
%      requestObj = rosmessage(client2)
%      responseObj = call(client2,requestObj);
%
%   See also ros.ServiceClient, ROSSVCSERVER, ROSSERVICE.
%

%   Copyright 2014-2021 The MathWorks, Inc.
%#codegen

    if isempty(coder.target)
        try
            client = ros.ServiceClient([], name, varargin{:});

            % Assign output message, if requested
            if nargout > 1
                reqMsg = rosmessage(client);
            end
        catch ex
            % Save stack traces and exception causes internally, but do not
            % print them to the console
            rosex = ros.internal.ROSException.fromException(ex);
            throwAsCaller(rosex);
        end
    else
        coder.internal.narginchk(1,6,nargin);
        client = ros.ServiceClient([], name, varargin{:});

        % Assign output message, if requested
        if nargout > 1
            reqMsg = rosmessage(client);
        end
    end
end
