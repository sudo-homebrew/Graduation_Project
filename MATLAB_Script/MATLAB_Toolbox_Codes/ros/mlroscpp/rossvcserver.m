function server = rossvcserver(name, type, varargin)
%ROSSVCSERVER Create a ROS service server object
%   SERVER = ROSSVCSERVER(SVCNAME,SVCTYPE) creates and returns a service
%   server object. The service will be available in the ROS network
%   through its name SVCNAME and has the type SVCTYPE. However, the service
%   object cannot respond to service requests until you specify a function
%   handle callback. SVCNAME and SVCTYPE are string scalars.
%
%   SERVER = ROSSVCSERVER(SVCNAME,SVCTYPE,CB) creates and returns
%   a service server object with SVCNAME and SVCTYPE. It also specifies
%   the function handle callback, CB, that constructs a response when
%   the server receives a request. CB can be a single function handle
%   or a cell array. The first element of the cell array must be a
%   function handle, or a string containing the name of a function.
%   The remaining elements of the cell array can be arbitrary user data
%   that is passed to the callback function.
%
%   SERVER = ROSSVCSERVER(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments.
%
%      "DataFormat" - Determines format of ROS message provided to the
%                     NewRequestFcn callback.
%                     Using structs can be faster than using message
%                     objects.
%                     Options are:
%                        "object" - Message object of the specified type
%                        "struct" - Message struct with compatible fields
%                     Default: "object"
%
%   Use ROSSVCSERVER to create a ROS service server that can receive
%   requests from, and send responses to, a ROS service client.
%   The service server must exist before creating the service client.
%   When the client is created, it establishes a connection to the
%   server. The connection persists while both client and server exist
%   and can reach each other.
%
%   The service server callback function requires at least three input
%   arguments and one output. The first argument, SRC, is the associated
%   service server object. The second argument, REQUEST, is the request
%   message object sent by the service client. The third argument is
%   the default response message object, DEFAULTRESPONSE.  Use the
%   DEFAULTRESPONSE as a starting point for constructing the function
%   output RESPONSE, which will be sent back to the service client
%   after the function returns.
%
%      function RESPONSE = serviceCallback(SRC, REQUEST, DEFAULTRESPONSE)
%          RESPONSE = DEFAULTRESPONSE;
%          % Build the response message here
%      end
%
%   While setting the callback, to construct a callback that accepts
%   additional parameters, use a cell array that includes both the
%   function handle callback and the parameters.
%
%
%   Example:
%
%      % Create a service server
%      % This assumes that there is an existing node object
%      % Use struct messages for better performance
%      server = ROSSVCSERVER("/gazebo/get_model_state",...
%         "gazebo_msgs/GetModelState","DataFormat","struct");
%
%      % Assign a callback for incoming service calls
%      server.NewRequestFcn = @fcn1;
%
%      % Create a new service server with a different name and assign a
%      % callback with user data in the constructor
%      % Use message objects with this server
%      userData = randi(20);
%      server2 = ROSSVCSERVER("/gazebo/get_model_state2",...
%          "gazebo_msgs/GetModelState",{@fcn2, userData},...
%          "DataFormat","object");
%
%   See also ros.ServiceServer, ROSSVCCLIENT, ROSSERVICE.

%   Copyright 2014-2021 The MathWorks, Inc.
%#codegen

    if isempty(coder.target)
        try
            server = ros.ServiceServer([], name, type, varargin{:});
        catch ex
            % Save stack traces and exception causes internally, but do not
            % print them to the console
            rosex = ros.internal.ROSException.fromException(ex);
            throwAsCaller(rosex);
        end
    else
        coder.internal.narginchk(1,5, nargin);
        % Ensure serviceType is not empty
        coder.internal.assert(nargin>2 && contains(type,'/'),'ros:mlroscpp:codegen:MissingMessageType',name,'ServiceServer');
        server = ros.ServiceServer([], name, type, varargin{:});
    end
end
