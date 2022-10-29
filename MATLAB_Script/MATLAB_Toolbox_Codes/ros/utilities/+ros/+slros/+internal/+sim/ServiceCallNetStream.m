classdef  ServiceCallNetStream < ros.slros.internal.sim.ServiceCallStream
%This class is for internal use only. It may be removed in the future.

%ServiceCallNetStream Call a service on the ROS service

%  Copyright 2018-2020 The MathWorks, Inc.

    properties
        %ServiceName - Name of service
        ServiceName = "/my_service"

        %ConnectionTimeout - Timeout for connection
        ConnectionTimeout = 5

        %IsConnectionPersistent - Indicate connection persistence
        IsConnectionPersistent = false
    end

    properties (Access = ?matlab.unittest.TestCase)
        %Client - Service client
        Client = []

        %Node - ROS node used to connect to network
        Node ros.Node
    end


    methods
        function obj = ServiceCallNetStream(nodeObj)
        %ServiceCallNetStream Standard constructor

            if nargin == 1
                % Use the pass node object
                obj.Node = nodeObj;
            else
                % Use the global node
                obj.Node = ros.internal.Global.getNodeHandle(false);
            end
        end

        function [resp, errorCode] = callService(obj, req)
        %callService Call the service

            import ros.slros.internal.block.ServiceCallErrorCode

            % Default returns
            resp = [];
            errorCode = uint8(ServiceCallErrorCode.SLSuccess);

            % Create the service client.
            if isempty(obj.Client) || ~isvalid(obj.Client)
                [client, errorCode] = connectToService(obj);

                % Return if the creation failed
                if errorCode ~= uint8(ServiceCallErrorCode.SLSuccess)
                    return;
                end
            else
                client = obj.Client;
            end

            % Execute the service call
            try
                resp = call(client, req);
            catch ME
                errorCode = uint8(ServiceCallErrorCode.SLOtherError);
                if strcmp(ME.identifier, 'ros:mlros:serviceclient:CallFailure')
                    errorCode = uint8(ServiceCallErrorCode.SLCallFailure);
                end
            end

            % Preserve client for next invocation
            if obj.IsConnectionPersistent
                obj.Client = client;
            end
        end
    end

    methods (Access = private)
        function [client, errorCode] = connectToService(obj)
        %connectToService Connect to service server

            import ros.slros.internal.block.ServiceCallErrorCode

            client = [];
            errorCode = uint8(ServiceCallErrorCode.SLSuccess);

            try
                % TODO: Remove conditional and just keep DataFormat,struct (g2199002)
                if contains(which('ros.ServiceClient'), 'mlroscpp')
                    client = ros.ServiceClient(obj.Node, char(obj.ServiceName), ...
                                               'Timeout', obj.ConnectionTimeout, ...
                                               'DataFormat', 'struct');
                else
                    client = ros.ServiceClient(obj.Node, char(obj.ServiceName), ...
                                               'Timeout', obj.ConnectionTimeout);
                end
            catch ME
                errorCode = uint8(ServiceCallErrorCode.SLOtherError);

                if strcmp(ME.identifier, 'ros:mlros:serviceclient:ConnectWaitTimeout')
                    errorCode = uint8(ServiceCallErrorCode.SLConnectionTimeout);
                end
            end
        end
    end

end
