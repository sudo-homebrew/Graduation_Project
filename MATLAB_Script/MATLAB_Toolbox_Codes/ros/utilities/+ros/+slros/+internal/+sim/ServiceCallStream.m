classdef ServiceCallStream < handle
%This class is for internal use only. It may be removed in the future.

%ServiceCallStream Interface for calling a service and getting a response

%  Copyright 2018-2020 The MathWorks, Inc.


    properties (Abstract)
        %ServiceName - Name of service
        ServiceName  string

        %ConnectionTimeout - Timeout for connection
        ConnectionTimeout double

        %IsConnectionPersistent - Indicate connection persistence
        IsConnectionPersistent logical
    end


    methods (Abstract)
        %callService Call a service with a service request
        %   This function should return 2 outputs: RESP, a ROS message
        %   representing the service response, and ERRORCODE, the error
        %   code for the previous operation.
        [resp, errorCode] = callService(obj, req)
    end

end
