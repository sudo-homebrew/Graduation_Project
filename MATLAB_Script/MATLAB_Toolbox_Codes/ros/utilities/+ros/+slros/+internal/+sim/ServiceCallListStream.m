classdef  ServiceCallListStream < ros.slros.internal.sim.ServiceCallStream
%This class is for internal use only. It may be removed in the future.

%ServiceCallListStream Getting a service response from a list
%   There is an underlying FIFO list of responses.

%   Copyright 2018-2020 The MathWorks, Inc.

    properties
        %ServiceName - Name of service
        ServiceName = "/my_service"

        %ConnectionTimeout - Timeout for connection
        ConnectionTimeout = 5

        %IsConnectionPersistent - Indicate connection persistence
        IsConnectionPersistent = false
    end

    properties (Transient)
        %ResponseList - Cell array of service responses and error codes
        %   This is a cell array with 2 columns. The first column always contains
        %   a message object of the right type and the second column
        %   contains the corresponding error code:
        %     { respMsg1, uint8(errorCode1); ...
        %       respMsg2, uint8(errorCode2);...
        %       ...
        %     }
        ResponseList = cell(0,2)
    end

    properties (Transient, SetAccess = ?matlab.unittest.TestCase)
        %LastCallRequest - Stores the last request message that was passed into callService
        LastCallRequest
    end


    methods
        function obj = ServiceCallListStream(varargin)
            if nargin < 1
                obj.LastCallRequest = ros.Message.empty(0,1);
            else
                obj.LastCallRequest = varargin{1};
            end
        end
        
        function clearResponseList(obj)
        %clearResponseList Empty list of current responses
            obj.ResponseList = cell(0,2);
        end

        function [resp, errorCode] = callService(obj, req)
        %callService Pretend to call the service

            obj.LastCallRequest = req;

            % Throw an error if there is no service response to get
            if isempty(obj.ResponseList)
                error("ros:slros:internal:ResponseListEmpty", "Cannot retrieve service response, because the ResponseList cell array is empty.");
            end

            % Otherwise, remove response from the front of the FIFO and
            % return it
            [resp, errorCode] = obj.ResponseList{1,:};
            obj.ResponseList(1,:) = [];
        end
    end


    methods
        function set.ResponseList(obj, respList)
        %set.ResponseList Setter for ResponseList property

            validateattributes(respList, "cell", {"ncols", 2}, "ServiceCallListStream", "ResponseList");
            obj.ResponseList = respList;
        end
    end

end
