classdef FastSubscriber < ros.Subscriber
    %This class is for internal use only. It may be removed in the future.
    
    %FastSubscriber Subscribe to messages on a topic without safety checks
    
    %   Copyright 2020 The MathWorks, Inc.
    
    methods
        function obj = FastSubscriber(varargin)
            obj = obj@ros.Subscriber(varargin{:});
        end
    end
        
    methods (Access = protected)
        function checkMessageType(varargin)
            %checkMessageType Intentionally blank
        end

        function waitForMasterRegistration(varargin)
            %waitForMasterRegistration Intentionally blank
        end

        function waitForUnregistration(varargin)
            %waitForUnregistration Intentionally blank
        end
    end
end
