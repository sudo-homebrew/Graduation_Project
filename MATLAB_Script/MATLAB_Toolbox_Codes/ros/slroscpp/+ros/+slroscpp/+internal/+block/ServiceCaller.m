classdef ServiceCaller < ros.slros.internal.block.ServiceCallerBase
    
    
    %This class is for internal use only. It may be removed in the future.
    
    %ServiceCaller Call service on ROS network and receive response
    %
    %   H = ros.slros.internal.block.ServiceCaller creates a system
    %   object, H, that sends a request to a service server on the ROS network and
    %   outputs the response message received
    %
    %   This system object is intended for use with the MATLAB System
    %   block. In order to access the ROS functionality from MATLAB, see
    %   ROSSVCCLIENT
    %
    %   See also rossvcclient.
    
    %   Copyright 2018-2020 The MathWorks, Inc.
    
    %#codegen
    
    properties (Access = ?matlab.unittest.TestCase, Transient)
        %InputConverter - Conversion for service request bus
        InputConverter = ros.slroscpp.internal.sim.BusStructToROSMsgConverter.empty
        
        %OutputConverter - Conversion for service response bus
        OutputConverter = ros.slroscpp.internal.sim.ROSMsgToBusStructConverter.empty
    end
    
    methods
        function obj = ServiceCaller(varargin)
            %ServiceCaller Standard constructor
            
            % Support name-value pair arguments when constructing the object.
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods (Access = protected)
        %% Overloaded system object functions
        function setConverters(obj)
            % Create the bus converters
            obj.InputConverter = ros.slroscpp.internal.sim.BusStructToROSMsgConverter(...
                strcat(obj.ServiceType, 'Request'), obj.ModelName);
            obj.OutputConverter = ros.slroscpp.internal.sim.ROSMsgToBusStructConverter(...
                strcat(obj.ServiceType, 'Response'), obj.ModelName);
        end
    end
    
end

% LocalWords:  slros
