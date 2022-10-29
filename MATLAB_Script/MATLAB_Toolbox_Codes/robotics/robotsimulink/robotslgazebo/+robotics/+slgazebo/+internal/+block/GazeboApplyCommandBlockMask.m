classdef GazeboApplyCommandBlockMask < robotics.slgazebo.internal.block.MsgTypeSelectionMask
    %This class is for internal use only. It may be removed in the future.
    
    %ApplyCommandBlockMask - Block mask callbacks for Apply Command block
    
    %   Copyright 2019 The MathWorks, Inc.
    
    properties
        
        %BusUtil Create the bus util object
        BusUtil = robotics.slcore.internal.bus.Util
        
    end    
    
    methods
              
        function updateSubsystem(obj,msgType,block)
            
            sigspec_block = [block '/SignalSpecification'];
            busNamePrefix = obj.BusUtil.BusNamePrefix;
            
            [busDataType, ~] = robotics.slcore.internal.bus.Util.messageTypeToDataTypeStr(msgType, bdroot(block),busNamePrefix);
            
            set_param(sigspec_block, 'OutDataTypeStr', busDataType);
            
        end
        
    end
    
    methods(Static)
        
        function dispatch(methodName, varargin)
            obj = robotics.slgazebo.internal.block.GazeboApplyCommandBlockMask();
            obj.(methodName)(varargin{:});
        end
        
    end
end
