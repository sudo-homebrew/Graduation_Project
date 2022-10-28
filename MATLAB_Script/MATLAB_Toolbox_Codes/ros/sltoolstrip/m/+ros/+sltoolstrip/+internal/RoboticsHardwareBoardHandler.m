classdef RoboticsHardwareBoardHandler < coder.internal.toolstrip.HardwareBoardHandler
%   Implementation of the Hardware Board Handler specific to Custom HW tab 

%   Copyright 2018-2019 The MathWorks, Inc.
    
    methods(Access=public)
        function obj = RoboticsHardwareBoardHandler(TargetName, Model)
            obj = obj@coder.internal.toolstrip.HardwareBoardHandler(TargetName, Model);
        end
        
        function available = isSignalLoggingAvailable(~)
            % This API should support both model name and model handle
            available = false;
        end
    end
    %
    % These methods are target dependent
    %
    methods(Access=protected)
        function val = isTargetConnected(~)
            % Assuming that the target is always connected
            val = true;
        end
        
        function val = isTargetDeployed(this)
            % If built, assuming that the custom target is also deployed
            % -> no need to deploy it
            val = this.isExtModeBuilt();
        end
    end
end
