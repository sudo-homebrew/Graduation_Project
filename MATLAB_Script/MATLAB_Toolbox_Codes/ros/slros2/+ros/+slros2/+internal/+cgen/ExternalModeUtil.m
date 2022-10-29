classdef (Hidden) ExternalModeUtil 
    %This class is for internal use only. It may be removed in the future.
    
    %ExternalModeUtil- Utility functions for external mode
    
    %   Copyright 2019-2020 The MathWorks, Inc.
    
    %#codegen
    
    methods (Static)
        function CloseFcn(hCS) %#ok<INUSD>
            %CloseFcn Close function for one click external mode
            %   This function is called under the following conditions:
            %   - The external mode model reached the end of its simulation time
            %   - The user pressed "Stop" during the external mode simulation
            %   - An error occurred during the external mode simulation

            % Renable external mode warnings
            warning('on', 'Simulink:Engine:ExtModeCannotDownloadParamBecauseNoHostToTarget');
        end
        
        function SetupFcn(hCS) %#ok<INUSD>
            %SetupFcn Setup function for one click external mode
            %   This function is called under when the user pressed
            %   "Monitor Tune" button

            % Disable external mode warnings about unsupported bus signals
            warning('off', 'Simulink:Engine:ExtModeCannotDownloadParamBecauseNoHostToTarget');
        end
    end
end

