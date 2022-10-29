classdef Util < handle
    %This class is for internal use only. It may be removed in the future.
    
    %Util Various utility functions
    
    %   Copyright 2020 The MathWorks, Inc.
    
    methods (Static)
        function util = getInstance()
            %getInstance Get the singleton instance of the utility class
            
            util = ros.internal.Util.instance;
        end                
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase})
        function util = instance(setUtil)
            %instance Get or set the utility class instance
            persistent utility
                        
            if nargin == 1
                utility = setUtil;
            end
            
            if isempty(utility)
                utility = ros.internal.Util;
            end
            
            util = utility;                        
        end            
    end
    
    methods (Access = private)
        function obj = Util
            %Util Private constructor to prevent explicit construction
        end
    end
        
    methods
        function waitUntilTrue(~, evalFcn, timeout)
            %waitUntilTrue Wait until function evaluates to TRUE
            %   waitUntilTrue(EVALFCN, TIMEOUT) blocks MATLAB execution
            %   until the function handle EVALFCN evaluates to TRUE or
            %   until a TIMEOUT occurs.
            %   The TIMEOUT is specified in seconds. Its default is
            %   infinity if given as empty input.
            
            validateattributes(evalFcn, {'function_handle'}, ...
                {'nonempty'}, 'waitUntilTrue', 'evalFcn');
            validateattributes(timeout, {'numeric'}, ...
                {}, 'waitUntilTrue', 'timeout');
            
            if isempty(timeout)
                timeout = Inf;
            end
            waitInterval = 0.005;
            
            startTime = clock;
            while ~feval(evalFcn)
                % If timeout occurs
                if etime(clock, startTime) > timeout
                    error(message('ros:mlros:util:WaitTimeout'));
                end
                                
                % Pause for a small interval.
                pause(waitInterval);
                
                % Have to execute drawnow to issue callbacks and to keep
                % figures responsive. Use the 'limitrate' option to ensure
                % that figure updates are limited to 20 Hz.
                drawnow('limitrate');
            end                     
        end
    end    
end

