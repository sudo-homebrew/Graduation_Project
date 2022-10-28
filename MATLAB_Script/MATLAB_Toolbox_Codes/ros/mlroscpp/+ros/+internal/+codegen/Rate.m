classdef Rate < ros.internal.mixin.ROSInternalAccess & coder.ExternalDependency
% This class is for internal use only. It may be removed in the future.
%
% Rate - Code generation equivalent for ros.Rate
% See also ros.Rate
%
%#codegen

% Copyright 2020 The MathWorks, Inc.

    properties
        %RateHelper - helper member for ros::Rate
        RateHelper
        
        %DesiredRate - Desired execution rate (Hz)
        DesiredRate
        
        %DesiredPeriod - Desired time period between executions (seconds)
        DesiredPeriod
    end
    
    properties(Access=protected)
        %StartTime - Start time at construction or reset (seconds)
        StartTime
        
        %PreviousPeriod - Time at previous waitfor call
        PreviousPeriod
        
        %IsReset - Indicator on whether ros::Rate get reset
        IsReset
        
        %InternalLastPeriod - Internal Elapsed time between last two waitfor calls (seconds)
        InternalLastPeriod
    end
    
    properties(SetAccess = protected)
        %OverrunAction - Action used for handling overruns
        OverrunAction
    end
    
    properties(Dependent, SetAccess = protected)
        %IsSimulationTime - Indicate if simulation time is used
        %   This property will be "true" if the "/use_sim_time" ROS
        %   parameter was set to true when the parent node was launched.
        IsSimulationTime
        
        %TotalElapsedTime - Elapsed time since construction or reset (seconds)
        TotalElapsedTime
        
        %LastPeriod - Elapsed time between last two waitfor calls (seconds)
        LastPeriod
    end

    methods
        function obj = Rate(node, desiredRate)
        %Rate - Constructor for Rate object
        %   Please see the class documentation for more details.
        %   See also: ros.Rate

            coder.inline('never')
            % Ensure input arguments are as expected
            if ~isempty(node)
                % A node cannot create another node in code generation
                coder.internal.assert(false,'ros:mlroscpp:codegen:NodeMustBeEmpty');
            end

            % The input argument desiredRate must be a numeric scalar
            validateattributes(desiredRate,{'numeric'},{'scalar'},...
                               'ros.Rate','desiredRate');

            coder.cinclude('ros/ros.h');
            obj.RateHelper = coder.opaque('std::unique_ptr<MATLABRate>','HeaderFile','mlroscpp_rate.h');
            obj.RateHelper = coder.ceval('MATLABRate_create',desiredRate);
            coder.ceval('MATLABRate_unused',coder.wref(obj.RateHelper));
            
            obj.DesiredRate = desiredRate;
            obj.DesiredPeriod = 1.0/obj.DesiredRate;
            obj.StartTime = tic;
            obj.PreviousPeriod = tic;
            obj.IsReset = true;
            obj.OverrunAction = 'slip';
            obj.InternalLastPeriod = NaN;
        end

        function waitfor(obj)
        %WAITFOR - Pause the code execution to achieve desired execution rate
        %   Please see the class documentation for more details.
        %   See also: rateControl

            coder.ceval('MATLABRate_sleep',obj.RateHelper);
            
            % Update PreviousPeriod and InternalLastPeriod
            obj.InternalLastPeriod = toc(obj.PreviousPeriod);
            obj.PreviousPeriod = tic;
            
            % LastPeriod becomes available again once we call waitfor after
            % reset
            obj.IsReset = false;
        end

        function reset(obj)
        %RESET - Sets the start time for the rate to now.

            coder.ceval('MATLABRate_reset',obj.RateHelper);
            obj.StartTime = tic;
            obj.IsReset = true;
        end

        function statistics(~)
        %STATISTICS - statistics of past execution periods
        %   This method does not support codegen. Please see the class
        %   documentation for more details.
        %   See also: rateControl

            coder.internal.assert(false, 'ros:mlroscpp:codegen:UnsupportedMethodCodegen', ...
                                  'statistics');
        end
        
        function isSimTime = get.IsSimulationTime(obj)
        %get.IsSimulationTime - getter for IsSimulationTime

            isSimTime = false;
            isSimTime = coder.ceval('MATLABRate_isSimTime',obj.RateHelper);
        end
        
        function totalElapsedTime = get.TotalElapsedTime(obj)
        %get.TotalElapsedTime - getter for TotalElapsedTime

            totalElapsedTime = toc(obj.StartTime);
        end
        
        function lastPeriod = get.LastPeriod(obj)
        %get.LastPeriod - getter for LastPeriod
        
            % Ensure RateHelper is not optimized away
            if obj.IsReset
                lastPeriod = NaN;
            else
                lastPeriod = obj.InternalLastPeriod;
            end
        end
    end

    methods (Static)        
        function ret = getDescriptiveName(~)
            ret = 'ROS Rate';
        end

        function ret = isSupportedContext(bldCtx)
            ret = bldCtx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo, bldCtx)
            if bldCtx.isCodeGenTarget('rtw')
                srcFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
                addIncludeFiles(buildInfo,'mlroscpp_rate.h',srcFolder);
            end
        end
    end
end
