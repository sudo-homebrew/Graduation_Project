classdef Rate < ...
        rateControl & ...
        ros.internal.mixin.ROSInternalAccess & ...
        robotics.core.internal.mixin.Unsaveable
    %Rate Execute a loop at a fixed frequency
    %   The Rate object allows you to run a loop at a fixed frequency.
    %   It uses the ROS node as a source for time information, so it can
    %   handle both wall clock time (system time) and ROS simulation time.
    %   If rosinit creates a ROS master in MATLAB, the global node
    %   uses wall clock time.
    %
    %   The accuracy of the rate execution is influenced by the scheduling
    %   resolution of your operating system and by the level of other
    %   system activity.
    %   The Rate object relies on the PAUSE function. If "pause('off')"
    %   is used to disable PAUSE, the rate execution will not be accurate.
    %
    %   R = ros.Rate(NODE, DESIREDRATE) creates a rate object R that executes
    %   a loop at a fixed frequency equal to DESIREDRATE. The DESIREDRATE is
    %   specified in Hz (executions per second). The OverrunAction is set to 'slip'.
    %   The time source is linked to the same time source as used by the valid
    %   ROS node object NODE. The default setting for OverrunAction is 'slip',
    %   which executes the next loop immediately if the LastPeriod is greater
    %   than DesiredPeriod.
    %
    %
    %   Rate properties:
    %      IsSimulationTime - Indicator if simulation time or wall clock time is used
    %      DesiredRate      - Desired execution rate (Hz)
    %      DesiredPeriod    - Desired time period between executions (seconds)
    %      TotalElapsedTime - Elapsed time since construction or reset (seconds)
    %      LastPeriod       - Elapsed time between last two waitfor calls (seconds)
    %      OverrunAction    - Action used for handling overruns
    %
    %   Rate methods:
    %      waitfor      - Pause the code execution to achieve desired execution rate
    %      reset        - Reset the Rate object
    %      statistics   - Returns the statistics of past execution periods
    %
    %
    %   Example:
    %       % Initialize ROS master and node
    %       rosinit;
    %       node = ros.Node('/testTime');
    %
    %       % Create a rate object to run at 20 Hz
    %       frequency = 20;
    %       r = ros.Rate(node, frequency);
    %
    %       % Start looping
    %       reset(r);
    %       for i = 1:10
    %            % User Code
    %            waitfor(r)
    %       end
    %
    %       % Shutdown ROS
    %       clear node;
    %       rosshutdown;
    %
    %   See also rateControl

    %   Copyright 2015-2020 The MathWorks, Inc.

    properties (SetAccess = private)
        %IsSimulationTime - Indicator if simulation time or wall clock time is used
        IsSimulationTime
    end

    methods
        function obj = Rate(node, desiredRate)
        %Rate Constructor for Rate object
        %   Please see the class documentation for more details.
        %   See also ros.Rate

            narginchk(2,2);

            % Initialize base class with desired rate
            obj@rateControl(desiredRate);

            % Parse ROS node input
            if isempty(node)
                node = ros.internal.Global.getNodeHandle(false);
            end

            % Validate that node is a ros.Node object
            validateattributes(node, {'ros.Node'}, {'scalar'}, ...
                               'Rate', 'node');

            timeObj = ros.internal.Time(node);
            obj.IsSimulationTime = timeObj.IsSimulationTime;

            % Set time provider to the ROS node time source (by default,
            % the rateControl base class uses the system time provider)
            if obj.IsSimulationTime
                obj.TimeProvider = ros.internal.NodeTimeProvider(node);
                obj.startTimeProvider;
            end
            node.ListofNodeDependentHandles{end+1} = matlab.internal.WeakHandle(obj);
        end

        function delete(obj)
        %DELETE Shut down ROS rate object
        %   DELETE(OBJ) shuts down the ROS rate object OBJ.

            delete(obj.TimeProvider)
        end
    end

    methods (Access = protected)
        function s = saveobj(obj)
        %saveobj Raise errors when attempt to save the object.
            s = saveobj@robotics.core.internal.mixin.Unsaveable(obj);
        end

        function onNodeShutdownComplete(obj, ~, ~)
        %onNodeShutdownComplete Called when the Java node finishes shut down
        %   This callback is triggered when the ROS node that this
        %   object is attached to shuts down. In this case, this
        %   handle object should be deleted, so it cannot be
        %   used in the workspace and is clearly marked as
        %   non-functional.

            obj.delete;
        end
    end

    methods (Static, Access = protected)
        function obj = loadobj(s)
        %loadobj Raise errors when attempt to load the object.
            obj = loadobj@robotics.core.internal.mixin.Unsaveable(s);
        end
    end

    %----------------------------------------------------------------------
    % MATLAB Code-generation
    %----------------------------------------------------------------------
    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
            name = 'ros.internal.codegen.Rate';
        end
    end
end
