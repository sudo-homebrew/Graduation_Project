classdef Time < robotics.core.internal.mixin.Unsaveable & ...
        ros.internal.mixin.ROSInternalAccess & handle
    %This class is for internal use only. It may be removed in the future.

    %Time Access ROS time functionality
    %   Normally, ROS will use your computer's system clock as a time
    %   source (system time). When you are connected to a robot simulation or
    %   play back logged data, ROS might publish time information on the
    %   /clock topic (simulation time). The simulation time can be accelerated or
    %   slowed in comparison to your system time.
    %   The Time object gives you access to either the system
    %   or simulation time, depending on your ROS network configuration.
    %   This can be used to timestamp messages or to measure time in the
    %   ROS network.
    %
    %   TIMEOBJ = ros.internal.Time(NODE) returns a ROS time object TIMEOBJ for
    %   accessing time functionality. The object will be attached to the ROS
    %   node NODE. If NODE is [], the object will attach to the global ROS node.
    %
    %   Time properties:
    %      CurrentTime       - (Read-Only) The current ROS time
    %      IsSimulationTime  - (Read-Only) Indicate if simulation time is used
    %
    %
    %   Examples:
    %       % Create the ROS Master and a node
    %       master = ros.Core;
    %       node = ros.Node('/testTime');
    %
    %       % Create the time object and connect to the node
    %       timeobj = ros.internal.Time(node);
    %
    %       % Get the current system or simulation time
    %       timeobj.CurrentTime
    %
    %
    %   See also ROSTIME.

    %   Copyright 2014-2020 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %CurrentTime - The current ROS time
        %   This property will show the simulation time published on the "/clock"
        %   topic if the "/use_sim_time" ROS parameter is set to true.
        %   Otherwise, this will return the system time.
        CurrentTime

        %CurrentSystemTime - The current system time
        %   This property returns the system time.
        CurrentSystemTime
    end

    properties (Transient, SetAccess = private)
        %IsSimulationTime - Indicate if simulation time is used
        %   This property will be "true" if the "/use_sim_time" ROS parameter
        %   was set to true when the parent node of this Time object was launched.
        IsSimulationTime
    end

    properties (Transient, Access = ?ros.internal.mixin.ROSInternalAccess)
        %InternalNode - Internal representation of the node object
        %   Node required to get parameter information
        InternalNode
        ServerNodeHandle
    end

    methods
        function obj = Time(node)
        %Time Construct an object handling ROS time
        %   Please see the class documentation
        %   (help ros.internal.Time) for more details.

        % If the input is empty, get the global node handle
            if (nargin < 1) || isempty(node)
                node = ros.internal.Global.getNodeHandle(false);
            end

            % Validate that node is a ros.Node object
            validateattributes(node, {'ros.Node'}, {'scalar'}, ...
                               'Time', 'node', 1);

            % Gateway to ROS server
            obj.InternalNode = node.InternalNode;
            obj.ServerNodeHandle = node.ServerNodeHandle;
            timeStruct = getCurrentTime(obj.InternalNode, obj.ServerNodeHandle, false);
            % IsSimulationTime is set to true if and only if the
            % /use_sim_time parameter was set before starting the node.
            % Hence it is set once when the Time object is created
            obj.IsSimulationTime = timeStruct.issimtime;
        end
    end

    methods
        function currentTime = get.CurrentTime(obj)
        %get.CurrentTime Retrieve the current ROS system time

        % getCurrentTime returns a structure with fields sec / nsec
            timeStruct = getCurrentTime(obj.InternalNode,...
                                        obj.ServerNodeHandle, false);
            currentTime = ros.msg.Time(timeStruct);
        end

        function currentSystemTime = get.CurrentSystemTime(obj)
            timeStruct = getCurrentTime(obj.InternalNode,...
                                        obj.ServerNodeHandle, true);
            currentSystemTime = ros.msg.Time(timeStruct);
        end

        function delete(obj)
        %DELETE Shut down parameter tree
        %   DELETE(OBJ) shuts down the ROS parameter tree object.
            obj.InternalNode = [];
            obj.ServerNodeHandle = [];
        end
    end
end
