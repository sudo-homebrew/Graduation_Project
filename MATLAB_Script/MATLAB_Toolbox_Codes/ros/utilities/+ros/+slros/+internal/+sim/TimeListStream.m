classdef TimeListStream < ros.slros.internal.sim.TimeStream
%This class is for internal use only. It may be removed in the future.

%TimeListStream Getting ROS time from a list
%   There is an underlying FIFO list of times.

%   Copyright 2018-2020 The MathWorks, Inc.

    properties (Transient, SetAccess = ?matlab.unittest.TestCase)
        %TimeList - Cell array of times
        %   This should be initialized with object of type
        %   ros.msg.Time.
        TimeList = {}
    end

    methods
        function currentTime = getTime(obj)
        %getTime Get the next ROS time from the list

        % Throw an error if there is no time to get
            if isempty(obj.TimeList)
                error("ros:slros:internal:TimeListEmpty", "Cannot retrieve time, because the TimeList cell array is empty.");
            end

            % Otherwise, remove time from the front of the FIFO and
            % return it
            currentTime = obj.TimeList{1};
            obj.TimeList(1) = [];
        end

        function clearTimeList(obj)
        %clearTimeList Empty list of current times
            obj.TimeList = {};
        end
    end

    methods
        function set.TimeList(obj, timeList)
        %set.TimeList Setter for TimeList property

            validateattributes(timeList, "cell", {}, "TimeListStream", "TimeList");
            obj.TimeList = timeList;
        end
    end

end
