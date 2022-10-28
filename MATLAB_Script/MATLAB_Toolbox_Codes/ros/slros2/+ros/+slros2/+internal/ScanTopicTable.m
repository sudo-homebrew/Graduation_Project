classdef ScanTopicTable < ros.slros2.internal.TopicTable
%This class is for internal use only. It may be removed in the future.

%ScanTopicTable

%   Copyright 2017-2021 The MathWorks, Inc.

    properties (Constant)
        %AdditionalColumns
        AdditionalColumns = {'ScanArrayLength'}

        %ValidMessageTypes
        ValidMessageTypes = {'sensor_msgs/LaserScan'}

        %NoTopicsAvailableMsgId - MsgID to use if there are no topics
        %   advertised for any of the ValidMessageTypes.
        NoTopicsAvailableMsgId = ...
            'ros:slros2:topicselector:NoScanTopicsAvailable'
    end

    methods

        function obj = ScanTopicTable(varargin)
        %ScanTopicTable Constructor
        %   OBJ = ScanTopicTable(NODE) creates an ScanTopicTable using
        %   the ros2node object, NODE. The table will stop
        %   updating and kill all remaining subscribers if 30 seconds
        %   elapse without a message being received by any subscriber.
        %   If no image topics are found on the ROS network, the
        %   constructor will error out.
        %
        %   OBJ = ScanTopicTable(NODE, infoChangeFcn) creates an
        %   ScanTopicTable that invokes infoChangeFcn each time the
        %   table is updated.
        %
        %   OBJ = ScanTopicTable(NODE, infoChangeFcn, TIMEOUT) creates
        %   an ScanTopicTable that stops updating and kills all
        %   subscribers if TIMEOUT seconds elapse without a message
        %   being received by any subscriber.
        %
        %   OBJ = ScanTopicTable(NODE, infoChangeFcn, TIMEOUT,
        %   allowEmptyTable) creates an ScanTopicTable with zero rows
        %   if allowEmptyTable is true and no image topics are found.
            obj = obj@ros.slros2.internal.TopicTable(varargin{:});
        end

        function [status, scanArrayLength] = extractData(~,  msg)
        % extractData gets called for every valid topic
            status = message('ros:slros2:topicselector:Valid').string;
            scanArrayLength = max(length(msg.ranges), length(msg.intensities));
        end


    end
end
