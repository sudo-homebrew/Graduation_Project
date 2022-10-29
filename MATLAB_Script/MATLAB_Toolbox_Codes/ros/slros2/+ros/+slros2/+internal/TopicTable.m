classdef (Abstract) TopicTable < robotics.core.internal.mixin.Unsaveable ...
        & ros.internal.mixin.ROSInternalAccess
    %This classs is for internla use only. It may be removed in the future.

    %TopicTable maintains a table of information based on the contents of a
    %   set of ROS 2 topics.
    %
    %  Sample use:
    %   selector = TopicTable;
    %   selector.openDialog(@(isAccepted,topicName) disp(topicName))

    %   Copyright 2021 The MathWorks, Inc.

    properties (Constant, Access = protected)
        %UnknownValueString
        UnknownValueString = "--"
    end

    properties (Constant, Abstract)
        %AdditionalColumns - Cell-array of char vectors naming the custom
        %   columns for this topic table
        AdditionalColumns

        %ValidMessageTypes - Cell-array of message types for filtering
        %   the list of topics
        ValidMessageTypes

        %NoTopicsAvailableMsgId - MsgID to use if there are no topics
        %   advertised for any of the ValidMessageTypes.
        NoTopicsAvailableMsgId
    end

    properties(SetAccess = private)

        %TopicInfo
        TopicInfo

        %Subscribers
        Subscribers

        %SubscriberTimer
        SubscriberTimer

        %InfoChangeFcn
        InfoChangeFcn

        %MessageTimeout
        MessageTimeout = 30
    end

    methods (Abstract)
        [status, val1, val2] = extractData(obj, msg);
        %extractData Extract status and other information from a ROS 2
        %   message. The number of "valN" outputs should be equal to the
        %   number of elements in obj.AdditionalColumns.
    end

    methods
        function obj = TopicTable(node, infoChangeFcn, timeout, allowEmptyTable)
        %TopicTable
            if nargin < 2
                infoChangeFcn = function_handle.empty;
            end
            if nargin < 3
                timeout = 30;
            end
            if nargin < 4
                allowEmptyTable = false;
            end

            obj.TopicInfo = table;
            [allTopicNames, allMessageTypes] = ...
                ros.ros2.internal.NetworkIntrospection.getTopicNamesTypes();

            % Build the table of parameter names and types
            validTopics = cellfun(@obj.isValidMessageType, allMessageTypes);
            numTopics = nnz(validTopics);
            obj.TopicInfo.Name = string(allTopicNames(validTopics));
            obj.TopicInfo.MessageType = string(allMessageTypes(validTopics));
            additionalData = repmat(obj.UnknownValueString, numTopics, numel(obj.AdditionalColumns));
            additionalTopicInfo = array2table(additionalData, ...
                                              'VariableNames', obj.AdditionalColumns);
            obj.TopicInfo = [obj.TopicInfo, additionalTopicInfo];
            obj.TopicInfo.Status = repmat( ...
                message('ros:slros2:topicselector:WaitingForMessage').string , ...
                numTopics, 1);
            obj.InfoChangeFcn = infoChangeFcn;
            obj.MessageTimeout = timeout;
            if numTopics > 0
                % Create Subscribers
                for k = 1:numTopics
                    obj.Subscribers{k} = ...
                        ros2subscriber(node, ...
                                       obj.TopicInfo.Name{k}, ...
                                       obj.TopicInfo.MessageType{k}, ...
                                       {@obj.onMessageReceived,k});
                end
                obj.SubscriberTimer = timer('StartDelay', obj.MessageTimeout, ...
                                            'TimerFcn', @(~,~,~)obj.killSubscribers);
                start(obj.SubscriberTimer);
            elseif ~allowEmptyTable
                ex = MSLException([], message(obj.NoTopicsAvailableMsgId));
                throw(ex);
            end
        end

        function delete(obj)
        %delete

        % Delete the timer
            if ~isempty(obj.SubscriberTimer) && isvalid(obj.SubscriberTimer)
                stop(obj.SubscriberTimer);
                delete(obj.SubscriberTimer);
            end

            % Shutdown the subscribers
            obj.killSubscribers();
        end

        function flag = isValidMessageType(obj, messageType)
            flag = ismember(messageType, obj.ValidMessageTypes);
        end

        function onMessageReceived(obj, msg, row)
            additionalData = cell(1, numel(obj.AdditionalColumns));
            [status, additionalData{:}] = obj.extractData(msg);
            obj.updateRow(row, status, additionalData{:});

            % Delete the subscriber for this row
            obj.Subscribers{row}.delete;

            % Restart the timer
            if ~isempty(obj.SubscriberTimer) && isvalid(obj.SubscriberTimer)
                stop(obj.SubscriberTimer);
            end
            start(obj.SubscriberTimer);
        end

        function updateRow(obj, row, status, varargin)
            if numel(varargin) < numel(obj.AdditionalColumns)
                varargin = [varargin, ...
                            repmat({obj.UnknownValueString}, ...
                                   1, numel(obj.AdditionalColumns)-numel(varargin))];
            end
            obj.TopicInfo.Status(row) = status;
            for k = 1:numel(obj.AdditionalColumns)
                obj.TopicInfo{row, obj.AdditionalColumns{k}} = string(varargin{k});
            end

            % Execute callback
            if ~isempty(obj.InfoChangeFcn)
                feval(obj.InfoChangeFcn,obj.TopicInfo);
            end
        end

        function killSubscribers(obj)
            for row = 1:numel(obj.Subscribers)
                if isvalid(obj.Subscribers{row})
                    obj.updateRow(row, ...
                                  message('ros:slros2:topicselector:NoMessageReceived').string);
                    obj.Subscribers{row}.delete;
                end
            end
        end
    end

    methods (Access = protected)
        function onNodeShutdown(obj,~,~)
        %onNodeShutdown Callback triggered when node shutdown begins
        %   Re-implement this method in derived classes if you want to
        %   be notified when the node shutdown process is started.
            obj.delete();
        end
    end
end
