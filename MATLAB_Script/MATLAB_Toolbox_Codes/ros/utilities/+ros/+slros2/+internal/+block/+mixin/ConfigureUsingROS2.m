classdef (Abstract) ConfigureUsingROS2 < handle
%This class is for internal use only. It may be removed in the future.

%ConfigureUsingROS2 Abstract mixin class for configuring block parameters via
%ROS 2

%   Copyright 2021-2022 The MathWorks, Inc.

%#codegen

    properties (Access = ?matlab.unittest.TestCase)
        %ConfigureUsingROS2Dialog - Configuration dialog object
        ConfigureUsingROS2Dialog = []

        %TopicTable - TopicTable instance
        TopicTable

        %Node - ROS 2 Node
        Node
    end

    properties (Constant, Access = ?matlab.unittest.TestCase)
        %NodeNamePrefix - Prefix for auto-generated node name
        NodeNamePrefix = 'ConfigureUsingROS2'
    end

    properties (Abstract, Access = protected)
        TableHeaders
    end

    methods (Abstract, Access = protected)
        dialogCloseCallbackImpl(obj, block, isAcceptedSelection, row)
        generateTopicTable(node, infoChangedCallback)
    end

    methods
        function configureUsingROS2(obj, block, dialogTitle)
        %configureUsingROS2 Executed when the user clicks the "Select"
        %button
            try %#ok<EMTC>
                % Creating new Node for election dialog
                uniqueName = obj.makeUniqueName(obj.NodeNamePrefix);
                obj.Node = ros2node(uniqueName, ...
                                    ros.ros2.internal.NetworkIntrospection.getDomainIDForSimulink, ...
                                    'RMWImplementation', ...
                                    ros.ros2.internal.NetworkIntrospection.getRMWImplementationForSimulink);

                % Open selection dialog
                obj.ConfigureUsingROS2Dialog = ...
                    ros.slros.internal.dlg.TableViewer( ...
                    obj.TableHeaders, 'Title', dialogTitle);
                obj.TopicTable = ...
                    obj.generateTopicTable(obj.Node, @obj.infoChangedCallback);
                obj.infoChangedCallback(obj.TopicTable.TopicInfo);
                obj.ConfigureUsingROS2Dialog.openDialog(@(varargin)obj.dialogCloseCallback(block, varargin{:}));
            catch ME
                % Send error to Simulink diagnostic viewer rather than a
                % DDG dialog.
                % NOTE: This does NOT stop execution.
                reportAsError(MSLDiagnostic(ME), getfullname(bdroot(block)), true);
            end
        end

        function infoChangedCallback(obj, topicInfo)
        %infoChangedCallback Callback function for TopicTable
            obj.ConfigureUsingROS2Dialog.updateData(topicInfo, ...
                                                    contains(topicInfo.Status, message('ros:slros2:topicselector:Valid').getString));
            obj.ConfigureUsingROS2Dialog.refresh();
        end
    end

    methods (Access = protected)
        function dialogCloseCallback(obj, block, isAcceptedSelection, row)
        %dialogCloseCallback Call child class callback and clean up
            obj.dialogCloseCallbackImpl(block, isAcceptedSelection, row);
            delete(obj.TopicTable);
            delete(obj.Node);
        end
    end

    methods(Access = protected, Static)
        function group = getConfigurableSettingsGroup(groupTitle, props, dialogTitle, varargin)
        %getConfigurableSettingsGroup Properties to configure with ROS 2
        %   Child classes should call this in getPropertyGroupsImpl
            group = matlab.system.display.Section( ...
                'Title', groupTitle, ...
                'PropertyList', props, ...
                varargin{:});
            key = 'ros:slros2:blockmask:ConfigureUsingROS2Label';
            group.Actions = matlab.system.display.Action(...
                @(actionData, obj) obj.configureUsingROS2(actionData.SystemHandle, dialogTitle), ...
                'Label', getString(message(key)), ...
                'Alignment', 'right');

            % Disable button when simulation is running
            matlab.system.display.internal.setCallbacks(group.Actions, ...
                                                        'IsEnabledFcn', @matlab.system.display.Action.disableWhileSystemLocked);
        end
    end

    methods(Static, Hidden)
        function newName = makeUniqueName(name)
        % Using the model name as the node name runs into some issues:
        %
        % 1) There are 2 MATLAB sessions running the same model
        %
        % 2) A model registers a node with ROS Master on model init
        %    and clears the node on model termination. In some cases, the ROS
        %    master can hold on to the node name even if the node
        %    itself (in ROSJAVA) is cleared. This causes a problem
        %    during model init on subsequent simulation runs.
        %
        % To avoid these kinds of issues, we randomize the node name
        % during simulation.
            newName = [name '_' num2str(randi(1e5,1))];
        end
    end
end
