classdef (Abstract) ConfigureUsingROS < handle
%This class is for internal use only. It may be removed in the future.

%ConfigureUsingROS Abstract mixin class for configuring block parameters via
%ROS

%   Copyright 2017-2019 The MathWorks, Inc.

%#codegen

    properties (Access = ?matlab.unittest.TestCase)
        %ConfigureUsingROSDialog - Configuration dialog object
        ConfigureUsingROSDialog = []

        %TopicTable - TopicTable instance
        TopicTable

        %Node - ROS Node
        Node
    end
    
    properties (Constant, Access = ?matlab.unittest.TestCase)
        % NodeNamePrefix - Prefix for auto-generated node name
        NodeNamePrefix = 'ConfigureUsingROS'
    end

    properties (Abstract, Access = protected)
        TableHeaders 
    end

    methods (Abstract, Access = protected)
        dialogCloseCallbackImpl(obj, block, isAcceptedSelection, row)
        generateTopicTable(node, infoChangedCallback)
    end

    methods
        function configureUsingROS(obj, block, dialogTitle)
        %configureUsingROS Executed when the user clicks the "Select" button
            try %#ok<EMTC>
                % Creating new Node for selection dialog
                rosMaster = ros.slros.internal.sim.ROSMaster();
                rosMaster.verifyReachable()
                uniqueName = rosMaster.makeUniqueName(obj.NodeNamePrefix);
                obj.Node = rosMaster.createNode(uniqueName);

                % Open selection dialog
                obj.ConfigureUsingROSDialog = ...
                    ros.slros.internal.dlg.TableViewer( ...
                        obj.TableHeaders, 'Title', dialogTitle);
                obj.TopicTable = ...
                    obj.generateTopicTable(obj.Node, @obj.infoChangedCallback);
                obj.infoChangedCallback(obj.TopicTable.TopicInfo);
                obj.ConfigureUsingROSDialog.openDialog(@(varargin)obj.dialogCloseCallback(block, varargin{:}));
            catch ME
                % Send error to Simulink diagnostic viewer rather than a
                % DDG dialog.
                % NOTE: This does NOT stop execution.
                reportAsError(MSLDiagnostic(ME), getfullname(bdroot(block)), true);
            end
        end

        function infoChangedCallback(obj, topicInfo)
        %infoChangedCallback Callback function for TopicTable
            obj.ConfigureUsingROSDialog.updateData(topicInfo, ...
                                                   contains(topicInfo.Status, message('ros:slros:topicselector:Valid').getString));
            obj.ConfigureUsingROSDialog.refresh();
        end
    end

    methods (Access = protected)
        function dialogCloseCallback(obj, block, isAcceptedSelection, row)
        %dialogCloseCallback Call child class callback and clean up
            obj.dialogCloseCallbackImpl(block, isAcceptedSelection, row);
            delete(obj.TopicTable)
            delete(obj.Node);
        end
    end

    methods(Access = protected, Static)
        function group = getConfigurableSettingsGroup(groupTitle, props, dialogTitle, varargin)
        %getConfigurableSettingsGroup Properties to configure with ROS
        %   Child classes should call this in getPropertyGroupsImpl
            group = matlab.system.display.Section( ...
                'Title', groupTitle, ...
                'PropertyList', props, ...
                varargin{:});
            key = 'ros:slros:blockmask:ConfigureUsingROSLabel';
            group.Actions = matlab.system.display.Action(...
                @(actionData, obj) obj.configureUsingROS(actionData.SystemHandle, dialogTitle), ...
                'Label', getString(message(key)), ...
                'Alignment', 'right');

            % Disable button when simulation is running
            matlab.system.display.internal.setCallbacks(group.Actions, ...
                                                        'IsEnabledFcn', @matlab.system.display.Action.disableWhileSystemLocked);
        end
    end
end
