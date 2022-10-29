classdef StatusBarView < robotics.ikdesigner.internal.view.View
%This function is for internal use only. It may be removed in the future.

%   StatusBarView View that displays the current configuration in a status bar at the base of the app window

%   Copyright 2021-2022 The MathWorks, Inc.

    properties (Access = ?matlab.unittest.TestCase)

        %Parent Handle to parent container window
        Parent

        %StatusBar Status bar object
        StatusBar

        %StatusLabel UILabel used to convey information in status bar
        StatusLabel

        %ConfigurationView
        ConfigurationView
    end

    properties (Constant)
        Tag = "ConfigStatusBar"

        LabelTag = "ConfigStatusLabel"

        CURRENTCONFIGLABEL = string(message('robotics:ikdesigner:statusbar:CurrentConfigLabel'))

        NONINITIALIZEDCONFIGLABEL = string(message('robotics:ikdesigner:statusbar:NonInitializedConfigLabel'))

        NOTSTOREDCONFIGLABEL = string(message('robotics:ikdesigner:statusbar:NonStoredConfigLabel'))

        TOOLTIP = string(message('robotics:ikdesigner:statusbar:DescriptionTooltip'))
    end

    methods
        function obj = StatusBarView(window)
        %StatusBarView

        % Create a grid and initialize a browser view
            obj.Parent = window;

            % Add the status bar
            obj.StatusBar = matlab.ui.internal.statusbar.StatusBar();
            obj.StatusBar.Tag = obj.Tag;

            % Add a label containing the text
            obj.StatusLabel = matlab.ui.internal.statusbar.StatusLabel();
            obj.StatusLabel.Tag = obj.LabelTag;
            obj.StatusLabel.Description = obj.TOOLTIP;
            obj.StatusLabel.Region = "right";
            obj.StatusBar.add(obj.StatusLabel);

            % Update the label
            obj.StatusLabel.Text = obj.CURRENTCONFIGLABEL + ": <" + obj.NONINITIALIZEDCONFIGLABEL + ">";

            add(window, obj.StatusBar);
        end

        function setup(obj, jointConfigAngularIdx)
            obj.StatusLabel.Text = obj.CURRENTCONFIGLABEL + ": <" + obj.NONINITIALIZEDCONFIGLABEL + ">";
            obj.ConfigurationView = robotics.ikdesigner.internal.view.ConfigurationView(jointConfigAngularIdx);
        end

        function initialize(~)
        % Initialize
        end

        function update(obj, name, config)
        %update Update the status bar value with a new name & displayed configuration
            if isempty(name)
                name = obj.NOTSTOREDCONFIGLABEL;
            end

            configString = obj.ConfigurationView.getUserFacingConfigString(config);
            obj.StatusLabel.Text = sprintf("%s: %s <%s>", obj.CURRENTCONFIGLABEL, name, configString);

        end
    end
end
