classdef ConfigController < robotics.manip.internal.InternalAccess
    %This class if for internal use only and may be removed in a future release

    %ConfigController Controller that listens to ConfigModel
    %   The ConfigController has listeners on the configurations model and
    %   views that can trigger to configurations model. It facilitates
    %   communication regarding the stored configurations and associated
    %   stored state data.

    %   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private, GetAccess = ?matlab.unittest.TestCase)

        %ConfigModel Model containing stored configuration data
        ConfigModel

        %SceneModel Model containing stored scene data
        SceneModel

        %SolverModel Model containing solver and solution data
        SolverModel

        %ConfigurationsPanelView Handle to the configurations panel view
        ConfigurationsPanelView

        %SceneCanvasView Handle to the scene canvas view
        SceneCanvasView

        %StatusBarView Handle to the status bar view
        StatusBarView
    end

    methods
        function obj = ConfigController(models, views)
            %ConfigController Constructor

            % The config controller only has model event listeners on the
            % config model
            obj.ConfigModel = models.ConfigModel;

            % Views that have listeners
            obj.ConfigurationsPanelView = views.ConfigurationsPanelView;
            obj.SceneCanvasView = views.SceneCanvasView;

            % Other models and views that may receive commands
            obj.SceneModel = models.SceneModel;
            obj.SolverModel = models.SolverModel;
            obj.StatusBarView = views.StatusBarView;
        end

        function initialize(obj)
            obj.addViewListeners;
            obj.addModelListeners;
        end
    end

    methods (Access = private)
        function addViewListeners(obj)
            addlistener(obj.ConfigurationsPanelView, 'RequestStoreCurrentConfig', @(source, event)obj.addConfigToModel() );
            addlistener(obj.ConfigurationsPanelView, 'RequestSnapToConfig', @(source, data)obj.snapToSelectedConfig(data) );
            addlistener(obj.ConfigurationsPanelView, 'DeleteConfig', @(source, data)obj.removeConfigRowFromModel(data) );
            addlistener(obj.ConfigurationsPanelView, 'MoveConfig', @(source, data)obj.moveConfigInModel(data) );
            addlistener(obj.ConfigurationsPanelView, 'TableEdited', @(source, data)obj.modifyModelConfigTable(data) );
        end

        function addModelListeners(obj)
            addlistener(obj.ConfigModel, 'ConfigModelUpdated', @(source, event)obj.updateConfigTableView(event) );
            addlistener(obj.ConfigModel, 'RequestClearCurrentConfigKey', @(source, data)obj.clearCurrentConfigAssociations() );
        end
    end

    methods

        function snapToSelectedConfig(obj, configEvent)
            %snapToSelectedConfig Move robot to the specified configuration
            %   Move to the specified configuration and update the
            %   dependent state views.

            % Extract the stored information from the configs model and
            % apply it to the app
            [config, ikState, collisionState, collisionData] = obj.ConfigModel.getConfigDetails(configEvent.Key);
            obj.updateAppWithStoredConfigData(configEvent.Key, config, ikState, collisionState, collisionData);
        end

        function addConfigToModel(obj)
            %addConfigToModel Add current configuration to  configurations model
            %   Gather the current state and store it in the configurations
            %   model using a key to identify the configuration. The key is
            %   returned from the configurations model after being added.

            % Gather info about the current configuration from the models
            currentConfig = obj.SceneModel.Config;
            currentIKState = obj.SolverModel.LastSolutionState;
            currCollState = obj.SceneModel.CollisionState;
            currCollRawData = obj.SceneModel.CollisionRawData;

            % Add stored data to model
            configKey = obj.ConfigModel.addConfig(currentConfig, currentIKState, currCollState, currCollRawData);

            % Label the current position with a key
            obj.SceneModel.ConfigKey = configKey;

            % Update the status bar
            obj.StatusBarView.update(obj.ConfigModel.ConfigName, currentConfig);
        end

        function clearCurrentConfigAssociations(obj)
            %clearCurrentConfigAssociations Remove the association between the current config and any stored state in models and views
            %   This method is called when the current configuration is no
            %   longer associated with a stored configuration state. In
            %   that case, the stored key is cleared and any user-facing
            %   views that communicate this state are reset.

            % Label the current position with a key
            obj.SceneModel.ConfigKey = string.empty;

            % Update the status bar
            currentConfig = obj.SceneModel.Config;
            obj.StatusBarView.update(obj.SceneModel.ConfigKey, currentConfig);
        end

        function removeConfigRowFromModel(obj, tableDeleteEvent)
            %removeConfigRowFromModel Remove a configuration from the configurations model
            %   This method is triggered by removing rows from the
            %   user-facing configurations table.

            obj.ConfigModel.removeConfig(tableDeleteEvent);
        end

        function moveConfigInModel(obj, moveEvent)
            %moveConfigInModel Move a configuration in the model
            %   Method to update the model table order so it matches
            %   changes in the view.

            obj.ConfigModel.rearrangeConfigTable(moveEvent)

        end

        function modifyModelConfigTable(obj, tableEditEvent)
            %modifyModelConfigTable Change the contents of an item in the configurations table

            obj.ConfigModel.editConfig(tableEditEvent);
        end

        function updateConfigTableView(obj, event)
            %updateConfigTableView Update the displayed configurations table
            %   Update individual rows in the user-facing configurations
            %   table to reflect changes in the model.

            configKeysToUpdate = event.AffectedKeys;
            obj.ConfigurationsPanelView.updateTableValues(event);

            if ismember(configKeysToUpdate, obj.SceneModel.ConfigKey)
                [config, ikState, collisionState, collisionData] = obj.ConfigModel.getConfigDetails(obj.SceneModel.ConfigKey);
                obj.updateAppWithStoredConfigData(obj.SceneModel.ConfigKey, config, ikState, collisionState, collisionData);
            end

        end
    end

    methods (Access = private)
        function updateAppWithStoredConfigData(obj, configKey, config, ikState, collisionState, collisionData)

            % Update the other models with stored data. The order is
            % important since these methods update some of the same data
            obj.SolverModel.applyPredefinedState(config, ikState);
            obj.SceneModel.ConfigKey = configKey;
            obj.StatusBarView.update(obj.ConfigModel.ConfigName, config);
            obj.SceneModel.updateCollisionState(collisionState, collisionData);

            eeBodyPose = obj.SolverModel.LastSolutionEEPose;
            obj.SceneCanvasView.updateMarkerPose(eeBodyPose);
        end
    end
end