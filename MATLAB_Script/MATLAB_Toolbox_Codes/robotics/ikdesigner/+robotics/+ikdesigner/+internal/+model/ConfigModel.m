classdef ConfigModel < robotics.ikdesigner.internal.model.Model
%This class if for internal use only and may be removed in a future release

%ConfigModel Model containing the stored joint configurations and associated collision data

%   Copyright 2021 The MathWorks, Inc.

    events
        %ConfigModelUpdated
        ConfigModelUpdated

        %RequestUIAlert Create a UI alert
        RequestUIAlert

        %RequestClearCurrentConfigKey
        RequestClearCurrentConfigKey

        %ResetSavedSessionCheck
        ResetSavedSessionCheck
    end

    properties (SetAccess = ?matlab.unittest.TestCase)
        %ConfigurationsMap Store joint configurations and the associated model states
        %   The configurations map is a containers.Map object where the key
        %   is a uuid and the values include the associated configuration
        %   and label, but also the states that will need to be restored
        %   when the robot moves to this state (e.g. collision and
        %   constraint satisfaction data).
        ConfigurationsMap

        %ConfigKeyArray String array that indicates the configuration table order
        %   The configurations are stored independently of their order, but
        %   the user-facing view has a specified order that must be
        %   preserved between sessions. The ConfigKeyArray is a string
        %   array that corresponds to the order of the configurations in
        %   the user-facing table view.
        ConfigKeyArray

        %SharedModelState Handle to the core states that are shared amongst models
        SharedModelState
    end

    properties (Dependent)
        %ConfigKey Key of the active configuration
        ConfigKey

        %ConfigName Name of the active configuration
        ConfigName

        %AllConfigKeys String array of all the stored configuration keys
        AllConfigKeys
    end

    properties (Constant)
        DEFAULTNEWCONFIGPREFIX = string(message('robotics:ikdesigner:configurationspanel:DefaultNewConfigPrefix'))

        CONFIGSNOTIMPORTEDERRHEADER = string(message('robotics:ikdesigner:configurationspanel:ConfigsNotImported')) + newline
    end

    methods
        function obj = ConfigModel(sharedModelState)
        %ConfigModel Constructor

            obj.SharedModelState = sharedModelState;
            obj.ConfigurationsMap = containers.Map.empty;
            obj.ConfigKeyArray = string.empty;

        end

        function setup(obj, modelData)
        %setup Reset data members and assign new values from incoming data

            if nargin > 1
                obj.ConfigurationsMap = modelData.ConfigurationsMap;
                obj.ConfigKeyArray = modelData.ConfigKeyArray;
            else
                obj.ConfigurationsMap = containers.Map.empty;
                obj.ConfigKeyArray = string.empty;
            end
        end

        function initialize(~)
        %initialize Initialize a new session
        end
    end

    methods

        function [config, ikState, collisionState, collisionData] = getConfigDetails(obj, configKey)
        %getConfigDetails Get configuration values & model states associated with a stored configuration
        %   This method accepts a key associated with a particular
        %   configuration, and returns the value of that configuration,
        %   as well as the associated ik solution and collision states
        %   at the specified configuration..

            config = obj.ConfigurationsMap(configKey).Configuration;
            ikState = obj.ConfigurationsMap(configKey).IKState;
            collisionState = obj.ConfigurationsMap(configKey).CollisionState;
            collisionData = obj.ConfigurationsMap(configKey).CollisionRawData;

        end

        function importConfigs(obj, importedConfigsEventData)
        %importConfigs Import configurations from an external source
        %   Given an event containing named configurations imported
        %   from an external source, this method parses those
        %   configurations, validates them, and adds them to the
        %   configuration model. For any invalid configurations, an
        %   associated error message is generated, and a ui alert is
        %   used to display all the configurations that could not be
        %   imported.

            configsArray = importedConfigsEventData.Data;
            configNamesArray = importedConfigsEventData.DataNames;
            errorsString = string.empty;

            % Iterate over all the data inputs and add them to the stored
            % configurations
            for i = 1:numel(configsArray)
                configMat = configsArray{i};
                configName = configNamesArray{i};
                [isValid, errorString] = obj.validateConfigInput(configMat, configName);

                if isValid
                    % Each input may be a single named configuration or a
                    % matrix of joint configurations
                    numConfigsInMat = size(configMat, 1);
                    for j = 1:numConfigsInMat
                        config = configMat(j,:);
                        if numConfigsInMat > 1
                            configName = sprintf("%s_%i", configNamesArray{i}, j);
                        end
                        newConfigEntry = robotics.ikdesigner.internal.mapdata.ConfigMapEntry(config(:), configName);
                        obj.addConfigEntryToMap(newConfigEntry);
                    end
                else
                    errorsString(end+1) = errorString; %#ok<AGROW>
                end
            end

            if ~isempty(errorsString)
                % Throw an error that indicates that not all the
                % configurations were successfully imported
                errorHeader = obj.CONFIGSNOTIMPORTEDERRHEADER;
                errorContents = sprintf("- %s"+newline, errorsString);
                obj.notifyUIAlert(errorHeader + errorContents);
            end
        end

        function configKey = addConfig(obj, config, ikState, collisionState, collRawData)
        %addConfig Add configuration to the stored configurations
        %   This method adds a configuration and the current ik
        %   solution & collision states to the model and returns the
        %   key that is now used to access that configuration state.

            configName = obj.generateDefaultConfigName;
            newConfigEntry = robotics.ikdesigner.internal.mapdata.ConfigMapEntry(config, configName);
            newConfigEntry.IKState = ikState;
            newConfigEntry.CollisionState = collisionState;
            newConfigEntry.CollisionRawData = collRawData;
            configKey = obj.addConfigEntryToMap(newConfigEntry);
        end

        function editConfig(obj, evt)
        %editConfig Edit the configuration value or associated data for a specified configuration
        %   This method accepts an event that indicates changes in the
        %   view and updates the corresponding model data. When the
        %   view changes are on values connected to collision or
        %   constraint satisfaction, it is necessary to reset the
        %   associated values.

            configKey = evt.AffectedKeys;
            configEntry = obj.ConfigurationsMap(configKey);

            % Depending what items were change (i.e. what columns were
            % edited), it may be necessary to update other states
            if evt.ValueChanged == robotics.ikdesigner.internal.model.ConfigTableColAssignment.Name
                configEntry.Name = evt.NewName;
                obj.ConfigurationsMap(configKey) = configEntry;
            else
                configEntry.Configuration = evt.NewConfig(:);
                obj.ConfigurationsMap(configKey) = configEntry;

                % Changing the value of the configuration vector will reset
                % the constraints and collision states
                obj.resetStoredConstraintState(configKey);
                obj.resetStoredCollisionState(configKey);
            end

            % Notify the controller of these changes
            updateType = robotics.ikdesigner.internal.model.ConfigUpdate.MapValuesChange;
            obj.notifyConfigTableUpdated(updateType, configKey);

        end

        function removeConfig(obj, evt)
        %removeConfig Remove configuration from the set of stored configurations
        %   This method deletes the configuration & associated data
        %   from the map and removes the keys from the array that
        %   specifies table order.

        % Remove the keys from the map
            configKeys = evt.AffectedKeys;
            for i = 1:numel(configKeys)
                obj.ConfigurationsMap.remove(configKeys(i));

                % If the key being removed corresponds to the current model
                % state, remove the association
                if obj.ConfigKey == configKeys(i)
                    notify(obj, "RequestClearCurrentConfigKey");
                end
            end

            % Remove the keys from the array
            arrayIdxToRemove = any(obj.ConfigKeyArray(:)' == configKeys(:),1);
            obj.ConfigKeyArray(arrayIdxToRemove) = [];

            obj.notifyModelBecameDirty();
        end

        function rearrangeConfigTable(obj, evt)
        %rearrangeConfigTable Update the configuration order specified in the model
        %   This method accepts and event and rearranges the config
        %   keys so that they match the order specified in the event.
        %   This method is used for updating the stored model state so
        %   that it matches the view. This will be used when the
        %   session is saved to preserve the state.

            newIndices = evt.NewRowIndices;
            obj.ConfigKeyArray = obj.ConfigKeyArray(newIndices);

            obj.notifyModelBecameDirty();
        end

        function resetAllCollisionStates(obj)
        %resetAllCollisionStates Reset collision for all stored configurations
        %   This method iterates over all the stored configurations and
        %   updates the stored collision to its default, unset state.
        %   The method then notifies the views so that the user-facing
        %   table is also updated.

            storedConfigKeys = obj.AllConfigKeys;

            % Exit if no data is stored
            if isempty(storedConfigKeys)
                return;
            end

            % Iterate overall the configurations
            for i = 1:numel(storedConfigKeys)
                obj.resetStoredCollisionState(storedConfigKeys(i));
            end

            % Notify the controller that the model has been updated
            updateType = robotics.ikdesigner.internal.model.ConfigUpdate.MapValuesChange;
            obj.notifyConfigTableUpdated(updateType, storedConfigKeys);
        end

        function resetCurrentConfigCollision(obj)
        %resetCurrentConfigCollision Reset collision for the active configuration

        % Exit if the active configuration does not correspond to a
        % stored configuration
            if isempty(obj.ConfigKey)
                return;
            end

            obj.resetStoredCollisionState(obj.ConfigKey);

            % Notify the controller that the model has been updated
            updateType = robotics.ikdesigner.internal.model.ConfigUpdate.MapValuesChange;
            obj.notifyConfigTableUpdated(updateType, obj.ConfigKey);

        end

        function resetConfigConstraintData(obj, evt)
        %resetConfigConstraintData Reset constraint satisfaction for a specified configuration

        % When constraints are added, edited, or removed, reset the
        % constraint state in the stored configurations
            if evt.UpdateType == robotics.ikdesigner.internal.model.ConstraintUpdate.MapValuesChange
                obj.resetAllStoredConstraintStates();
            end
        end

        function updateCurrentConfigCollision(obj, collisionState, areBodiesColliding)
        %updateCurrentConfigCollision Update the collision for the active configuration

        % Exit if the active configuration does not correspond to a
        % stored configuration
            if isempty(obj.ConfigKey)
                return;
            end

            obj.updateCollisionState(obj.ConfigKey, collisionState, areBodiesColliding);
        end

        function updateCollisionState(obj, configKey, collisionState, areBodiesColliding)
        %updateCollisionState Update the collision state associated with a particular configuration

            configEntry = obj.ConfigurationsMap(configKey);
            configEntry.CollisionState = collisionState;
            configEntry.CollisionRawData = areBodiesColliding;
            obj.ConfigurationsMap(configKey) = configEntry;

            updateType = robotics.ikdesigner.internal.model.ConfigUpdate.MapValuesChange;
            obj.notifyConfigTableUpdated(updateType, configKey);

        end

        function config = getConfig(obj, key)
        %getConfig Return the configuration value for a specified key

            config = obj.ConfigurationsMap(key).Configuration;
        end

        function [configsArray, configNamesArray] = getConfigData(obj)
        %getConfigData Get the array of named configurations in the order displayed in the user-facing table

        % To preserve table order, use the config key array to get the
        % values of the stored configurations. Since the config key
        % array is ordered, the entries will be presented in the same
        % order as is in the table.
            configKeysCellArray = cellstr(obj.ConfigKeyArray);
            configEntriesArray = cellfun(@(x)(obj.ConfigurationsMap(x)), configKeysCellArray, 'UniformOutput', false);

            % Extract the values and names of these configurations and
            % store them in a cell array
            configsArray = cellfun(@(x)(x.Configuration), configEntriesArray, 'UniformOutput', false);
            configNamesArray = cellfun(@(x)(x.Name), configEntriesArray, 'UniformOutput', false);
        end

        function keys = get.AllConfigKeys(obj)
        %get.AllConfigKeys Get a string array of all the stored configuration keys

            keys = string(obj.ConfigurationsMap.keys);
        end

        function key = get.ConfigKey(obj)
        %get.ConfigKey Returns the key of the active configuration
        %   If the active configuration corresponds to a stored value,
        %   this will return a non-empty string. If the active
        %   configuration does not match a stored value, this will
        %   return empty.

            key = obj.SharedModelState.CurrentConfigKey;
        end

        function configName = get.ConfigName(obj)
        %get.ConfigName Returns the name of the active configuration
        %   If the active configuration corresponds to a stored value,
        %   this will return a non-empty string. If the active
        %   configuration does not match a stored value, this will
        %   return empty.

            if isempty(obj.ConfigKey)
                configName = string.empty;
            else
                configName = obj.ConfigurationsMap(obj.ConfigKey).Name;
            end
        end
    end

    methods (Access = private)

        function resetAllStoredConstraintStates(obj)
        %resetAllStoredConstraintStates Reset all constraint states to their default (unset) values

            storedConfigKeys = obj.ConfigurationsMap.keys;
            for i = 1:numel(storedConfigKeys)
                obj.resetStoredConstraintState(storedConfigKeys{i});
            end
        end

        function resetStoredConstraintState(obj, configKey)
        %resetAllStoredConstraintStates Reset constraint state associated with the specified configuration key to default (unset) values

            configEntry = obj.ConfigurationsMap(configKey);
            configEntry.IKState.Info = [];
            configEntry.IKState.ConstraintKeys = [];
            obj.ConfigurationsMap(configKey) = configEntry;
        end

        function resetStoredCollisionState(obj, configKey)
        %resetAllStoredConstraintStates Reset collision state associated with the specified configuration key to default (unset) values

            configEntry = obj.ConfigurationsMap(configKey);
            configEntry.CollisionState = robotics.ikdesigner.internal.model.RBTCollisionState.NotEvaluated;
            configEntry.CollisionRawData = false(size(configEntry.CollisionRawData));
            obj.ConfigurationsMap(configKey) = configEntry;

        end

        function defaultConfigName = generateDefaultConfigName(obj)
        %generateDefaultConfigName Generate a default configuration name that does not conflict with other existing configurations
        %   While the configuration name does not have to be unique, it
        %   is helpful if the default names are unique by default.

            idx = 1;
            defaultConfigName = sprintf("%s%i", obj.DEFAULTNEWCONFIGPREFIX, idx);
            allNames = cellfun(@(x)(x.Name), obj.ConfigurationsMap.values, "UniformOutput",true);
            while any(strcmp(defaultConfigName, allNames))
                idx = idx + 1;
                defaultConfigName = sprintf("%s%i", obj.DEFAULTNEWCONFIGPREFIX, idx);
            end

        end

        function configKey = addConfigEntryToMap(obj, newConfigEntry)
        %addConfigEntryToMap Add a configurations map entry to the map
        %   This method generates a key for a new entry in the
        %   configurations map, adds the entry to the map, updates the
        %   corresponding array that specifies table order, notifies of
        %   the change, and returns the key.

            configKey = matlab.lang.internal.uuid();
            obj.ConfigurationsMap(configKey) = newConfigEntry;

            % Append the key to the array of keys that indicate table order
            obj.appendKeyToTable(configKey);

            % Notify the controller that the model has been updated
            updateType = robotics.ikdesigner.internal.model.ConfigUpdate.ValuesAdded;
            obj.notifyConfigTableUpdated(updateType, configKey);
        end

        function notifyConfigTableUpdated(obj, updateType, affectedKeys)
        %notifyConfigTableUpdated Indicate to the view that the stored configs have changed

            evtData = robotics.ikdesigner.internal.event.ConfigEventData(updateType, obj.ConfigurationsMap);
            evtData.AffectedKeys = affectedKeys;
            notify(obj, "ConfigModelUpdated", evtData);

            obj.notifyModelBecameDirty();
        end


        function newRow = appendKeyToTable(obj, key)
        %appendKeyToTable Add key as the last entry in the key array (bottom of the table)

            newRow = numel(obj.ConfigKeyArray)+1;
            obj.ConfigKeyArray(newRow) = key;
        end

        function [isValid, errorString] = validateConfigInput(obj, configMatrix, configName)
        %validateConfigInput Validate a configuration matrix
        %   Verify that the configuration matrix has the expected
        %   dimensions for the current rigid body tree object. This
        %   method returns a flag, isValid, that is true when the
        %   configuration matrix is valid and false otherwise, and a
        %   string indicating the error when the configuration is
        %   invalid.

            isValid = true;
            errorString = string.empty;
            try
                numNonFixedJoints = obj.SharedModelState.RigidBodyTree.TreeInternal.VelocityNumber;
                validateattributes(configMatrix, {'numeric'}, {'ncols', numNonFixedJoints}, 'inverseKinematicsDesigner', configName);
            catch ME
                errorString = ME.message;
                isValid = false;
            end

        end

        function notifyUIAlert(obj, messageString)
        %notifyUIAlert Notify controller that the app window needs to issue a UI alert

            messageEvt = robotics.ikdesigner.internal.event.MessageEvent(messageString);
            notify(obj, 'RequestUIAlert', messageEvt);
        end
    end
end
