classdef AppStateController < handle
    %This class if for internal use only and may be removed in a future release
    
    %AppStateController Controller that listens to AppStateModel
    %   The AppStateController has listeners on the app state model and
    %   views that can trigger to app state model. It facilitates
    %   communication regarding the higher-level app state.
    
    %   Copyright 2021 The MathWorks, Inc.
    
    properties (Access = ?matlab.unittest.TestCase)
        
        %AppModels Class containing all the app models
        AppModels
        
        %AppViews Primary app window that also contains all the app views
        AppViews

        %AppContainerWindow
        AppContainerWindow
        
        %AppStateModel Model containing data about the current app state
        AppStateModel
        
        %SceneModel Model containing scene contents and state data
        SceneModel
        
        %Solver Model containing IK solver and solution data
        SolverModel
        
        %ConfigModel Model containing stored configuration data
        ConfigModel
        
        %ToolstripView Handle to the toolstrip view
        ToolstripView

        %ConfigurationsPanelView Handle to the configurations panel view
        ConfigurationsPanelView
    end

    properties (Constant)
        CORRUPTEDAPPSESSIONERR = string(message('robotics:ikdesigner:dataimportexport:CorruptedAppSession').getString)
    end
    
    methods
        function obj = AppStateController(models, views)
            %AppStateController Constructor
            
            import robotics.ikdesigner.internal.model.AppState
            
            obj.AppModels = models;
            obj.AppViews = views;

            obj.AppContainerWindow = obj.AppViews.Window;
            
            % The app state controller only has model event listeners on
            % the app state model
            obj.AppStateModel = models.AppStateModel;
            obj.AppStateModel.goto(AppState.Startup);
            
            % Other models that may receive commands
            obj.SceneModel = models.SceneModel;
            obj.SolverModel = models.SolverModel;
            obj.ConfigModel = models.ConfigModel;
            
            % Views that have listeners
            obj.ToolstripView = views.ToolstripView;
            obj.ConfigurationsPanelView = views.ConfigurationsPanelView;
            
        end
        
        function initialize(obj)
            obj.addViewListeners;
            obj.addModelListeners;
        end
    end
    
    methods (Access = private)
        
        function addViewListeners(obj)
            addlistener(obj.ToolstripView, 'NewSessionRBTSelected', @(source, evtData)obj.initializeNewSession(evtData) );
            addlistener(obj.ToolstripView, 'RequestAppSessionChange', @(source, event)obj.verifyAppSaved() );
            addlistener(obj.ToolstripView, 'RequestSaveSession', @(source, evtData)obj.saveSession() );
            addlistener(obj.ToolstripView, 'RequestSaveAsSession', @(source, evtData)obj.saveAsSession(evtData) );
            addlistener(obj.ToolstripView, 'RequestLoadSession', @(source, evtData)obj.loadSessionFromEvent(evtData) );
            addlistener(obj.ToolstripView, 'RequestSolverDataExport', @(source, evtData)obj.exportSolverData() );
            addlistener(obj.ToolstripView, 'RequestConfigDataExport', @(source, evtData)obj.exportConfigData() );
            addlistener(obj.ToolstripView, 'RequestAppBusy', @(source, evtData)obj.setAppWindowBusyState(true) );
            addlistener(obj.ToolstripView, 'RequestAppNotBusy', @(source, evtData)obj.setAppWindowBusyState(false) );
            addlistener(obj.ToolstripView, 'CollisionFileAdded', @(source, event)obj.addCollisionToSceneModel(event) );
            addlistener(obj.ToolstripView, 'SolverAdded', @(source, event)obj.replaceSolverInSolverModel(event) );
            addlistener(obj.ToolstripView, 'ConstraintsAdded', @(source, event)obj.addConstraintsToSolverModel(event) );
            addlistener(obj.ToolstripView, 'ConfigurationsAdded', @(source, event)obj.addConfigsToConfigurationsModel(event) );
            addlistener(obj.ToolstripView, 'RequestUIAlert', @(source, event)obj.triggerUIAlert(event) );
            addlistener(obj.AppViews, 'RequestSaveSession', @(source, evtData)obj.saveSession() );
            addlistener(obj.AppContainerWindow, 'WindowStateChanged', @(source, evtData)obj.updateDependentAppWindows() );
            addlistener(obj.ConfigurationsPanelView, 'RequestUIAlert', @(source, event)obj.triggerUIAlert(event) );
        end
        
        function addModelListeners(obj)          
            addlistener(obj.AppModels, 'RequestUIAlert', @(source, event)obj.triggerUIAlert(event) );
            addlistener(obj.AppModels, 'SessionSaved', @(source, event)obj.updateViewsForSavedSession() );

            % System-level listener actions
            addlistener(obj.SolverModel, 'ModelBecameDirty', @(source, event)obj.setSavedSessionTo(false) );
            addlistener(obj.ConfigModel, 'ModelBecameDirty', @(source, event)obj.setSavedSessionTo(false) );
            addlistener(obj.SceneModel, 'ModelBecameDirty', @(source, event)obj.setSavedSessionTo(false) );
            addlistener(obj.SolverModel, 'RequestUIAlert', @(source, event)obj.triggerUIAlert(event) );
            addlistener(obj.ConfigModel, 'RequestUIAlert', @(source, event)obj.triggerUIAlert(event) );
        end
        
        function initializeNewSession(obj, sessionData)
            %initializeNewSession Given a new tree, trigger actions to set up a new session
            
            import robotics.ikdesigner.internal.model.AppState
            
            % Update the app state
            obj.setAppWindowBusyState(true);
            obj.AppStateModel.goto(AppState.Startup);
            
            % Populate models with a new robot and initialize them
            obj.AppModels.setupSession(sessionData);
            obj.AppModels.initialize();
            
            % Set up views with initial state data and initialize them
            obj.AppViews.setup(obj.SceneModel, obj.SolverModel, obj.ConfigModel);
            obj.AppViews.initialize();
            
            % Advance the AppState
            obj.AppStateModel.goto(AppState.InSession);
            obj.setAppWindowBusyState(false);
            
        end

        function saveSession(obj)
            %saveSession Save current session to same file as in past
            %   This method tries to save the session to the same file as
            %   it was previously saved. If that is not successful, the
            %   method redirects the save action to "save as", so the user
            %   may specify the file target.

            obj.setAppWindowBusyState(true);
            isSaved = obj.AppModels.saveMat();
            if ~isSaved
                obj.ToolstripView.initiateSaveAsSession();
            else
                obj.setAppWindowBusyState(false);
            end
        end

        function saveAsSession(obj, filePathEventData)
            %saveAsSession Save the session with a specified file name

            obj.setAppWindowBusyState(true);
            filepath = filePathEventData.FilePath;
            obj.AppModels.saveAsMat(filepath);
            obj.setAppWindowBusyState(false);

        end

        function loadSessionFromEvent(obj, filePathEventData)
            %loadSessionFromEvent Load session from data event
            %   This method accepts an event containing the file path of
            %   the data to load, and initiates the session load. 
            
            % Update the app state
            obj.setAppWindowBusyState(true);

            % Get the file path from the event and use it to set up the app
            % models
            filepath = filePathEventData.FilePath;
            obj.loadSession(filepath);
        end

        function loadSession(obj, filepath)
            %loadSession Load session given a file path
            %   This method validates the input, attempts to load the data
            %   specified in the filepath, sets up the models, and then
            %   sets up and initializes the views form the model data.

            import robotics.ikdesigner.internal.model.AppState

            [modelLoadSuccess, isAppCorrupted] = obj.AppModels.loadSession(filepath);

            if modelLoadSuccess
                try
                    obj.AppModels.initialize();
    
                    % Set up views with initial state data and initialize them
                    obj.AppViews.setup(obj.SceneModel, obj.SolverModel, obj.ConfigModel);
                    obj.AppViews.initialize();

                    % Advance the AppState and set the session as saved
                    obj.AppStateModel.goto(AppState.InSession);
                    obj.AppStateModel.SavedSessionCheck = true;

                    % Update the app name to reflect what was loaded
                    [~, fileName] = fileparts(filepath);
                    obj.AppViews.setSessionName(fileName);
                    obj.AppViews.SavedSessionCheck = true;
                catch
                    % Set up with the model data has failed, which puts the
                    % app in a bad state
                    isAppCorrupted = true;
                end
            end

            % When an error occurs during the load process, the app is left
            % in a corrupted state. In that case, throw an error telling
            % the user they need to start fresh and move to a recovery
            % state.
            if isAppCorrupted
                userFacingMessage = obj.CORRUPTEDAPPSESSIONERR;
                messageEvent = robotics.ikdesigner.internal.event.MessageEvent(userFacingMessage);
                obj.triggerUIAlert(messageEvent);
                obj.resetAppForBadStateRecovery();
            end

            % Ensure the app window is no longer busy
            obj.setAppWindowBusyState(false);
        end

        function resetAppForBadStateRecovery(obj)
            %resetApp Disable views when app is put in a bad state
            %   By default, this brings the app to a corrupted state.
            %   Loading or starting a new session will exit this state.

            %TODO
        end

        function updateViewsForSavedSession(obj)
            %updateViewsForSavedSession Update the window when a session is saved

            sessionPath = obj.AppStateModel.SavedSessionName;
            [~, sessionName] = fileparts(sessionPath);
            obj.AppViews.setSessionName(sessionName);
            obj.setSavedSessionTo(true);
        end

        function setSavedSessionTo(obj, isSaved)
            %setSavedSessionTo Update the model and views when the model is saved or dirty

            obj.AppStateModel.SavedSessionCheck = isSaved;
            obj.ToolstripView.setSessionSavedTo(isSaved);
            obj.AppViews.SavedSessionCheck = isSaved;
        end

        function updateDependentAppWindows(obj)
            %updateDependentAppWindows Update state of non-appcontainer windows given appcontainer state change
            %   When the main app container window is closed, any non-modal
            %   figures that were created by it must also be closed (modal
            %   figures would already have been closed since they cannot be
            %   accessed independently from the app).

            if obj.AppContainerWindow.WindowState == matlab.ui.container.internal.appcontainer.AppWindowState.CLOSED
                obj.ToolstripView.closeOpenFigures();
            end

        end

        function addCollisionToSceneModel(obj, event)
            %addCollisionToSceneModel Add collision object to scene

            obj.SceneModel.addCollisionObjectsToSceneModel(event); 

            % Ensure the app is not blocked
            obj.setAppWindowBusyState(false);
        end

        function replaceSolverInSolverModel(obj, event)
            %replaceSolverInSolverModel Replace the solver in the solver model

            % Since only one solver object may be imported, it must be the
            % first entry in the data cell array
            solverData = event.Data{1};

            % Update the current solver with the parameters from the newly
            % provided solver
            obj.SolverModel.replaceSolverParameters(solverData); 

            % Ensure the app is not blocked
            obj.setAppWindowBusyState(false);           
        end

        function addConstraintsToSolverModel(obj, event)
            %addConstraintsToSolverModel Add constraints to solver

            obj.SolverModel.addCustomConstraints(event);

            % Ensure the app is not blocked
            obj.setAppWindowBusyState(false);
        end

        function addConfigsToConfigurationsModel(obj, event)
            %addConfigsToConfigurationsModel Add joint configurations to stored configurations

            obj.ConfigModel.importConfigs(event);

            % Ensure the app is not blocked
            obj.setAppWindowBusyState(false);
        end

        function exportSolverData(obj)
            %exportSolverData Export solver and constraint data via a UI

            [solver, constraintsArray, constraintNamesArray] = obj.SolverModel.exportSolverAndConstraints;
            obj.ToolstripView.exportSolverData(solver, constraintsArray, constraintNamesArray);

        end

        function exportConfigData(obj)
            %exportConfigData Export configuration data via a UI

            [configArray, configNameArray] = obj.ConfigModel.getConfigData;
            obj.ToolstripView.exportConfigData(configArray, configNameArray);

        end

        function verifyAppSaved(obj)
            %verifyAppSaved Verify app is saved and indicate status to toolstrip
            
            if obj.AppStateModel.AppState ~= robotics.ikdesigner.internal.model.AppState.InSession
                obj.ToolstripView.AllowSessionChange = true;
                return;
            end

            if obj.AppStateModel.SavedSessionCheck
                canProceed = true;
            else
                canProceed = obj.AppViews.confirmProceed();
            end
            obj.ToolstripView.AllowSessionChange = canProceed;
        end

        function setAppWindowBusyState(obj, isBusy)
            %setAppWindowBusyState Set the app window to be busy or clear
            
            obj.AppViews.Window.Busy = isBusy;
        end

        function triggerUIAlert(obj, msgEvent)
            %triggerUIAlert Create a UI alert window with a message

            obj.setAppWindowBusyState(false);
            obj.AppViews.alert(msgEvent);

        end
    end

    methods (Access = ?robotics.ikdesigner.internal.InverseKinematicsDesigner)
        function loadSessionFromFilepath(obj, filePath)
            %loadSessionFromFilepath Initiate session load from a file path
            %   This method is used as an entry point by the app's MATLAB
            %   API to load a session from file.
            
            % Update the app state
            obj.setAppWindowBusyState(true);

            % Get the file path from the event and use it to set up the app
            % models
            obj.loadSession(filePath);

        end
    end
end