classdef AppModels < handle
    %This class if for internal use only and may be removed in a future release
    
    %AppModels Object composed of all app models
    
    %   Copyright 2021 The MathWorks, Inc.

    events
        RequestUIAlert

        SessionSaved
    end
    
    properties (SetAccess = ?matlab.unittest.TestCase)

        %SharedModelState Key state variables accessible from other models
        SharedModelState
        
        %SceneModel Model pertaining to items in the scene
        SceneModel
        
        %SolverModel Model pertaining to the IK Solver
        SolverModel
        
        %ConfigModel Model containing the stored configuration data
        ConfigModel
        
        %AppStateModel Model containing data about the current app state
        AppStateModel
        
    end

    properties (Constant)
        INVALIDSESSIONERR = string(message('robotics:ikdesigner:dataimportexport:InvalidSessionFile').getString)
        INVALIDAPPMODELHEADER = string(message('robotics:ikdesigner:dataimportexport:InvalidAppModelHeader').getString)
    end
    
    methods
        function obj = AppModels()
            %AppModels Constructor
            %   The constructor creates all the model objects. The models
            %   are persistent for the lifetime of the app. When a new
            %   session is started or an old one is loaded, the setup and
            %   initialize functions are used to swap the data in the
            %   models.
        
            obj.SharedModelState = robotics.ikdesigner.internal.model.CoreSceneState;
            obj.SceneModel = robotics.ikdesigner.internal.model.SceneModel(obj.SharedModelState);
            obj.SolverModel = robotics.ikdesigner.internal.model.SolverModel(obj.SharedModelState);
            obj.ConfigModel = robotics.ikdesigner.internal.model.ConfigModel(obj.SharedModelState);
            obj.AppStateModel = robotics.ikdesigner.internal.model.AppStateModel;
        end
        
        function setupSession(obj, sessionData)
            %setupSession Set up the models for a new app session
            %   This step acts as a reset, replacing current model data
            %   with the correct data for a new session.
            
            % The new session uses data from the data import view, which is
            % a cell array of all variables. However the only item that
            % defines a new session is the robot, so on the first index is
            % needed. Use this to set up the shared model state. All the
            % other dependent models already have a handle to this object,
            % and will update their dependent values during setup.
            robot = sessionData.Data{1};
            obj.SharedModelState.setup(robot);
            obj.SceneModel.setup();
            obj.SolverModel.setup();
            obj.ConfigModel.setup();
            obj.AppStateModel.setup();
        end
        
        function initialize(obj)
            %initialize Initialize models
            
            obj.SceneModel.initialize();
            obj.SolverModel.initialize();
            obj.ConfigModel.initialize();
            obj.AppStateModel.initialize();
        end

        function isSaved = saveMat(obj)
            %saveMat Save the app models to a previously specified filename
            %   Save the app model to the file path previously used. If
            %   successful, the method returns true. If a file path has yet
            %   to be defined, or if the currently defined one does not
            %   exist, this method returns false.

            filepath = obj.AppStateModel.SavedSessionName;

            % Check that this file path exists
            if isempty(filepath) || ~exist(filepath, 'file')
                % The user has to save at least once
                isSaved = false;
            else
                obj.saveAsMat(filepath);
                isSaved = true;
            end
        end

        function saveAsMat(appdata, filepath)
            %saveAsMat Save the app models to a specified file
            %   The app models are stored in a file so that they can be
            %   reloaded to start a new session. Note that "appdata" is
            %   used in lieu of object so that the stored variable in the
            %   MAT file will be more descriptively named.

            % Store the current saved session and use a try/catch
            % architecture so that if the save fails, the model data
            % remains valid
            oldSessionName = appdata.AppStateModel.SavedSessionName;
            try
                appdata.AppStateModel.SavedSessionName = filepath;
                save(filepath, "appdata");
                notify(appdata, "SessionSaved");
            catch ME
                appdata.AppStateModel.SavedSessionName = oldSessionName;
                messageEvent = robotics.ikdesigner.internal.event.MessageEvent(ME.message);
                notify(appdata, 'RequestUIAlert', messageEvent);
            end
        end

        function [isDataLoaded, isAppCorrupted] = loadSession(obj, filepath)
            %loadSession Load session from file
            %   Load an AppModels object from file and use it to set up the
            %   current set of app models. These can then be initialized
            %   and used to initialize the views.

            isDataLoaded = false;
            isAppCorrupted = false;

            % First try loading the data and if that doesn't work, simply
            % error and go back to the existing session
            try
                loadedFileData = load(filepath);
            catch ME
                messageEvent = robotics.ikdesigner.internal.event.MessageEvent(ME.message);
                notify(obj, 'RequestUIAlert', messageEvent);
                return;
            end
            
            % Once the data is loaded, validate it to make sure it follows
            % the expected session format
            [isValid, errMsg] = obj.validateAppSessionData(loadedFileData);
            if ~isValid
                messageEvent = robotics.ikdesigner.internal.event.MessageEvent(errMsg);
                notify(obj, 'RequestUIAlert', messageEvent);
                return;
            end


            % Lastly, try to load the session. If this fails, set the flag
            % to indicate that the model is corrupted and return.
            try
                obj.setupFromModel(loadedFileData.appdata);
            catch
                isAppCorrupted = true;
                return;
            end

            % If all the load validation passed and the model was loaded,
            % set a flag to indicate as much
            isDataLoaded = true;

        end
    end

    methods (Access = private)
        function setupFromModel(obj, appdata)
            obj.SharedModelState.setupFromModel(appdata.SharedModelState);
            obj.SceneModel.setup(appdata.SceneModel);
            obj.SolverModel.setup(appdata.SolverModel);
            obj.ConfigModel.setup(appdata.ConfigModel);
            obj.AppStateModel.setup(appdata.AppStateModel);
        end

        function [isValid, errMsg] = validateAppSessionData(obj, loadedModelData)

            % Initialize output
            isValid = true;
            errMsg = string.empty;

            % Make sure the loaded model data contains the appdata variable
            f = fieldnames(loadedModelData);
            if ~contains(f, "appdata")
                isValid = false;
                errMsg = obj.INVALIDSESSIONERR;
                return;
            end

            % Check that app data is the correct type
            appdata = loadedModelData.appdata;
            if ~isa(appdata, "robotics.ikdesigner.internal.model.AppModels")
                isValid = false;
                errMsg = obj.INVALIDSESSIONERR;
                return;
            end

            % From here, the verification is on the content of the app
            % models, meaning that any difference in data is likely due to
            % user modification or version changes. An error message is at
            % best a diagnostic tool, so the output will be a combination
            % of a parent message that indicates the nature of the error,
            % and a specific message from the actual problem.
            parentErrMsg = obj.INVALIDAPPMODELHEADER;

            % Check that the AppModels object has the expected fields and
            % they have the correct types
            actAppdataPropNames = fieldnames(appdata);
            expAppdataPropNames = fieldnames(obj);
            try
                % The app model should have the same number of fields, and
                % they should have the same names. Uniqueness is assumed
                % since they are object properties.
                validateattributes(appdata, "robotics.ikdesigner.internal.model.AppModels","nonempty",'ikdesigner','appdata');
                validateattributes(actAppdataPropNames, "cell",{'numel', numel(expAppdataPropNames)},'ikdesigner','appdata property names');
                cellfun(@(s)validatestring(s, expAppdataPropNames), actAppdataPropNames, 'UniformOutput', false);

                % Each of the fields should have the same type as the
                % equivalent model field and be non-empty.
                for i = 1:numel(expAppdataPropNames)
                    propToValidate = expAppdataPropNames{i};
                    expClass = class(obj.(propToValidate));
                    validateattributes(appdata.(propToValidate), {expClass},"nonempty",'ikdesigner',sprintf('appdata.%s', propToValidate));
                end
            catch ME
                isValid = false;
                errMsg = string(ME.message);
            end

            % Assemble a complete error message
            errMsg = parentErrMsg + newline + newline + errMsg;

        end
    end
    
end
