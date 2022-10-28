classdef AppControllers < handle
    %This class if for internal use only and may be removed in a future release
    
    %AppControllers Object composed of all app controllers
    %   This app uses a paradigm in which each controller is pertains to a
    %   particular segment, but more concretely, each controller has
    %   listeners for only one model (and multiple relevant views). This
    %   strategy helps to ensure that each controller facilitates only an
    %   appropriate subset of communication. The controllers do send
    %   commands to other models, but they only listen to the model that
    %   most closely pertains to the relevant domain, as clearly indicated
    %   by the matching nomenclature.

    %   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = ?matlab.unittest.TestCase)

        %SceneController Controller that listens to SceneModel
        %   The SceneController has listeners on the scene model and views
        %   that pertain to the scene. It facilitates communication
        %   regarding the contents and state of the scene.
        SceneController

        %SolverController Controller that listens to SolverModel
        %   The SolverController has listeners on the solver model and
        %   views that can trigger to solver model. It facilitates
        %   communication regarding the solver actions and solutions.
        SolverController

        %ConfigController Controller that listens to ConfigModel
        %   The ConfigController has listeners on the configurations model
        %   and views that can trigger to configurations model. It
        %   facilitates communication regarding the stored configurations
        %   and associated stored state data.
        ConfigController

        %AppStateController Controller that listens to AppStateModel
        %   The AppStateController has listeners on the app state model and
        %   views that can trigger to app state model. It facilitates
        %   communication regarding the higher-level app state.
        AppStateController

    end

    methods
        function obj = AppControllers(appModels, appViews)
            %AppControllers Constructor

            % All the controllers exist for the life of the app. They do
            % not store data besides the handles to the associated models
            % and views.
            obj.SceneController = robotics.ikdesigner.internal.controller.SceneController(appModels, appViews);
            obj.SolverController = robotics.ikdesigner.internal.controller.SolverController(appModels, appViews);
            obj.ConfigController = robotics.ikdesigner.internal.controller.ConfigController(appModels, appViews);
            obj.AppStateController = robotics.ikdesigner.internal.controller.AppStateController(appModels, appViews);

        end

        function initialize(obj)
            %initialize Initialize all controllers
            %   Controllers don't store data besides handles to the models
            %   and views they communicate with, so initialization
            %   predominantly refers to setting up listeners.

            obj.AppStateController.initialize();
            obj.SceneController.initialize();
            obj.ConfigController.initialize();
            obj.SolverController.initialize();
        end
    end

end
