classdef InverseKinematicsDesigner < handle
    %This function is for internal use only. It may be removed in the future.
    
    %InverseKinematicsDesigner launches the inverse kinematics designer app
    
    %   Copyright 2021 The MathWorks, Inc.
    
    properties (SetAccess = private, GetAccess = {?matlab.unittest.TestCase, ?robotics_tests.app.ikdesigner.IKAppTester})
        
        %AppModels Object composed of all app models
        AppModels
        
        %AppWindow Main user-facing app view
        AppWindow
        
        %AppControllers Object composed of all app controllers
        AppControllers        
    end
    
    methods
        function obj = InverseKinematicsDesigner(sessionPathName)
            %InverseKinematicsDesigner Constructor

            % Validate inputs first
            if nargin > 0
                validateattributes(sessionPathName, {'char', 'string'}, {'scalartext'}, 'inverseKinematicsDesigner', 'sessionPathName');
                mustBeFile(sessionPathName);
                constructFromSession = true;
            else
                constructFromSession = false;
            end
            
            % The models are all part of the app model
            obj.AppModels = robotics.ikdesigner.internal.model.AppModels;
            
            % The views are all part of the app window
            obj.AppWindow = robotics.ikdesigner.internal.view.AppWindow;
            
            % The controllers act as routers between the model and views.
            % Each controller corresponds to a model, and listens only to
            % that model, but may listen to multiple views. The controllers
            % facilitate data routing between the models and views.
            obj.AppControllers = robotics.ikdesigner.internal.controller.AppControllers(obj.AppModels, obj.AppWindow);
            obj.AppControllers.initialize();
                
            % Show the app window
            obj.AppWindow.show();

            % Load a model if applicable
            if constructFromSession
                % Wait until the app has initialized
                waitfor(obj.AppWindow.FigureDocumentGroup, 'DocumentCount', 1);

                % Load session from file
                obj.AppControllers.AppStateController.loadSessionFromFilepath(string(sessionPathName));
            end
        end
    end
end

