classdef AppStateModel < robotics.ikdesigner.internal.model.Model
    %This class if for internal use only and may be removed in a future release
    
    %AppStateModel Model object that stores data pertaining to session state
    
    %   Copyright 2021 The MathWorks, Inc.
    
    events
        StateChanged
    end
    
    properties (SetAccess = private)
        AppState
    end

    properties
        SavedSessionName
        
        SavedSessionCheck
    end
    
    methods
        function obj = AppStateModel
            %AppStateModel Constructor

            obj.AppState = [];
            obj.SavedSessionName = string.empty;
            obj.SavedSessionCheck = false;
            
        end
        
        function setup(obj, modelData)
            %setup Method to start from a specific state
            obj.AppState = robotics.ikdesigner.internal.model.AppState.Startup;
            obj.SavedSessionCheck = false;
            obj.SavedSessionName = string.empty;
            
            if nargin > 1
                obj.SavedSessionName = modelData.SavedSessionName;
            end
        end
        
        function initialize(~)
            %initialize Initialize a new session
            
        end
        
        function goto(obj, state)
            %goto Go to the specified app state
            
            obj.AppState = state;
            notify(obj, 'StateChanged');
        end


    end
end