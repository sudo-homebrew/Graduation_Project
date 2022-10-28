classdef AppContainerWrapper < matlab.ui.container.internal.AppContainer
    %This class is for internal use only. It may be removed in the future.

    %APPCONTAINERWRAPPER This class is the replacement for java swing based
    %   matlab.ui.internal.desktop.ToolGroup. This is a thin wrapper around
    %   java script based AppContainer to support ToolGroup functionality
    %   setHelpCallback, open, setWaiting addFigure, similar constructor,
    %   setPosition and a utility method to bring a app container figure
    %   document to focus

    % Copyright 2021 The MathWorks, Inc.
    methods
        function obj = AppContainerWrapper(title,name,defaultLayout)
            %AppContainerWrapper construct app container object with
            %   specified name and title. In addition user can also specify
            %   the default app layout.
            
            AppContainerInfo.Tag = name;
            AppContainerInfo.Title = title;
            if (nargin > 2) && (~isempty(defaultLayout))
                AppContainerInfo.DefaultLayout = defaultLayout;
            end
            obj@matlab.ui.container.internal.AppContainer(AppContainerInfo);
            % tile layout change is not allowed
            obj.UserDocumentTilingEnabled = false;
        end
        
        function setHelpCallback(obj,appDocName)
            %setHelpCallback set the quick access bar help button pushed
            %   down callback
            
            qabbtn = matlab.ui.internal.toolstrip.qab.QABHelpButton();
            qabbtn.DocName = appDocName;
            obj.add(qabbtn);
        end
        
        function open(obj)
            %open make the app visible 
            
            if obj.State < 2
                % when the app is not in terminated state
                obj.Visible = true;
            end
        end
        
        function setWaiting(obj,waitingState,uiElement)
            %setWaiting set the waiting state of app container or the
            %   specified ui element. It only supports ui elements that have
            %   a setable property enabled.
            
            if nargin > 2
                uiElement.Enabled = ~waitingState;
            else
                % disable access to entire app
                obj.set("Busy",waitingState);
            end
        end
        
        function addFigure(obj, figDoc)
            %addFigure
            
            % every figure document must have a parent figure document
            % group added to app container before adding figure document
            figDocGroup = matlab.ui.internal.FigureDocumentGroup("Tag" , figDoc.Figure.Tag);
            obj.add(figDocGroup);
            
            obj.add(figDoc);
        end
        
        function setPosition(obj, pos)
            %setPosition sets position to 1x4 vector pos ([topLeftX,
            %   topLeftY, width, height])
            
            obj.WindowBounds = pos;
        end
    end
    
    methods (Static)
        function selectFigureDocAndBringToFocus(figDoc)
            %selectFigureDocAndBringToFocus
            
            % select the figure document 
            figDoc.FigureDoc.Selected = true;
            % bring the figure contained in the figure document to focus
            figure(figDoc.FigureDoc.Figure);
        end
    end
end