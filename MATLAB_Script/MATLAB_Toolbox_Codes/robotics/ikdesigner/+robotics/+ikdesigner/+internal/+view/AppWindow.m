classdef AppWindow < robotics.ikdesigner.internal.view.View
%This function is for internal use only and may be removed in a future release

%AppWindow Top level app window

%   Copyright 2021-2022 The MathWorks, Inc.

    events
        RequestSaveSession
    end

    properties (SetAccess = {?robotics.ikdesigner.internal.view.View, ?matlab.unittest.TestCase})
        % Properties that represent AppContainer elements

        %Window Primary AppContainer element
        Window

        %CanClose Boolean that indicates whether app can be closed or save is required
        CanClose
    end

    properties (Dependent)
        SavedSessionCheck
    end

    properties (Access = {?robotics.ikdesigner.internal.InverseKinematicsDesigner, ?matlab.unittest.TestCase, ?robotics_tests.app.ikdesigner.IKAppTester})

        %FigureDocumentGroup Document area containing the figures
        FigureDocumentGroup

    end

    properties (Access = {?matlab.unittest.TestCase, ?robotics_tests.app.ikdesigner.IKAppTester})

        %SceneCanvasDocument Document that contains the scene canvas
        SceneCanvasDocument

        %SceneBrowserPanel Panel that contains the scene browser
        SceneBrowserPanel

        %SceneInspectorPanel Panel that contains the scene inspector
        SceneInspectorPanel

        %ConstraintsBrowserPanel Panel that contains the constraints browser
        ConstraintsBrowserPanel

        %ConfigurationsPanel Panel that contains the scene browser
        ConfigurationsPanel

        %HelpButton
        HelpButton

        %SessionName
        SessionName
    end

    properties (SetAccess = {?robotics.ikdesigner.internal.view.View, ?matlab.unittest.TestCase})

        %SceneCanvasView Handle to the scene canvas view
        SceneCanvasView

        %SceneBrowserView Handle to the scene browser view
        SceneBrowserView

        %SceneInspectorView Handle to the scene inspector view
        SceneInspectorView

        %ConstraintsBrowserView Handle to the constraints browser view
        ConstraintsBrowserView

        %ConfigurationsPanelView Handle to the configurations panel view
        ConfigurationsPanelView

        %ToolstripView Toolstrip view
        ToolstripView

        %StatusBarView Status bar view
        StatusBarView

    end

    properties (Constant)

        WINDOWTITLE = string(message('robotics:ikdesigner:appwindow:AppWindowTitle'))
        WINDOWTAG = "InverseKinematicsDesigner"

        % Properties used for the window when a dirty session is abandoned
        CONFIRMCONTINUETITLE = string(message('robotics:ikdesigner:appwindow:UnsavedChangesTitle'))
        CONFIRMCONTINUETEXT = string(message('robotics:ikdesigner:appwindow:UnsavedChangesText'))
        SAVEANDCONTINUEBTNLABEL = string(message('robotics:ikdesigner:appwindow:SaveAndContinueButtonLabel'))
        CONTINUEBTNLABEL = string(message('robotics:ikdesigner:appwindow:ContinueWithoutSavingButtonLabel'))
        CANCELBTNLABEL = string(message('robotics:ikdesigner:appwindow:CancelButtonLabel'))

        %Dimensions
        DEFAULTBOUNDS = [100 100 1400 800]  % Default window size (lower X, lower Y, width, height)
        DEFAULTLEFTPANELWIDTH = 250
        DEFAULTRIGHTPANELWIDTH = 325        % Chosen so the rigid body inertia table has no scroll bars at default width

        DEFAULTSESSIONNAME = string(message('robotics:ikdesigner:appwindow:DefaultSessionName'))

        FIGUREGROUPTITLE = string(message('robotics:ikdesigner:appwindow:FigureGroupTitle'))

        CONSTRAINTSBROWSERTITLE = string(message('robotics:ikdesigner:constraintsbrowser:ViewTitle'))

        SCENEBROWSERTITLE = string(message('robotics:ikdesigner:scenebrowser:ViewTitle'))

        SCENEINSPECTORTITLE = string(message('robotics:ikdesigner:sceneinspector:ViewTitle'))

        SCENECANVASTITLE = string(message('robotics:ikdesigner:scenecanvas:ViewTitle'))

        CONFIGURATIONSPANELTITLE = string(message('robotics:ikdesigner:configurationspanel:ViewTitle'))
    end


    methods
        function obj = AppWindow()
        %AppWindow Constructor

        % Each app session must have a unique tag
            persistent appIdx;
            if isempty(appIdx)
                appIdx = 1;
            end

            obj.initializeAppWindow(appIdx);
            appIdx = appIdx + 1;

            % Add component views to the window
            obj.createAppViewPanels;
            obj.createFigureDocumentGroup;
            obj.ToolstripView = robotics.ikdesigner.internal.view.ToolstripView(obj.Window);
            obj.StatusBarView = robotics.ikdesigner.internal.view.StatusBarView(obj.Window);

            % Initialize the scene canvas
            obj.setupSceneCanvasDocument;
            obj.SceneCanvasView = robotics.ikdesigner.internal.view.SceneCanvasView;

            % Add a help button
            obj.addHelpButton('inversekinematicsdesigner');

            % Assign a default session name
            obj.SessionName = obj.DEFAULTSESSIONNAME;

            % Assign the can-close callback
            obj.Window.CanCloseFcn = @(~,~)obj.confirmClose();
        end

        function setup(obj, sceneModel, solverModel, configModel)
        %setup Set up session
        %   When a new session is created or loaded from file, the
        %   setup method is used to reset existing data members and
        %   provide them with new stored session data.

            % Get derived data
            revJtAxesIdx = robotics.manip.internal.RigidBodyTreeUtils.identifyRevoluteJoint(sceneModel.RigidBodyTree);

            % Data related to scene content
            obj.setupSceneCanvasDocument;
            obj.SceneCanvasView.setup(obj.SceneCanvasDocument.Figure, sceneModel.RigidBodyTree, sceneModel.SceneObjectsMap, sceneModel.RigidBodyKeyMap, solverModel.LastSolutionEEPose);
            obj.SceneBrowserView.setup(sceneModel.RigidBodyTree, sceneModel.SceneObjectsMap, sceneModel.RigidBodyKeyMap);
            obj.ConstraintsBrowserView.setup(solverModel.ConstraintsMap);
            obj.ConfigurationsPanelView.setup(revJtAxesIdx, configModel.ConfigurationsMap, configModel.ConfigKeyArray);
            obj.SceneInspectorView.setup();
            obj.StatusBarView.setup(revJtAxesIdx);

            % Data related to solver content
            obj.ToolstripView.setup(sceneModel.RigidBodyTree, solverModel.IKSolver, sceneModel.RigidBodyKeyMap);

            % Set default app session
            obj.SessionName = obj.DEFAULTSESSIONNAME;
        end

        function initialize(obj)
        %initialize Initialize the app views

            initialize(obj.ToolstripView);
            initialize(obj.SceneCanvasView);
            initialize(obj.SceneBrowserView);
            initialize(obj.SceneInspectorView);
            initialize(obj.ConstraintsBrowserView);
            initialize(obj.ConfigurationsPanelView);
            initialize(obj.StatusBarView);

            obj.SavedSessionCheck = false;
        end

        function canProceed = confirmProceed(obj)
            %confirmProceed UI dialog that asks user whether or not they would like to continue with their action
  
                canProceed = true;
                confirmString = uiconfirm(obj.Window, obj.CONFIRMCONTINUETEXT, obj.CONFIRMCONTINUETITLE, ...
                    "Options", [obj.SAVEANDCONTINUEBTNLABEL obj.CONTINUEBTNLABEL obj.CANCELBTNLABEL], ...
                    "DefaultOption", 1, 'CancelOption', 3, 'Icon', 'warning');
                if strcmp(confirmString, obj.SAVEANDCONTINUEBTNLABEL)
                    notify(obj, "RequestSaveSession");
                elseif strcmp(confirmString, obj.CANCELBTNLABEL)
                    canProceed = false;
                end
        end

        function alert(obj, msgEvent)
        %alert Produce a dialog that shows error messages

            uialert(obj.Window, msgEvent.String, "Error");
        end

        function show(obj)
        %show Launch app interface

            obj.Window.Visible = true;
        end

        function setSessionName(obj, sessionName)
            obj.SessionName = sessionName;
            obj.Window.Title = obj.WINDOWTITLE + " - " + obj.SessionName;
        end

        function set.SavedSessionCheck(obj, isSaved)
            if isSaved
                suffix = "";
            else
                suffix = "*";
            end
            obj.CanClose = isSaved;
            obj.Window.Title = obj.WINDOWTITLE + " - " + obj.SessionName + suffix;
        end

        function isSaved = get.SavedSessionCheck(obj)
            isSaved = obj.CanClose;
        end
    end

    methods (Access = private)
        function initializeAppWindow(obj, appIdx)

            obj.Window = matlab.ui.container.internal.AppContainer;
            obj.Window.Title = obj.WINDOWTITLE;
            obj.Window.Tag = obj.WINDOWTAG + num2str(appIdx);
            obj.Window.WindowBounds = obj.DEFAULTBOUNDS;
        end

        function createFigureDocumentGroup(obj)
        %createFigureDocumentGroup Create a figure document group
        %   All the figures are stored in the figure document group in
        %   the central part of the app window.

            obj.FigureDocumentGroup = matlab.ui.internal.FigureDocumentGroup;
            obj.FigureDocumentGroup.Title = obj.FIGUREGROUPTITLE;
            add(obj.Window, obj.FigureDocumentGroup);
        end

        function createAppViewPanels(obj)
        %createAppViewPanels Create panels that house views
        %   There are multiple panels in the app window that are used
        %   to parent figure views. Each panel hosts a uifigure, which
        %   in turn is composed of a view. In this way, the panels
        %   control how the different views are arranged in the primary
        %   app window.

            import robotics.ikdesigner.internal.helpers.UIElementFactory

            % Add a scene browser panel on the left
            [obj.SceneBrowserView, obj.SceneBrowserPanel] = UIElementFactory.createPanelView(obj.Window, ...
                                                                                             @(fig)robotics.ikdesigner.internal.view.SceneBrowserView(fig), ...
                                                                                             "Title", obj.SCENEBROWSERTITLE, ...
                                                                                             "Tag", "SceneBrowser", ...
                                                                                             "Region", "left", ...
                                                                                             "Index", 1, ...
                                                                                             "PreferredWidth", obj.DEFAULTLEFTPANELWIDTH);

            % Add a scene inspector panel on the right
            [obj.SceneInspectorView, obj.SceneInspectorPanel] = UIElementFactory.createPanelView(obj.Window, ...
                                                                                                 @(fig)robotics.ikdesigner.internal.view.SceneInspectorView(fig), ...
                                                                                                 "Title", obj.SCENEINSPECTORTITLE, ...
                                                                                                 "Tag", "SceneInspector", ...
                                                                                                 "Region", "right", ...
                                                                                                 "Index", 1, ...
                                                                                                 "PreferredWidth", obj.DEFAULTRIGHTPANELWIDTH);

            % Add the constraints browser on the left
            [obj.ConstraintsBrowserView, obj.ConstraintsBrowserPanel] = UIElementFactory.createPanelView(obj.Window, ...
                                                                                                         @(fig)robotics.ikdesigner.internal.view.ConstraintsBrowserView(fig), ...
                                                                                                         "Title", obj.CONSTRAINTSBROWSERTITLE, ...
                                                                                                         "Tag", "ConstraintsBrowser", ...
                                                                                                         "Region", "left", ...
                                                                                                         "Index", 2, ...
                                                                                                         "PreferredWidth", obj.DEFAULTLEFTPANELWIDTH);

            % Add the configurations panel in the center
            defaultCenterWidth = obj.DEFAULTBOUNDS(3) - obj.DEFAULTLEFTPANELWIDTH - obj.DEFAULTRIGHTPANELWIDTH;
            [obj.ConfigurationsPanelView, obj.ConfigurationsPanel] = UIElementFactory.createPanelView(obj.Window, ...
                                                                                                      @(fig)robotics.ikdesigner.internal.view.ConfigurationsPanelView(fig), ...
                                                                                                      "Title", obj.CONFIGURATIONSPANELTITLE, ...
                                                                                                      "Tag", "ConfigurationsPanel", ...
                                                                                                      "Region", "bottom", ...
                                                                                                      "Index", 1, ...
                                                                                                      "PreferredWidth", defaultCenterWidth, ...
                                                                                                      "PreferredHeight", 270);

        end

        function addHelpButton(obj, docName)
        %addHelpButton Add a help button to the app window

            obj.HelpButton = matlab.ui.internal.toolstrip.qab.QABHelpButton();
            obj.HelpButton.DocName = docName;
            obj.Window.add(obj.HelpButton);
        end

        function updateTitle(obj, newTitle, isSaved)
        %updateTitle Update the app title with session data and saved state

            if strlength(newTitle) > 0
                suffix = " - " + newTitle;
            else
                suffix = "";
            end
            obj.Window.Title = obj.WINDOWTITLE + suffix;
            if isSaved
                obj.setSaved();
            else
                obj.setDirty();
            end
        end

        function setDirty(obj)
        %setDirty Update app title to indicate a dirty state

            currentTitle = obj.Window.Title;
            if ~endsWith(currentTitle, "*")
                obj.Window.Title = currentTitle + "*";
            end
        end

        function setSaved(obj)
        %setSaved Update app title to indicate a saved state

            currentTitle = obj.Window.Title;
            if endsWith(currentTitle, "*")
                obj.Window.Title = replaceBetween(currentTitle, strlength(currentTitle), strlength(currentTitle), "");
            end
        end

        function canCloseApp = confirmClose(obj)
            %confirmClose Check whether the app can be closed

            if ~obj.CanClose
                confirmString = uiconfirm(obj.Window, obj.CONFIRMCONTINUETEXT, obj.CONFIRMCONTINUETITLE, ...
                    "Options", [obj.SAVEANDCONTINUEBTNLABEL obj.CONTINUEBTNLABEL obj.CANCELBTNLABEL], ...
                    "DefaultOption", 1, 'CancelOption', 3, 'Icon', 'warning');
                if strcmp(confirmString, obj.SAVEANDCONTINUEBTNLABEL)
                    notify(obj, "RequestSaveSession");
                elseif strcmp(confirmString, obj.CONTINUEBTNLABEL)
                    obj.CanClose = true;
                end
            end
            canCloseApp = obj.CanClose;
        end

        function setupSceneCanvasDocument(obj)
        %setupSceneCanvasDocument Set up the figure document group for a new session
        %   In this app, here is only one figure in the figure document
        %   group: the scene canvas. This method sets up the figure
        %   document group for a new session by closing any old open
        %   figure windows and creating a new one in its stead.

        % In this app, only one scene canvas figure can exist at one
        % time
            if ~isempty(obj.SceneCanvasDocument)
                obj.SceneCanvasDocument.close();
                delete(obj.SceneCanvasDocument);
            end

            % Add a document that contains the Scene Canvas UIFigure
            obj.SceneCanvasDocument = matlab.ui.internal.FigureDocument("Title",obj.SCENECANVASTITLE, ...
                                                                        "DocumentGroupTag", obj.FigureDocumentGroup.Tag, ...
                                                                        "Closable", false);
            add(obj.Window, obj.SceneCanvasDocument);
        end
    end
end
