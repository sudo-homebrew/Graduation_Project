classdef (ConstructOnLoad) FigureManager < handle
    %This class is for internal use only. It may be removed in the future.
    
    %FigureManager Provides infrastructures for RigidBodyTree plot
    %   interaction, such as display a banner message at the top of the
    %   figure when a body is clicked.
    %
    %   Only one FigureManager object is created per Parent figure,
    %   subsequent attempts to create a FigureManager object with the same
    %   Parent will return a reference to the existing FigureManager object.
    %
    %   The class constructor will be invoked when the figure manager object
    %   is reloaded from a saved .fig file
    
    %   Copyright 2017-2021 The MathWorks, Inc.
    
    properties (Constant)
        
        %BackgroundColor Background color for message banner
        BackgroundColor = [253 231 65]./255 % bright yellow
        
        %FontSize Font size for text in top message banner
        FontSize = 10
        
        %FigureManagerIdentifier
        FigureManagerIdentifier = 'rbt_figure_manager_object'
        
        %RobotShowTagsIdentifier
        RobotShowTagsIdentifier = 'robot_show_tags'
        
    end
    
    properties (Transient)
        
        Listeners
        
        CameraPropListeners
        
        Parent
        
        InfoPanel
        
        InfoText
        
    end
    
    properties
        
        BannerText ={'robotics system toolbox'}
        
        BannerTextHeight = 12
        
    end
    
    methods
        function obj = FigureManager(ax, optStruct)
            %FigureManager Constructor
            %   The input ax is a valid axes handle and an optional
            %   structure that allows customization of the figure when the
            %   figure manager is constructed. This is used to disable
            %   incompatible UI components.
            
            if nargin > 0
                obj.Parent = ax.Parent;
            else
                obj.Parent = gcf; % when reload from .fig
            end
            
            if nargin < 1
                optStruct = robotics.manip.internal.createFigureManagerOpts();
            end
            
            name = robotics.manip.internal.FigureManager.FigureManagerIdentifier;
            retObj = getappdata(obj.Parent, name);
            
            % if a FigureManager handle is not found in the figure, create
            % a new one
            if isempty(retObj) || ~isvalid(retObj)
                
                % wipe out existing corner coordinate frame axes, if any
                ccfaxs = findall(obj.Parent, 'type', 'axes', 'tag', 'CornerCoordinateFrame');
                for k = 1:length(ccfaxs)
                    delete(ccfaxs(k));
                end
                
                % re-create corner coordinate frames for axes that have robots
                axs = findall(obj.Parent, 'type', 'axes');
                for k = 1:length(axs)
                    p = findall(axs(k), 'type','patch');
                    showTagPrefix = robotics.manip.internal.VisualizationInfo.ShowTagPrefix;
                    result = arrayfun( @(x) strncmp(x.Tag, showTagPrefix, length(showTagPrefix)), p );
                    if ~isempty(result) && any(result)
                        obj.createCornerCoordinateFrame(axs(k));
                    end
                end
                
                % save FigureManager handle to application-defined data in
                % the figure
                setappdata(obj.Parent, name, obj);
                if optStruct.InitializeBannerWidgets
                    initializeBannerWidgets(obj);
                end
                
                % listeners
                obj.Listeners.DestroyFigure = addlistener(obj.Parent, 'ObjectBeingDestroyed', @(~,~)delete(obj));
                obj.Listeners.Resize = addlistener(obj.Parent, 'SizeChanged', @(~,~)resize(obj));

                % The following listener clears the figure selection when
                % the mouse is pressed anywhere in the figure
                if optStruct.EnableClearInfoOnMousePress
                    obj.Listeners.ClearInfo = addlistener(obj.Parent, 'WindowMousePress',@(~,~) clearInfo(obj));
                end
                
                % Just in case these are not turned on yet
                pa = pan(obj.Parent);
                rot3d = rotate3d(obj.Parent);
                zm = zoom(obj.Parent);
                
                rot3dmode = getuimode(obj.Parent, 'Exploration.Rotate3d');
                rot3dmode.WindowScrollWheelFcn = {@robotics.manip.internal.FigureManager.localScrollWheel};
                
                panmode = getuimode(obj.Parent, 'Exploration.Pan');
                panmode.WindowScrollWheelFcn = {@robotics.manip.internal.FigureManager.localScrollWheel};
                
                % callback filter
                rot3d.ButtonDownFilter = @robotics.manip.internal.FigureManager.redirectForPatch;
                
                pa.ButtonDownFilter = @robotics.manip.internal.FigureManager.redirectForPatch;
                
                zm.ButtonDownFilter = @robotics.manip.internal.FigureManager.redirectForPatch;
            else
                % otherwise just return the found handle
                obj = retObj;
            end
        end
        
        function delete(obj)
            %delete Class destructor
            
            % clean up listeners
            obj.Listeners = robotics.manip.internal.FigureManager.deleteListenerStruct(obj.Listeners);
            
            obj.CameraPropListeners = robotics.manip.internal.FigureManager.deleteListenerCellArray(obj.CameraPropListeners);
            
            % clean up widgets
            if ~isempty(obj.InfoPanel) && ishghandle(obj.InfoPanel)
                delete(obj.InfoPanel);
                obj.InfoPanel = [];
                obj.InfoText = []; % gets deleted with panel
            end
            
            
        end
        
        
        function initializeBannerWidgets(obj)
            obj.InfoPanel = uipanel( ...
                'Parent', obj.Parent, ...
                'HandleVisibility', 'off', ...
                'BorderType', 'line', ...
                'Units', 'pixels', ...
                'Position', [1 1 1 1], ...
                'BackgroundColor', obj.BackgroundColor, ...
                'Visible', 'off');
            
            obj.InfoText = uicontrol( ...
                'Parent', obj.InfoPanel, ...
                'HandleVisibility', 'off', ...
                'Style','text', ...
                'BackgroundColor', obj.BackgroundColor, ...
                'HorizontalAlignment', 'left', ...
                'FontSize', obj.FontSize, ...
                'FontWeight', 'bold', ...
                'Units', 'pixels', ...
                'Position', [1 1 1 1], ...
                'Visible', 'on');
            
        end
        
        
        function resize(obj)
            %resize Callback for SizeChanged event
            updateBannerPos(obj);
        end
        
        
        function updateBannerPos(obj)
            %updateBannerPos Invoked when the figure window resizes
            originalUnits = obj.Parent.Units;
            obj.Parent.Units = 'pixels'; % make sure the pos reading is in pixels
            figPos = obj.Parent.Position;
            obj.Parent.Units = originalUnits;
            
            
            % Text gets offset automatically past border of parent panel,
            % so (1,1) is up and right of lower-left border line.
            %
            % Panel must be:
            %   - full width of figure, regardless of text width
            %   - 2 pixels taller than text, to allow for border line
            %
            
            infoPanelHeight = obj.BannerTextHeight + 2;
            
            panelPos = [1 figPos(4)+1-infoPanelHeight figPos(3) infoPanelHeight];
            
            obj.InfoPanel.Position = panelPos;
            
            % The widest we can go with the text is figure width - 2:
            obj.InfoText.Position  = [1 1 figPos(3)-2 obj.BannerTextHeight];
            
            
        end
        
        function updateBannerText(obj)
            %updateBannerText
            t = obj.BannerText;
            if iscellstr(t)
                
                obj.InfoText.String = t;
                
                originalFU = obj.InfoText.FontUnits;
                obj.InfoText.FontUnits = 'pixels';
                fs = obj.InfoText.FontSize;
                obj.InfoText.FontUnits = originalFU;
                
                obj.BannerTextHeight = length(t)*(fs+2)+2;
            end
            updateBannerPos(obj);
            
        end
        
        
        function showBanner(obj)
            %showBanner
            if ~isempty(obj.InfoPanel) && ishghandle(obj.InfoPanel)
                obj.InfoPanel.Visible = 'on';
            end
        end
        
        function clearInfo(obj)
            %clearInfo Clear banner and highlighting on rigid body tree
            %   Call back to hide body info banner, clear body
            %   highlight and joint axis visuals in the current figure
            
            % hide banner
            if ~isempty(obj.InfoPanel) && ishghandle(obj.InfoPanel)
                obj.InfoPanel.Visible = 'off';
            end
            
            %find all axes in the current figure
            hFigure = obj.Parent;
            axs = findall(hFigure, 'Type', 'axes');
            showtags = getappdata(obj.Parent, robotics.manip.internal.FigureManager.RobotShowTagsIdentifier);
            
            for k = 1:length(axs)
                patchesInHighlight = findall(axs(k), 'Type', 'patch', 'LineStyle', '-');

                for i = 1:length(patchesInHighlight)
                    % make sure only RigidBodyTree related graphic objects
                    % are touched
                    if any(strcmp(showtags, patchesInHighlight(i).Tag))
                        patchesInHighlight(i).LineStyle = 'none';
                    end
                    
                end                
                robotics.manip.internal.FigureManager.clearJointAxis(axs(k));
            end
        end

        function clearPatchHighlights(obj, edgeColor)
            %clearPatchHighlights Reset edge highlights for all patches with a given edge color
            
            %find all axes in the current figure
            hFigure = obj.Parent;
            axs = findall(hFigure, 'Type', 'axes');
            
            for k = 1:length(axs)
                patchesInHighlight = findall(axs(k), 'Type', 'patch', 'LineStyle', '-', 'EdgeColor', edgeColor);

                for i = 1:length(patchesInHighlight)
                    % make sure only RigidBodyTree related graphic objects
                    % are touched
                    patchesInHighlight(i).LineStyle = 'none';                    
                end
                
                robotics.manip.internal.FigureManager.clearJointAxis(axs(k));
            end
        end
        
        
        function createCornerCoordinateFrame(obj, ax)
            %createCornerAxes Creates a corner coordinates frame for the
            %   given input axes AX that will indicate the orientation of
            %   3D scene rendered in AX
            
            % make sure that AX does not already have a listener
            for i = 1:length(obj.CameraPropListeners)
                if ax == obj.CameraPropListeners{i}.Object{1}
                    return;
                end
            end
            
            % so AX is not listened on
            axpos = ax.Position;
            axposOuter = ax.OuterPosition;
            ax2pos = [axposOuter(1)+0.015, axposOuter(2)+0.015, axpos(3)*0.2, axpos(4)*0.2];
            
            ax2 = axes('Parent', obj.Parent, ...
                'Position', ax2pos, ...
                'Color','none', ...
                'Tag', 'CornerCoordinateFrame', ...
                'HitTest', 'off', ...
                'Visible', 'off', ...
                'HandleVisibility', 'off');
            
            ax2.XLim = [-1,1];
            ax2.YLim = [-1,1];
            ax2.ZLim = [-1,1];
            axis(ax2,'vis3d');
            axis(ax2, 'equal');
            
            % defaultCamDistance = norm(ax.CameraPosition - ax.CameraTarget);
            defaultCamDistance = 18;
            
            % set up unidirectional listener for camera properties
            obj.CameraPropListeners{end+1} = ...
                ax.addlistener({'CameraUpVector', 'CameraPosition', 'CameraTarget', 'View', 'Projection'}, ...
                'PostSet', @(s,e)robotics.manip.internal.FigureManager.updateCamera(s, e, ax2, defaultCamDistance) );
            
            robotics.manip.internal.FigureManager.drawCornerCoordinateFrame(ax2);
            
            % sync up for the first time by triggering the property set event
            ax.CameraPosition = ax.CameraPosition;

            % however, setting CameraPosition directly also silently
            % changes cameraPositionMode to manual, which affects the zoom
            % behavior. So we have to change it back to auto.
            ax.CameraPositionMode = 'auto';
        end
        
    end
    
    
    methods (Static)
        function listeners = deleteListenerStruct(listeners)
            %deleteListenerStruct The input LISTENERS is a struct with
            %   listener handles retained in each field. This method
            %   identifies each field, deletes each listener, and
            %   replaces the handle with an empty in the struct.
            %
            %   If any field contains a struct, this function will
            %   recursively descend into that field.
            %
            %   LISTENERS may be an empty matrix or struct.
            %   This method returns the updated "empty" struct.
            
            if ~isempty(listeners)
                listenerNames = fieldnames(listeners);
                
                for i = 1:numel(listenerNames)
                    lname = listenerNames{i};
                    l = listeners.(lname);
                    if isstruct(l)
                        % Recurse on child struct:
                        listeners.(lname) = robotics.manip.internal.FigureManager.deleteListenerStruct(l);
                    else
                        % Delete listener handle
                        delete(l);
                    end
                    listeners.(lname) = []; % replace listener in field
                end
            end
            
        end
        
        function listeners = deleteListenerCellArray(listeners)
            %deleteListenerCellArray
            if ~isempty(listeners)
                for i = 1:length(listeners)
                    if iscell(listeners{i})
                        listeners{i} = robotics.manip.internal.FigureManager.deleteListenerCellArray(listeners{i});
                    else
                        delete(listeners{i});
                    end
                    listeners{i} = [];
                end
            end
        end
        
        
        function flag = redirectForPatch(src, evt) %#ok<INUSD>
            %redirectForPatch Callback function to intercept and filter
            %   button down events so that they won't be captured by the
            %   predefined uimodes
            %
            %   src   handle to the object that has been clicked on
            %   evt   handle to event data object (empty in this release)
            %
            %   The output flag is a logical that determines whether the
            %   rotate/pan/zoom operation should take place or the 'ButtonDownFcn'
            %   property of the clicked object should take precedence
            
            %fprintf('%s\n', src.Type);
            if strcmpi(src.Type, 'patch')
                flag = true;
            else
                flag = false;
            end
            
        end
        
        function updateCamera(src, evt, ax2, defaultCamDistance) %#ok<INUSL>
            %updateCamera Update the camera properties of ax2 such that
            %   they tracks those in ax.
            
            ax = evt.AffectedObject;
            
            if ~isempty(ax) && ~isempty(ax2)
                
                direction = ax.CameraPosition - ax.CameraTarget;
                ax2.CameraPosition = direction/norm(direction) * defaultCamDistance;
                ax2.CameraTarget = [0 0 0];
                ax2.CameraUpVector = ax.CameraUpVector;
                ax2.Projection = ax.Projection;
            end
            
        end
        
        function drawCornerCoordinateFrame(ax)
            %drawCornerCoordinateFrame
            hold(ax, 'on');
            onCleanup(@() hold(ax, 'off')); %#ok<UNONC>
            d = 0.8;
            plot3(ax, [0 d], [0 0], [0 0], 'r', 'linewidth', 2);
            plot3(ax, [0 0], [0 d], [0 0], 'g', 'linewidth', 2);
            plot3(ax, [0 0], [0 0], [0 d], 'b', 'linewidth', 2);
            d1 = 1;
            text(ax, d1, 0, 0, 'x', 'color', 'r');
            text(ax, 0, d1, 0, 'y', 'color', 'g');
            text(ax, 0, 0, d1, 'z', 'color', 'b');
        end
        
        
        function clearJointAxis(ax)
            %clearJointAxis Clear the highlighted joint axis from ax
            p = findall(ax, 'type', 'patch', 'Tag', 'HighlightedJointAxis');
            for i = 1:length(p)
                delete(p(i));
            end
        end
        
        
        function localScrollWheel(src, evt)
            %localScrollWheel
            currentAxes = src.CurrentAxes;
            zoomlevel = 0.9;
            if evt.VerticalScrollCount < 0
                camzoom(currentAxes,1/zoomlevel);
            elseif evt.VerticalScrollCount > 0
                camzoom(currentAxes,zoomlevel);
            end
        end
        
    end
end

