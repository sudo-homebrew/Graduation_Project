classdef AppViewOrganizer < handle & ...
        robotics.appscore.internal.mixin.MsgCatalogHelper & ...
        nav.slamapp.internal.mixin.CustomizedLayouts

    %This class is for internal use only. It may be removed in the future.

    %APPVIEWORGANIZER This class arranges the tabs and figure documents in
    %   the SLAM app. The APPVIEWORGANIZER can be instantiated by itself
    %   without connection to controller, and in this state, it is
    %   just a "dumb" UI with pre-defined event listener interface and
    %   doesn't do any real stuff.

    % Copyright 2018-2022 The MathWorks, Inc.

    properties
        %ToolGroup This is the top-level App container
        ToolGroup

        %TabGroup Tab group that holds app tabs. In general one is enough
        TabGroup
    end

    properties % figure documents in the app

        %MapFigDoc
        MapFigDoc

        %LoopClosureFigDoc
        LoopClosureFigDoc

        %IncrementalFigDoc
        IncrementalFigDoc

        %InfoFigDoc
        InfoFigDoc

        %ScanFigDoc
        ScanFigDoc

        %OdomFigDoc
        OdomFigDoc

        %InfoWSVarsFigDoc
        InfoWSVarsFigDoc
    end

    properties %tabs

        %MapBuilderTab Also the main tab
        MapBuilderTab

        %BagImporterTab Tab for importing data from rosbag
        BagImporterTab

        %WSImporterTab Tab for importing data from MATLAB workspace
        WSImporterTab

        %ModificationTab Tab for modifying scan matchings
        ModificationTab

    end

    properties % layouts
        DefaultMainTiling

        DefaultMainTiling1

        DefaultImportTiling

        DefaultImportTiling1

        LoopClosureFigOnlyTiling

        IncrementalFigOnlyTiling
    end


    methods
        function obj = AppViewOrganizer()
        %AppViewOrganizer Constructor
            import matlab.ui.internal.toolstrip.*
            import nav.slamapp.internal.*

            obj.MsgIDPrefix = 'nav:navslamapp:appvieworganizer';

            title = obj.retrieveMsg('AppTitle');
            name = 'SLAMApp';
            % Setting the app default layout to DefaultMainTiling1. This is
            % useful for proper app layout rendering upon initialization.
            defaultLayout = obj.createLayout3F('main1');

            obj.ToolGroup = nav.slamapp.internal.AppContainerWrapper(title,name,defaultLayout);
            obj.ToolGroup.setPosition([100 100 1080 900]);

            obj.ToolGroup.setHelpCallback('SLAMMapBuilder');


            obj.TabGroup = TabGroup();
            obj.TabGroup.Tag = 'SLAMApp';
            obj.MapBuilderTab = MapBuilderTab(obj.TabGroup);
            obj.BagImporterTab = BagImporterTab(obj.TabGroup);
            obj.WSImporterTab = WorkspaceImporterTab(obj.TabGroup);
            obj.ModificationTab = ModificationTab(obj.TabGroup);

            obj.ToolGroup.addTabGroup(obj.TabGroup);

            % open the app first then add figure for quick response
            obj.ToolGroup.open();
            obj.freeze(); % only after toolgroup opens
            
            obj.createFigures;
            
            obj.MapBuilderTab.setWidgetAvailability(0); % turn off all buttons upon start
            
            obj.addFigures;
            
            obj.refreshAppViewAccordingToState(0);
        end

        function delete(obj)
            %delete Destructor
            if ~isempty(obj.ToolGroup) && isvalid(obj.ToolGroup)
                obj.ToolGroup.close('force',true);
                obj.MapFigDoc.delete;
                obj.LoopClosureFigDoc.delete;
                obj.IncrementalFigDoc.delete;
                obj.InfoFigDoc.delete;
                obj.InfoWSVarsFigDoc.delete;
                obj.ScanFigDoc.delete;
                obj.OdomFigDoc.delete;
            end
        end



        function createFigures(obj)
            %createFigures
            import nav.slamapp.internal.*

            % map builder (main) figures
            obj.MapFigDoc = MapFigureDocument('MapFigure');
            obj.LoopClosureFigDoc = LoopClosureFigureDocument('LoopClosureFigure');
            obj.IncrementalFigDoc = IncrementalFigureDocument('IncrementalFigure');

            % importer figures
            obj.InfoFigDoc = InfoFigureDocument('InfoFigure');
            obj.InfoWSVarsFigDoc = InfoWSVarsFigureDocument('InfoWSVarsFigure');
            obj.ScanFigDoc = ScanFigureDocument('ScanFigure');
            obj.OdomFigDoc = OdomFigureDocument('OdomFigure');

            % default layouts
            obj.DefaultMainTiling = obj.createLayout3F('main');

            obj.DefaultMainTiling1 = obj.createLayout3F('main1');

            obj.DefaultImportTiling = obj.createLayout3F('main1');

            obj.DefaultImportTiling1 = obj.createLayout3F('main1');
          
            obj.LoopClosureFigOnlyTiling = obj.createLayout1F('loop');

            obj.IncrementalFigOnlyTiling = obj.createLayout1F('incremental');
        end

        function addFigures(obj)
            %addFigures

            obj.ToolGroup.addFigure(obj.MapFigDoc.FigureDoc);
            obj.ToolGroup.addFigure(obj.LoopClosureFigDoc.FigureDoc);
            obj.ToolGroup.addFigure(obj.IncrementalFigDoc.FigureDoc);

            obj.ToolGroup.addFigure(obj.InfoFigDoc.FigureDoc);
            obj.ToolGroup.addFigure(obj.InfoWSVarsFigDoc.FigureDoc);
            obj.ToolGroup.addFigure(obj.ScanFigDoc.FigureDoc);
            obj.ToolGroup.addFigure(obj.OdomFigDoc.FigureDoc);
        end


        function showDefaultMapBuilderFigureDocuments(obj, idx)
            %showDefaultMapBuilderFigureDocuments
            
            % make sure the figures are turned on in tile order.
            % When app container figure document visible property
            % is set to false app container hides it from the user.
            obj.MapFigDoc.FigureDoc.Visible = true;
            obj.LoopClosureFigDoc.FigureDoc.Visible = true;
            obj.IncrementalFigDoc.FigureDoc.Visible = true;

            obj.InfoFigDoc.FigureDoc.Visible = false;
            obj.InfoWSVarsFigDoc.FigureDoc.Visible = false;
            obj.ScanFigDoc.FigureDoc.Visible = false;
            obj.OdomFigDoc.FigureDoc.Visible = false;

            if idx == 0
                obj.applyDocumentTiling(obj.DefaultMainTiling1);
            else
                obj.applyDocumentTiling(obj.DefaultMainTiling);
            end             
            
            % bring increamental figure to focus by default in map builder
            % view, this is helpful for key board shortcuts
            obj.ToolGroup.selectFigureDocAndBringToFocus(obj.IncrementalFigDoc);
        end


        function showDefaultBagImporterFigureDocuments(obj)
            %showDefaultBagImporterFigureDocuments
            %   This is the "import from rosbag" view

            % When app container figure document visible property
            % is set to false app container hides it from the user.
            obj.InfoFigDoc.FigureDoc.Visible = true;
            obj.InfoWSVarsFigDoc.FigureDoc.Visible = false;
            obj.ScanFigDoc.FigureDoc.Visible = true;
            obj.OdomFigDoc.FigureDoc.Visible = true;
            
            obj.MapFigDoc.FigureDoc.Visible = false;
            obj.LoopClosureFigDoc.FigureDoc.Visible = false;
            obj.IncrementalFigDoc.FigureDoc.Visible = false;
            
            obj.applyDocumentTiling(obj.DefaultImportTiling);

            % bring odometry figure to focus by default in workspace
            % importer view, this is helpful for key board shortcuts
            obj.ToolGroup.selectFigureDocAndBringToFocus(obj.OdomFigDoc);
        end

        function showDefaultWSImporterFigureDocuments(obj)
            %showDefaultWSImporterFigureDocuments
            %   This is the "import from workspace variables" view

            % make sure the figures are turned on in tile order.
            % When app container figure document visible property
            % is set to false app container hides it from the user.
            obj.InfoFigDoc.FigureDoc.Visible = false;
            obj.InfoWSVarsFigDoc.FigureDoc.Visible = true;
            obj.ScanFigDoc.FigureDoc.Visible = true;
            obj.OdomFigDoc.FigureDoc.Visible = true;
            
            obj.MapFigDoc.FigureDoc.Visible = false;
            obj.LoopClosureFigDoc.FigureDoc.Visible = false;
            obj.IncrementalFigDoc.FigureDoc.Visible = false;
            
            obj.applyDocumentTiling(obj.DefaultImportTiling1);

            % bring odometry figure to focus by default in workspace
            % importer view, this is helpful for key board shortcuts
            obj.ToolGroup.selectFigureDocAndBringToFocus(obj.OdomFigDoc);
        end

        function ShowLoopClosureFigureDocumentOnly(obj)
            %ShowLoopClosureFigureDocumentOnly

            % When app container figure document visible property
            % is set to false app container hides it from the user.
            obj.MapFigDoc.FigureDoc.Visible = false;
            obj.LoopClosureFigDoc.FigureDoc.Visible = true;
            obj.IncrementalFigDoc.FigureDoc.Visible = false;

            obj.InfoFigDoc.FigureDoc.Visible = false;
            obj.InfoWSVarsFigDoc.FigureDoc.Visible = false;
            obj.ScanFigDoc.FigureDoc.Visible = false;
            obj.OdomFigDoc.FigureDoc.Visible = false;

            obj.applyDocumentTiling(obj.LoopClosureFigOnlyTiling);
        end


        function ShowIncrementalFigureDocumentOnly(obj)
            %ShowIncrementalFigureDocumentOnly

            % When app container figure document visible property
            % is set to false app container hides it from the user.
            obj.MapFigDoc.FigureDoc.Visible = false;
            obj.LoopClosureFigDoc.FigureDoc.Visible = false;
            obj.IncrementalFigDoc.FigureDoc.Visible = true;

            obj.InfoFigDoc.FigureDoc.Visible = false;
            obj.InfoWSVarsFigDoc.FigureDoc.Visible = false;
            obj.ScanFigDoc.FigureDoc.Visible = false;
            obj.OdomFigDoc.FigureDoc.Visible = false;

            obj.applyDocumentTiling(obj.IncrementalFigOnlyTiling);
        end

        function applyDocumentTiling(obj,layout)
            %applyDocumentTiling

            obj.ToolGroup.Layout = layout;
            obj.ToolGroup.bringToFront;
        end

        %%
        function freeze(obj)
        %freeze
            obj.ToolGroup.setWaiting(true);
        end

        function thaw(obj)
        %thaw
            obj.ToolGroup.setWaiting(false);
        end

        function freezeWorkingArea(obj)
        %freezeWorkingArea
            % disable all ui elements in working area
            % disable map figure slider
            slider = obj.MapFigDoc.Figure.findobj('Tag','MapFigSlider_SliderContainer');
            slider.Enable = 'off';
    
            % disable all buttons in loop closure figure document
            % loop closure prev, next buttons and loop closure modified
            % icon button (3 buttons exist in loop closure figure)
            buttons = obj.LoopClosureFigDoc.Figure.findobj('Type','uibutton');
    
            for k = 1:length(buttons)
                buttons(k).Enable = 'off';
            end
        end

        function thawWorkingArea(obj)
        %thawWorkingArea
            % enable all ui elements in working are
            % enable map figure slider
            slider = obj.MapFigDoc.Figure.findobj('Tag','MapFigSlider_SliderContainer');
            slider.Enable = 'on';
    
            % enable all buttons in loop closure figure document
            % loop closure prev, next buttons and loop closure modified
            % icon button (3 buttons exist in loop closure figure)
            buttons = obj.LoopClosureFigDoc.Figure.findobj('Type','uibutton');
    
            for k = 1:length(buttons)
                buttons(k).Enable = 'on';
            end
        end

        function wipeMainWorkingArea(obj)
        %wipeMainWorkingArea
            obj.MapFigDoc.Slider.ContainerPanel.Visible = 'off';
            obj.MapFigDoc.restoreToInitState;
            obj.IncrementalFigDoc.restoreToInitState;
            obj.LoopClosureFigDoc.restoreToInitState;
        end

        %%
        function refreshAppViewAccordingToState(obj, state)
        %refreshAppViewAccordingToState
            import nav.slamapp.internal.*
            import matlab.ui.internal.toolstrip.*
            %obj.freeze();
            switch(state)
                case States.Init
                    obj.MapBuilderTab.show;
                    obj.BagImporterTab.hide;
                    obj.WSImporterTab.hide;
                    obj.ModificationTab.hide;
                    obj.showDefaultMapBuilderFigureDocuments(0);
                    obj.MapBuilderTab.setWidgetAvailability(3586); % this is [1 1 1  0 0 0  0 0 0  0 1 0] with left msb

                case States.LoadingBag
                    obj.MapBuilderTab.hide;
                    obj.BagImporterTab.show;
                    obj.WSImporterTab.hide;
                    obj.showDefaultBagImporterFigureDocuments;
                    obj.thaw();

                case States.LoadingFromWS
                    obj.MapBuilderTab.hide;
                    obj.BagImporterTab.hide;
                    obj.WSImporterTab.show;
                    obj.showDefaultWSImporterFigureDocuments;
                    obj.thaw();

                case States.SensorDataReady
                    obj.MapBuilderTab.show;
                    obj.BagImporterTab.hide;
                    obj.WSImporterTab.hide;
                    obj.showDefaultMapBuilderFigureDocuments(0);
                    obj.MapBuilderTab.setWidgetAvailability(3906); % this is [1 1 1  1 0 1  0 0 0  0 1 0 ] with left msb

                    obj.MapBuilderTab.RunButton.Icon = Icon.RUN_24;
                    obj.MapBuilderTab.RunButton.Text = obj.MapBuilderTab.retrieveMsg('RunButtonName');
                    obj.MapBuilderTab.RunButton.Description = obj.MapBuilderTab.retrieveMsg('RunButtonDescription');

                    obj.MapFigDoc.InfoText.Text = obj.MapFigDoc.retrieveMsg('ReminderToClickBuild');
                    obj.MapFigDoc.InfoText.BackgroundColor = obj.MapFigDoc.BannerBackgroundColor;
                    obj.MapFigDoc.Banner.BackgroundColor = obj.MapFigDoc.BannerBackgroundColor;

                    obj.wipeMainWorkingArea;

                    obj.MapFigDoc.Banner.Visible = 'on';

                case States.Mapping
                    obj.MapBuilderTab.setWidgetAvailability(64); % this is [0 0 0 0 0 1 0 0 0 0 0 0] with left msb
                    obj.MapFigDoc.InfoText.BackgroundColor = obj.MapFigDoc.SyncColor;
                    obj.MapFigDoc.Banner.BackgroundColor = obj.MapFigDoc.SyncColor;
                    obj.MapFigDoc.InfoText.Text =  obj.MapFigDoc.retrieveMsg('ReminderToSyncMap');
                    obj.MapFigDoc.showBanner(0);

                case States.MappingPaused
                    obj.MapBuilderTab.notify('MapBuilderTab_RefreshToolstrip_MappingPaused');
                    obj.MapBuilderTab.show;
                    obj.ModificationTab.hide;
                    obj.BagImporterTab.hide;
                    obj.WSImporterTab.hide;
                    obj.MapFigDoc.Slider.thaw();
                    obj.showDefaultMapBuilderFigureDocuments(0);

                case States.Mapped
                    obj.MapBuilderTab.notify('MapBuilderTab_RefreshToolstrip_Mapped');
                    obj.MapBuilderTab.show;
                    obj.ModificationTab.hide;
                    obj.BagImporterTab.hide;
                    obj.WSImporterTab.hide;
                    obj.MapFigDoc.Slider.thaw();
                    obj.showDefaultMapBuilderFigureDocuments(0);

                case States.ModifyingLoopClosure
                    obj.MapBuilderTab.hide;
                    obj.ModificationTab.show;
                    obj.MapFigDoc.Slider.freeze();
                    obj.ShowLoopClosureFigureDocumentOnly();

                case States.ModifyingIncremental
                    obj.MapBuilderTab.hide;
                    obj.ModificationTab.show;
                    obj.MapFigDoc.Slider.freeze();
                    obj.ShowIncrementalFigureDocumentOnly();
                otherwise
            end
            %obj.thaw();
        end

    end
end