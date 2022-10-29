classdef SLAMMapBuilder < handle ...
        & robotics.appscore.internal.mixin.MsgCatalogHelper
    %This class is for internal use only. It may be removed in the future.

    %SLAMMapBuilder This class is the entrance point for SLAM Map Builder app

    % Copyright 2018-2021 The MathWorks, Inc.

    properties
        AppViewOrg

        BagImporterModel
        WSImporterModel
        MapBuilderModel

        BagImporterController
        WSImporterController
        MapBuilderController

        ScanFigSliderModel
        ScanFigSliderController

        StateMachineModel
        StateMachineController

        SettingsDialog
        SettingsModel
        SettingsController

        MapFigSliderModel
        MapFigSliderController

        ModificationModel
        ModificationController
    end

    methods
        function obj = SLAMMapBuilder(varargin)
        %SLAMMapBuilder Constructor
            import nav.slamapp.internal.*
            import robotics.appscore.internal.*

            narginchk(0,2);

            obj.AppViewOrg = AppViewOrganizer();

            obj.SettingsDialog = SettingsDialog(obj.AppViewOrg.ToolGroup);

            obj.StateMachineModel = StateMachineModel();

            obj.ScanFigSliderModel = SliderModel();
            obj.MapFigSliderModel = SliderModel();
            obj.BagImporterModel = BagImporterModel();
            obj.WSImporterModel = WorkspaceImporterModel();
            obj.MapBuilderModel = MapBuilderModel();
            obj.SettingsModel = SettingsModel();
            obj.ModificationModel = ModificationModel();

            obj.ScanFigSliderController = SliderController(obj.ScanFigSliderModel, ...
                                                           obj.AppViewOrg.ScanFigDoc.Slider);

            obj.MapFigSliderController = SliderController(obj.MapFigSliderModel, ...
                                                          obj.AppViewOrg.MapFigDoc.Slider);

            obj.BagImporterController = BagImporterController(obj.BagImporterModel, ...
                                                              obj.MapBuilderModel, ...
                                                              obj.ScanFigSliderModel, ...
                                                              obj.StateMachineModel, ...
                                                              obj.AppViewOrg.BagImporterTab, ...
                                                              obj.AppViewOrg);
                                                          
            obj.WSImporterController = WorkspaceImporterController(obj.WSImporterModel, ...
                                                              obj.MapBuilderModel, ...
                                                              obj.ScanFigSliderModel, ...
                                                              obj.StateMachineModel, ...
                                                              obj.AppViewOrg.WSImporterTab, ...
                                                              obj.AppViewOrg);
            obj.MapBuilderController = MapBuilderController(obj.BagImporterModel, ...
                                                            obj.WSImporterModel, ...
                                                            obj.StateMachineModel, ...
                                                            obj.MapBuilderModel, ...
                                                            obj.SettingsModel, ...
                                                            obj.MapFigSliderModel, ...
                                                            obj.ScanFigSliderModel, ...
                                                            obj.ModificationModel, ...
                                                            obj.AppViewOrg.MapBuilderTab, ...
                                                            obj.AppViewOrg, ...
                                                            obj.SettingsDialog);

            obj.StateMachineController = StateMachineController(obj.StateMachineModel, obj.AppViewOrg);



            obj.SettingsController = SettingsController(obj.SettingsModel, ...
                                                        obj.BagImporterModel, ...
                                                        obj.WSImporterModel, ...
                                                        obj.StateMachineModel, ...
                                                        obj.MapBuilderModel, ...
                                                        obj.SettingsDialog, ...
                                                        obj.AppViewOrg.MapBuilderTab);


            obj.ModificationController = ModificationController(obj.ModificationModel, ...
                                                              obj.StateMachineModel, ...
                                                              obj.MapBuilderModel, ...
                                                              obj.AppViewOrg.ModificationTab, ...
                                                              obj.AppViewOrg);

            obj.MsgIDPrefix = 'nav:navslamapp:app';

            % with session or rosbag file name
            if (nargin == 1) && (ischar(varargin{1}) || isstring(varargin{1}))

                fileName = varargin{1};
                obj.AppViewOrg.freeze();
                try
                    data = load(fileName);
                    isMat = true;
                catch
                    isMat = false;
                end

                if isMat % it's a session file first
                    try
                        if ~isfield(data, 'slamAppSession')
                            [~, msgObj] = obj.AppViewOrg.MapBuilderTab.retrieveMsg('InvalidSessionFile', fileName);
                            error(msgObj);
                        end
                        obj.MapBuilderController.deserialize(fileName);
                        obj.MapBuilderController.SessionFileName = fileName;
                        % the SessionFileName must be set prior to calling 'setAppTitle'
                        obj.MapBuilderController.setAppTitle();
                        obj.AppViewOrg.thaw();
                    catch ME
                        delete(obj.AppViewOrg);
                        delete(obj);

                        rethrow(ME);
                    end
                else

                    % if not, is it a rosbag?
                                
                    if ~robotics.internal.license.isROSToolboxLicensed
                        delete(obj.AppViewOrg);
                        delete(obj);
                        nav.algs.internal.error('nav:navslamapp', 'app:UseWithROSBags');
                    end
                    [isSuccess, errTag, errID] = obj.MapBuilderController.extractBagInfo(fileName);
                    if isSuccess
                        obj.BagImporterModel.notify('BagImporterModel_RequestUpdateBagImporterTab');
                        obj.StateMachineModel.toLoadingBag();
                        obj.AppViewOrg.OdomFigDoc.clearOdom();
                    else
                        if ~isempty(errID) && (strcmp(errID, 'shared_robotics:validation:FileNotExist') || ...
                                               strcmp(errID, 'shared_robotics:validation:FilePathIsDir'))
                            [~, msgObj] = obj.getMsg(errID, fileName);
                        else
                            [~, msgObj] = obj.retrieveMsg(errTag, fileName);
                        end

                        delete(obj.AppViewOrg);
                        delete(obj);

                        error(msgObj);
                    end
                end



            % with array or cell array of lidarScan objects
            elseif (nargin == 1) && ( iscell(varargin{1}) || isa(varargin{1}, 'lidarScan'))
                
                scans = varargin{1};
                scansSizeIsValid = (numel(size(scans)) == 2) && (min(size(scans)) == 1);
                scansEntriesChecked = false;
                if scansSizeIsValid
                    if iscell(scans)
                        checkScans = cellfun(@(x) isa(x, 'lidarScan') && ~isempty(x), scans);
                    else
                        checkScans = arrayfun(@(x) ~isempty(x), scans);
                    end
                    scansEntriesChecked = ~isempty(checkScans) && all(checkScans);
                end
                scansAreValid = scansSizeIsValid && scansEntriesChecked; 
                
                if scansAreValid
                    if isa(scans, 'lidarScan')
                        scans = arrayfun(@(x) {x}, scans); % if it's an array of lidarScan objects, convert it to a cell array
                    end
                    scans = scans(:);
                    obj.MapBuilderModel.loadProcessedData(scans, {});
                    obj.StateMachineModel.State = States.LoadingFromWS;
                    obj.StateMachineModel.toSensorDataReady();
                else
                    obj.AppViewOrg.ToolGroup.close('force',true);
                    error(message('nav:navslamapp:app:InvalidInputScans'));
                end

            % with array or cell array of lidarScan objects + cell array or matrix of relative poses
            elseif (nargin == 2) && ( iscell(varargin{1}) || isa(varargin{1}, 'lidarScan')) && (iscell(varargin{2}) || isa(varargin{2}, 'double'))
                scans = varargin{1};
                
                % check size for scans
                scansSizeIsValid = (numel(size(scans)) == 2) && (min(size(scans)) == 1);
                
                % check each entry in scans
                scansEntriesChecked = false;
                if scansSizeIsValid
                    if iscell(scans)
                        checkScans = cellfun(@(x) isa(x, 'lidarScan') && ~isempty(x), scans);
                    else
                        checkScans = arrayfun(@(x) ~isempty(x), scans);
                    end
                    scansEntriesChecked = ~isempty(checkScans) && all(checkScans);
                end
               
                scansAreValid = scansSizeIsValid && scansEntriesChecked; 
                
                if ~scansAreValid
                    obj.AppViewOrg.ToolGroup.close('force',true);
                    error(message('nav:navslamapp:app:InvalidInputScans'));
                end
                
                
                poses = varargin{2};
                % check size for poses
                if iscell(poses)
                    posesSizeIsValid = (numel(size(poses)) == 2) && (min(size(scans)) == 1);
                    checkPoses = cellfun(@(x) isnumeric(x) && all(isfinite(x)) && numel(x) == 3 && ~isempty(x), poses);
                    posesEntriesChecked = ~isempty(checkPoses) && all(checkPoses);
                else % Nx3 matrix
                    posesSizeIsValid = (size(poses,2) == 3);
                    posesEntriesChecked = isa(poses, 'double') && isreal(poses) && all(all(isfinite(poses)));
                end
                posesAreValid = posesSizeIsValid && posesEntriesChecked;
                
                if ~posesAreValid
                    obj.AppViewOrg.ToolGroup.close('force',true);
                    error(message('nav:navslamapp:app:InvalidInputPoses'));
                end
                
                if isa(scans, 'lidarScan')
                    scans = arrayfun(@(x) {x}, scans); % if it's an array of lidarScan objects, convert it to a cell array
                end
                scans = scans(:);
                
                if isa(poses, 'double')
                    poses = mat2cell(poses, ones(size(poses,1),1), 3);
                end
                poses = poses(:);
                
                % the number of poses must match number of scans
                if numel(scans) ~= numel(poses)
                    obj.AppViewOrg.ToolGroup.close('force',true);
                    error(message('nav:navslamapp:app:ScansAndPosesNumMismatch'));
                end
                
                
                obj.MapBuilderModel.loadProcessedData(scans, poses);
                obj.StateMachineModel.State = States.LoadingFromWS;
                obj.StateMachineModel.toSensorDataReady();
            else
                if nargin > 0
                    warning(message('nav:navslamapp:app:UnrecognizedInput'));
                end
            end

            % the life cycle of SLAMMapBuilder object is tied to its
            % ToolGroup object
            obj.AppViewOrg.ToolGroup.CanCloseFcn = @(source) closeAppCallback(obj);

            obj.AppViewOrg.thaw();
        end

        function close = closeAppCallback(obj)
        %closeCallback
            import nav.slamapp.internal.*

            close = false;
            if obj.MapBuilderModel.IsAppDirty && ~isempty(obj.AppViewOrg.ToolGroup.Title)

                dlgs = Dialogs(obj.AppViewOrg.ToolGroup);
                if obj.StateMachineModel.State == nav.slamapp.internal.States.LoadingBag || ...
                        obj.StateMachineModel.State == nav.slamapp.internal.States.LoadingFromWS || ...
                        obj.StateMachineModel.State == nav.slamapp.internal.States.ModifyingIncremental || ...
                        obj.StateMachineModel.State == nav.slamapp.internal.States.ModifyingLoopClosure

                    [selection, possibleSelections] = dlgs.questionDialog('SaveSessionInNonMainTabs', obj.AppViewOrg.MapBuilderTab.retrieveMsg('CloseAppHint'));
                    possibleSelections = ['-', possibleSelections];
                else
                    hasThirdChoice = true;
                    [selection, possibleSelections] = dlgs.questionDialog('SaveSession', obj.AppViewOrg.MapBuilderTab.retrieveMsg('CloseAppHint'), hasThirdChoice);
                end

                if obj.MapBuilderModel.IsRunning
                    obj.MapBuilderController.pauseMapBuilding;
                    pause(0.2);
                end

                switch selection
                  case possibleSelections{1}
                    isSuccess = obj.MapBuilderController.saveSessionWithErrorHandling;
                    if ~isSuccess
                        return;
                    end
                    close = true; % approve closing the App
                    delete(obj.AppViewOrg);
                    delete(obj);

                  case possibleSelections{2} % close app without saving current session
                    close = true; % approve closing the App
                    delete(obj.AppViewOrg);
                    delete(obj);
                end

            elseif (isempty(obj.AppViewOrg.ToolGroup.Title) || ~obj.MapBuilderModel.IsAppDirty)
                close = true; % close immediately if GroupTitle is set to empty 
                delete(obj.AppViewOrg);
                delete(obj);
            end
        end

        function delete(obj)
        %delete destructor
            if ~isempty(obj.AppViewOrg) && isvalid(obj.AppViewOrg)
                delete(obj.AppViewOrg);
            end
        end

    end
end
