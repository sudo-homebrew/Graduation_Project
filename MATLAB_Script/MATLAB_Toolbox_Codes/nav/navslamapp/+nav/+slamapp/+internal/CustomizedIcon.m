classdef CustomizedIcon < handle
%CUSTOMIZEDICON

% Copyright 2018 The MathWorks, Inc.

    methods (Static)
        function icon = IMPORT_FROM_FILE_24
            fileName = fullfile(getIconDir, 'ImportFromFile_24px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = IMPORT_FROM_FILE_16
            fileName = fullfile(getIconDir, 'ImportFromFile_16px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = IMPORT_FROM_WORKSPACE_16
            fileName = fullfile(getIconDir, 'ImportFromWorkspace_16px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = SYNC_MAP_24
            fileName = fullfile(getIconDir, 'SyncMap_24px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = SYNC_MAP_FAST_24
            fileName = fullfile(getIconDir, 'SyncMapFast_24px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = OCC_MAP_24
            fileName = fullfile(getIconDir, 'OccMap_24px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = AUTO_LOOPCLOSURE_24
            fileName = fullfile(getIconDir, 'Auto_LoopClosure_24px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = AUTO_INCREMENTAL_24
            fileName = fullfile(getIconDir, 'Auto_Incremental_24px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = MODIFY_LOOPCLOSURE_24
            fileName = fullfile(getIconDir, 'Modify_LoopClosure_24px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = MODIFY_INCREMENTAL_24
            fileName = fullfile(getIconDir, 'Modify_Incremental_24px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = IGNORE_SCANMATCH_16
            fileName = fullfile(getIconDir, 'Ignore_ScanMatch_16px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = LINEAR_24
            fileName = fullfile(getIconDir, 'Linear_24px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = ANGULAR_24
            fileName = fullfile(getIconDir, 'Angular_24px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = SNAP_24
            fileName = fullfile(getIconDir, 'Snap_24px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = LAYOUT_THREE_24
            fileName = fullfile(getIconDir, 'LayoutThree_24px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end

        function icon = GENERATE_MATLAB_SCRIPT_16
            fileName = fullfile(getIconDir, 'Generate_MATLABScript_16px.png');
            icon = matlab.ui.internal.toolstrip.Icon(fileName);
        end
    end
end

function iconDir = getIconDir()
%getIconDir Local function
    iconDir = fullfile(matlabroot, 'toolbox', 'nav', 'navslamapp', 'icons');
end
