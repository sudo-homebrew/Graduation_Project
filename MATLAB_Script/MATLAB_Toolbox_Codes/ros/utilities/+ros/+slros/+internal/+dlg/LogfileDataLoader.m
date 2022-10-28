classdef LogfileDataLoader < handle
    %This class is for internal use only. It may be removed in the future.

    %  LogfileDataLoader opens a DDG dialog that lets the user select the
    %  logfile from which to load playback data. Once the user accepts the
    %  changes, the information is saved the specified model workspace.
    %
    %  Sample use:
    %   dlg = ros.slros.internal.dlg.LogfileDataLoader.retrieveDialog(modelName);

    %   Copyright 2017-2021 The MathWorks, Inc.

    properties (Access=private)
        % FilterSpec
        FilterSpec = '*.bag';
        
        % EmptyDataObj
        EmptyDataObj = ros.BagSelection.empty();
    end
    properties (Access = protected)
        %Filename Full path to logfile input or selected by the user
        Filename = '';

        %StartOffset When to start the playback from, compared to logfile
        %   start time. Empty defaults to play back whole file
        StartOffset = '';

        %Duration How long to continue playback for in the logfile
        %   Empty defaults to play back whole file
        Duration = '';

        %ModelName Name of model in which block resides, or blank if using
        %   base workspace
        ModelName = '';
    end

    properties (Access = protected, Constant)
        %DialogTagPrefix Prefix for the dialog tag, for easy searching
        DialogTagPrefix = 'slros_logfiledataloader_';
    end

    methods (Static)

        %% Dialog management
        function dlg = retrieveDialog(modelName,varargin)
            %retrieveDialog Return the dialog for loading data to the model
            %   If the dialog exists, it will be returned, otherwise it will be
            %   opened
            %   modelName - this dialog will assign data to the specified
            %               model's workspace
            %   dlg - DAStudio.Dialog object

            dlg = findDDGByTag(strcat(ros.slros.internal.dlg.LogfileDataLoader.DialogTagPrefix, modelName));
            if ~isempty(dlg)
                dlg = dlg(1);
                show(dlg)
            else
                obj = ros.slros.internal.dlg.LogfileDataLoader(modelName,varargin{:});            
                dlg = openDialog(obj);
            end
        end

        function closeDialog(modelName)
            %closeDialog Close the dialog associated with the model if open
            %   If the dialog exists, it will be closed
            %   modelName - model associated with the dialog

            dlg = findDDGByTag(strcat(ros.slros.internal.dlg.LogfileDataLoader.DialogTagPrefix, modelName));
            if ~isempty(dlg)
                delete(dlg)
            end
        end

    end

    methods

        %% Constructor and dialog setup
        function obj = LogfileDataLoader(modelName,varargin)
            %LogfileDataLoader Construct a dialog for loading data from a logfile
            %   modelName - this dialog will assign data to the specified
            %               model's workspace

            % Default to using base workspace model provided
            if nargin < 1
                modelName = '';
            end
            if nargin > 1
                obj.FilterSpec = validatestring(varargin{1},{'*.db3','*.bag'});
            end
            if nargin > 2
                validateattributes(varargin{2},{'ros.BagSelection','ros2bag'},{},'retreiveDialog');
                obj.EmptyDataObj = varargin{2};
            end
            % Save model name for later
            obj.ModelName = modelName;

            % See if there is currently any logfile specified in model
            % workspace and pre-load with that
            [fname, ~, startOff, dur] = ...
                ros.slros.internal.block.ReadDataBlockMask.getAllDataFromWS(obj.ModelName, obj.EmptyDataObj);
            if ischar(fname) && isvector(fname)
                obj.Filename = fname;
            end
            if ~isempty(startOff) && isnumeric(startOff) && ...
                    isscalar(startOff) && ~isnan(startOff)
                obj.StartOffset = sprintf('%d', startOff);
            end
            if ~isempty(dur) && isnumeric(dur) && ...
                    isscalar(dur) && ~isnan(dur) && dur >= 0
                obj.Duration = sprintf('%d', dur);
            end
        end

        function dlg = openDialog(obj)
            %openDialog Create, lay out, and make visible the dialog window

            % Open dialog
            dlg = DAStudio.Dialog(obj);
        end

    end

    methods (Hidden)

        %% Control callbacks
        function filenameCallback(obj, value)
            obj.Filename = strtrim(value);
        end

        function browseCallback(obj, dlg)
            titleTxt = getString(message('ros:slros:readlog:BrowseTitle'));
            [filename, pathname] = ...
                uigetfile(obj.FilterSpec, titleTxt, obj.Filename);
            if ~isequal(filename, 0)
                filepath = [pathname, filename];
                obj.Filename = strtrim(fullfile(filepath));
                refresh(dlg)
            end

        end

        function startOffsetCallback(obj, value)
            obj.StartOffset = strtrim(value);
        end

        function durationCallback(obj, value)
            obj.Duration = strtrim(value);
        end

        function [shouldClose, errorMsg] = dlgApplyClose(obj)
            %dlgApplyClose Callback for when the 'OK' button is pressed, but before the dialog closes

            % Check for valid values in entry fields, and set to
            % appropriate variables in the workspace
            try
                % Process logfile path
                absFilePath = robotics.internal.validation.findFilePath(obj.Filename);

                % Process logfile
                if endsWith(string(obj.Filename),'db3')
                    dataobj = ros2bag(fileparts(obj.Filename));
                else
                    dataobj = rosbag(obj.Filename);
                end

                % Process start offset
                if isempty(obj.StartOffset)
                    startOff = NaN;
                else
                    startOff = str2double(obj.StartOffset);
                    if isnan(startOff)
                        error(getString(message('ros:slros:readlog:InvalidStartOffset')))
                    end
                end

                % Process duration
                if isempty(obj.Duration)
                    dur = NaN;
                else
                    dur = str2double(obj.Duration);
                    if isnan(dur) || dur < 0
                        error(getString(message('ros:slros:readlog:InvalidDuration')))
                    end
                end

                % Set values in workspace
                ros.slros.internal.block.ReadDataBlockMask.setAllDataInWS(...
                    obj.ModelName, absFilePath, dataobj, startOff, dur)

                % Allow dialog to close
                shouldClose = true;
                errorMsg = '';
            catch ME
                shouldClose = false;
                errorMsg = ME.message;
            end
        end

        %% Dialog layout
        function dlgstruct = getDialogSchema(obj)
            %getDialogSchema Determine the dialog window layout

            % Instructions
            desc.Type = 'text';
            desc.Tag = 'Description';
            desc.WordWrap = true;
            desc.Name = getString(message('ros:slros:readlog:LogfileDescText'));

            % Grouping
            groupBoxDesc.Type       = 'group';
            groupBoxDesc.Items      = {desc};
            groupBoxDesc.RowSpan    = [1 1];
            groupBoxDesc.ColSpan    = [1 1];
            groupBoxDesc.LayoutGrid = [1 1];

            % Filename
            filename.Name = getString(message('ros:slros:readlog:LogfilePathPrompt'));
            filename.Type = 'edit';
            filename.Tag = 'Filename';
            filename.Mode = true;
            filename.DialogRefresh = true;
            filename.Value = obj.Filename;
            filename.ObjectMethod = 'filenameCallback'; % call method on UDD source object
            filename.MethodArgs   = {'%value'}; % object handle is implicit first arg
            filename.ArgDataTypes = {'string'}; % 'handle' is type of %dialog
            filename.RowSpan = [1 1];
            filename.ColSpan = [1 3];

            % Browse button
            browse.Name = getString(message('ros:slros:readlog:BrowsePrompt'));
            browse.Type = 'pushbutton';
            browse.Tag = 'Browse';
            browse.ObjectMethod = 'browseCallback'; % call method on UDD source object
            browse.MethodArgs   = {'%dialog'}; % object handle is implicit first arg
            browse.ArgDataTypes = {'handle'}; % 'handle' is type of %dialog
            browse.RowSpan = [1 1];
            browse.ColSpan = [4 4];

            % Start time offset
            startoffset.Name = getString(message('ros:slros:readlog:StartOffsetPrompt'));
            startoffset.Type = 'edit';
            startoffset.Tag = 'StartOffset';
            startoffset.Mode = true;
            startoffset.DialogRefresh = true;
            startoffset.Value = obj.StartOffset;
            startoffset.ObjectMethod = 'startOffsetCallback'; % call method on UDD source object
            startoffset.MethodArgs   = {'%value'}; % object handle is implicit first arg
            startoffset.ArgDataTypes = {'string'}; % 'handle' is type of %dialog
            startoffset.RowSpan = [2 2];
            startoffset.ColSpan = [1 4];

            % Duration
            duration.Name = getString(message('ros:slros:readlog:DurationPrompt'));
            duration.Type = 'edit';
            duration.Tag = 'Duration';
            duration.Mode = true;
            duration.DialogRefresh = true;
            duration.Value = obj.Duration;
            duration.ObjectMethod = 'durationCallback'; % call method on UDD source object
            duration.MethodArgs   = {'%value'}; % object handle is implicit first arg
            duration.ArgDataTypes = {'string'}; % 'handle' is type of %dialog
            duration.RowSpan = [3 3];
            duration.ColSpan = [1 4];

            % Grouping
            groupBoxParams.Name       = getString(message('ros:slros:readlog:LogfileOptionsTitle'));
            groupBoxParams.Type       = 'group';
            groupBoxParams.Items      = {filename, browse, startoffset, duration};
            groupBoxParams.RowSpan    = [1 1];
            groupBoxParams.ColSpan    = [1 1];
            groupBoxParams.LayoutGrid = [3 4];

            % Main dialog
            dlgstruct.DialogTitle = getString(message('ros:slros:readlog:LoadLogfileTitle'));
            dlgstruct.HelpMethod = 'ros.slros.internal.helpview';
            dlgstruct.HelpArgs =  {'rosLogfileDataLoaderDlg'};  % doc topic id
            dlgstruct.PreApplyMethod = 'dlgApplyClose';

            % Do not make this dialog modal wrt to other DDG dialogs
            dlgstruct.Sticky = false;

            % Buttons to show on dialog (these are options to pass to DDG,
            % not the final strings, so there is no need to use message
            % catalog)
            dlgstruct.StandaloneButtonSet =  ...
                {'Ok', 'Cancel', 'Help'}; % also available: 'Revert', 'Apply'

            dlgstruct.Items = {groupBoxDesc, groupBoxParams};
            dlgstruct.DialogTag = strcat(obj.DialogTagPrefix, obj.ModelName);
        end

    end

end
