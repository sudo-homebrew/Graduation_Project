classdef (Abstract) ServiceSelector < handle
%This class is for internal use only. It may be removed in the future.

%  ServiceSelector (abstract) opens a DDG dialog that lets the user select
%  from a list of available ROS services (shared between ROS 1 and ROS 2).
%  Once the user accepts the changes (or cancels the dialog), a callback is
%  invoked with the closure action and selected topic type.

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = protected)
        Title = message('ros:slros:svcselector:ServiceDialogTitle').getString;
        ROSService = ''
        ROSServiceType = ''
        ServiceList = {} % list of available topics
        ServiceTypeList = {} % service type corresponding to each service
        CloseFcnHandle = function_handle.empty
    end

    methods (Access = protected, Abstract)
        % Sub-classes must implement the following methods
        setServiceListAndTypes(obj);
    end

    methods
        function dlg = openDialog(obj, closeFcnHandle)
        % closeFcnHandle: handle to function that takes three arguments
        %   closeFcn(isAcceptedSelection, rosService, rosServiceType)
        %    isAccpectedSelection: true if user clicked on 'ok', false if
        %         user clicked on 'cancel' or closed window.
        %    rosService: last selected ROS service (string)
        %    rosServiceType: type of the last selected ROS service (string)

            validateattributes(closeFcnHandle,{'function_handle'},{'scalar'});
            obj.CloseFcnHandle = closeFcnHandle;
            setServiceListAndTypes(obj);
            dlg = DAStudio.Dialog(obj);
            dlg.setWidgetValue('rosservicelist', 0); % select first item in the list
            obj.ROSService = obj.ServiceList{1};
            obj.ROSServiceType = obj.ServiceTypeList{1};
        end
    end

    methods (Hidden)
        function dlgCallback(obj, dlg, tag, value)
        % dlgCallback Called when user selects an item from the list

            obj.ROSService = obj.ServiceList{value+1}; % value is zero-based
            obj.ROSServiceType = obj.ServiceTypeList{value+1};
            dlg.refresh;
        end

        function dlgClose(obj, closeaction)
        % dlgClose Called when user close the DDG dialog
        %   closeaction is 'ok' if user clicked OK,
        %       'cancel' if user clicked cancel or closed window

            if ~isempty(obj.CloseFcnHandle)
                isAcceptedSelection = strcmpi(closeaction, 'ok');
                try
                    feval(obj.CloseFcnHandle, isAcceptedSelection, obj.ROSService, obj.ROSServiceType);
                catch
                    % Absorb all errors. If they are propagated back to
                    % DDG, this causes MATLAB to crash (Can't convert to
                    % warnings are not as they are not displayed either).
                end
            end
        end

        function dlgstruct = getDialogSchema(obj)
            svclist.Name = '';
            svclist.Type = 'listbox';
            svclist.Entries = obj.ServiceList;
            svclist.Tag = 'rosservicelist';
            svclist.MultiSelect = false;
            svclist.ObjectMethod = 'dlgCallback'; % call method on UDD source object
            svclist.MethodArgs = {'%dialog', '%tag', '%value'}; % object handle is implicit first arg
            svclist.ArgDataTypes = {'handle', 'string', 'mxArray'}; % 'handle' is type of %dialog
            svclist.Value = 0;
            svclist.NameLocation = 2; % top left

            % Main dialog
            dlgstruct.DialogTitle = obj.Title;
            dlgstruct.HelpMethod = 'ros.slros.internal.helpview';
            dlgstruct.HelpArgs = {'rosServiceSelectDlg'}; % doc topic id
            dlgstruct.CloseMethod = 'dlgClose';
            dlgstruct.CloseMethodArgs = {'%closeaction'};
            dlgstruct.CloseMethodArgsDT = {'string'};

            % Make this dialog modal wrt to other DDG dialogs
            % (i.e. doesn't block MATLAB command line)
            dlgstruct.Sticky = true;

            % Buttons to show on dialog (these are options to pass to DDG,
            % not the final strings, so there is no need to use service
            % catalog)
            dlgstruct.StandaloneButtonSet = ...
                {'Ok', 'Cancel', 'Help'}; % also available: 'Revert', 'Apply'
            dlgstruct.Items = {svclist};
        end
    end
end
