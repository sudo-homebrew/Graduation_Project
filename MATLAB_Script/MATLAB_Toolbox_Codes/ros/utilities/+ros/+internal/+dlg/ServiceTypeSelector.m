classdef (Abstract) ServiceTypeSelector < handle
%This class is for internal use only. It may be removed in the future.

%  ServiceTypeSelector opens a DDG dialog that lets the user select from a
%  list of ROS service types (shared between ROS 1 and ROS 2). Once the
%  user accepts the changes (or cancels the dialog), a callback is invoked
%  with the closure action and selected service type.
%
%  Sample use:
%   selector = ros.slros2.internal.dlg.ServiceTypeSelector;
%   % The first argument is the service type to select by default
%   selector.openDialog('example_interfaces/AddTwoInts', ...
%   @(isAccepted,serviceType) disp(serviceType));

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = protected)
        Title = message('ros:slros:svcselector:ServiceTypeDialogTitle').getString;
        ROSService = ''
        ServiceList = {}
        CloseFcnHandle = function_handle.empty
    end

    methods (Access = protected, Abstract)
        % Sub-classes must implement the following methods
        setServicelist(obj);
    end

    methods
        function obj = ServiceTypeSelector()
        % Nothing to do in constructor
        end

        function set.ServiceList(obj, val)
            validateattributes(val, {'cell'},{'nonempty'},'setServiceList','ServiceList',2);
            obj.ServiceList = val;
        end

        function dlg = openDialog(obj, initialSvcSelection, closeFcnHandle)
        % closeFcnHandle: handle to function that takes two arguments
        %   closeFcn(isAcceptedSelection, rosService)
        %       isAcceptedSelection: true if user clicked on 'ok', false if
        %         user clicked on 'cancel' or closed window
        %       rosService: last selected ROS service (string)

            assert(ischar(initialSvcSelection) || isempty(initialSvcSelection));
            validateattributes(closeFcnHandle, {'function_handle'},{'scalar'});
            obj.CloseFcnHandle = closeFcnHandle;
            setServicelist(obj);
            dlg = DAStudio.Dialog(obj);
            if isempty(initialSvcSelection)
                return;
            end

            % Find initial service selection, if any
            index = find(strcmpi(initialSvcSelection, obj.ServiceList));
            if ~isempty(index)
                dlg.setWidgetValue('rossvctypelist', index-1); %zero-based
                obj.ROSService = obj.ServiceList{index};
            else
                warning(message('ros:slros:svcselector:ServiceTypeNotFound', ...
                                initialSvcSelection));
            end
        end
    end

    methods (Hidden)
        function dlgCallback(obj, dlg, tag, value)
        %dlgCallback Called when user selects an item from the list
            obj.ROSService = obj.ServiceList{value+1}; % value is zero-based
            dlg.refresh;
        end

        function dlgClose(obj, closeaction)
        % dlgClose Called when user close the DDG dialog
        % closeaction is 'ok' if user clicked OK,
        %   'cancel' if user clicked cancel or closed window
            if ~isempty(obj.CloseFcnHandle)
                isAcceptedSelection = strcmpi(closeaction, 'ok');
                try
                    feval(obj.CloseFcnHandle, isAcceptedSelection, obj.ROSService);
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
            svclist.Tag = 'rossvctypelist';
            svclist.MultiSelect = false;
            svclist.ObjectMethod = 'dlgCallback'; % call method on UDD source object
            svclist.MethodArgs = {'%dialog', '%tag', '%value'}; % object handle is implicit first arg
            svclist.ArgDataTypes = {'handle', 'string', 'mxArray'}; % 'handle' is type of %dialog
            svclist.Value = 0;
            svclist.NameLocation = 2; % top left

            % Main dialog
            dlgstruct.DialogTitle = obj.Title;
            dlgstruct.HelpMethod = 'ros.slros.internal.helpview';
            dlgstruct.HelpArgs = {'rosServiceTypeSelectDlg'}; % doc id
            dlgstruct.CloseMethod = 'dlgClose';
            dlgstruct.CloseMethodArgs = {'%closeaction'};
            dlgstruct.CloseMethodArgsDT = {'string'};

            % Make this dialog modal wrt to other DDG dialogs
            % (i.e. doesn't block MATLAB command line)
            dlgstruct.Sticky = true;

            % Buttons to show on dialog (these are options to pass to DDG,
            % not the final strings, so there is no need to use message
            % catalog)
            dlgstruct.StandaloneButtonSet =  ...
                {'Ok', 'Cancel', 'Help'}; % also available: 'Revert', 'Apply'

            dlgstruct.Items = {svclist};
        end
    end
end
