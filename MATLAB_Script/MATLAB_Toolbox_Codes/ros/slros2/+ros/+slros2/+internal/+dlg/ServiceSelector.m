classdef ServiceSelector < ros.internal.dlg.ServiceSelector
%This class is for internal use only. It may be removed in the future.

%   Copyright 2021 The MathWorks, Inc.

    methods
        function obj = ServiceSelector()
            obj.Title = message('ros:slros2:blockmask:ServiceDialogTitle').getString;
        end
    end
    methods (Access = protected)
        function setServiceListAndTypes(obj)

            [obj.ServiceList, obj.ServiceTypeList] = ros.ros2.internal.NetworkIntrospection.getServiceNamesTypes();
            if numel(obj.ServiceList) < 1
                error(message('ros:slros2:blockmask:NoServicesAvailableError'));
            end
        end
    end
    methods (Hidden)
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
            dlgstruct.HelpArgs = {'ros2ServiceSelectDlg'}; % doc topic id
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
