classdef ServiceTypeSelector < ros.internal.dlg.ServiceTypeSelector
%This class is for internal use only. It may be removed in the future.

%   Copyright 2021 The MathWorks, Inc.

    methods
        function obj = ServiceTypeSelector()
            obj.Title = message('ros:slros2:blockmask:ServiceTypeDialogTitle').getString;
        end
    end

    methods (Access = protected)
        function setServicelist(obj)
            obj.ServiceList = ros.slros2.internal.block.MessageBlockMask.getServiceList;
            assert(numel(obj.ServiceList)>0, 'Expected non-empty service list');
        end
    end

    methods (Hidden)
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
            dlgstruct.HelpArgs = {'ros2ServiceTypeSelectDlg'}; % doc id
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
