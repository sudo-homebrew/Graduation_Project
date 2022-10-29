classdef SetupConfigSetDialog < codertarget.sltoolstrip.SetupConfigSetDialog
% This function is for internal use only. It may be removed in the future.

    % SETUPCONFIGSETDIALOG implements the SetupConfigSetDialog specific to
    % SocApp.
    % This dialog displays relevant hardware boards to user when 
    % 'Robot Operating System (ROS)' App is activated.
    
    % Copyright 2019-2020 The MathWorks, Inc.
    
    methods(Access = public)
        function this = SetupConfigSetDialog(model, configSet)
            this = this@codertarget.sltoolstrip.SetupConfigSetDialog(model, configSet);
        end
        
        function msg = getMessageText(~)
            msg = DAStudio.message('ros:slros:toolstrip:SetupDialogDescription');
        end

        function ret = getAppTitle(~)
            ret = DAStudio.message('ros:slros:toolstrip:SetupDialogTitle');
        end

        function supportedBoards = getSupportedBoards(~)
            %GETSUPPORTEDBOARDS Return a list of supported hardware boards for ROS App
            supportedBoards = ros.sltoolstrip.internal.getSupportedHardwareBoards();
        end
        
        function defaultItem = getDefaultItem(~)
            defaultItem = DAStudio.message('ros:slros:toolstrip:SetupDialogDefaultSelection');
        end
        
        function dlgPrompt = getDialogPrompt(~)
            dlgPrompt = DAStudio.message('ros:slros:toolstrip:SetupDialogSelectionPrompt');
        end
        
        function okAction(this, dlg)
           % apply hardware board selection and update configset
           set_param(this.ConfigSetCopy, 'HardwareBoard', dlg.getComboBoxText('hardwareBoardSelection'));
           okAction@coder.internal.toolstrip.SetupConfigSetDialog(this, dlg);
        end
    end
end
