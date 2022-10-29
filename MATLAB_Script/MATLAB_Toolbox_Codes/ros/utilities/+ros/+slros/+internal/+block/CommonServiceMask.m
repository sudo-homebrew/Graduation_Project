classdef (Abstract) CommonServiceMask < ros.slros.internal.block.CommonMask
%This class is for internal use only. It may be removed in the future.

%CommonServiceMask Common base class for ROS service block masks

%   Copyright 2018-2021 The MathWorks, Inc.

    methods
        %% ModelCloseFcn
        function modelCloseFcn(~, block)
        %modelCloseFcn Called when the model is closed
        %   Delete all message buses for this particular model.

            ros.slros.internal.bus.clearBusesOnModelClose(block);
        end
    end

    methods
        %% Block InitFcns

        % Callbacks for individual params are invoked when:
        % * User opens a mask dialog
        % * User modifies a param value and changes focus
        % * User modifies a param value and selects OK or Apply
        % * The model is updated (user presses Ctrl+D or simulates the model)

        function constantBlkInitFcn(obj, constantBlk)
        %constantBlkInitFcn Initialize the constant block in subsystem
        %   This is called by the InitFcn callback by the
        %   "Call Service/Constant" block

            block = get_param(constantBlk, 'parent');
            obj.createServiceBuses(block);
        end

        function sysobjInitFcn(obj, sysobjBlock) %#ok<INUSL>
        %sysobjInitFcn Initialize the system object in subsystem
        %   This is called by the InitFcn callback by the
        %   "Call Service/ServiceCaller" block

            rosSrvType = get_param(sysobjBlock, 'ServiceType');

            [expectedInputBusName, expectedOutputBusName] = ...
                ros.slros.internal.bus.Util.rosServiceTypeToBusNames(rosSrvType, bdroot(sysobjBlock));

            currentInputBusName = get_param(sysobjBlock, 'SLInputBusName');
            currentOutputBusName = get_param(sysobjBlock, 'SLOutputBusName');

            if ~strcmp(currentInputBusName, expectedInputBusName)
                % Do not mark the model dirty when the Simulink bus name
                % is updated on the underlying mask of Simulink ROS I/O
                % blocks.
                % All changes to model is ignored when this
                % "preserveDirty" variable is in Scope, hence this only
                % encompasses the set_param call in this if-else section.
                preserveDirty = Simulink.PreserveDirtyFlag(bdroot(sysobjBlock),'blockDiagram');  %#ok<NASGU>
                set_param(sysobjBlock, 'SLInputBusName', expectedInputBusName);
            end
            if ~strcmp(currentOutputBusName, expectedOutputBusName)
                preserveDirty = Simulink.PreserveDirtyFlag(bdroot(sysobjBlock),'blockDiagram');  %#ok<NASGU>
                set_param(sysobjBlock, 'SLOutputBusName', expectedOutputBusName);
            end
        end

    end

    %%
    methods
        %% Callbacks
        % Callbacks for individual params are invoked when the user:
        % * Opens a mask dialog
        % * Modifies a param value and changes focus
        % * Modifies a param value and selects OK or Apply
        % * Updates the model (presses Ctrl+D or simulates the model)
        %
        % Note - these are **not** invoked when user does a SET_PARAM
        function serviceSourceSelect(obj, block)
        %serviceSourceSelect Source of the service name has changed

            maskValues = get_param(block, 'MaskValues');
            maskVisibilities = get_param(block, 'MaskVisibilities');
            maskEnables = get_param(gcb,'MaskEnables');

            mask = Simulink.Mask.get(block);
            dlg = mask.getDialogControls;

            d = obj.MaskDlgIndex.ServiceSelect;
            m = obj.MaskDlgIndex.ServiceTypeSelect;

            if strcmpi(maskValues{obj.MaskParamIndex.SourceDropdown}, obj.TopicSourceSpecifyOwn)
                % Custom topic
                % Enable editing of topic
                maskEnables{obj.MaskParamIndex.ServiceNameEdit} = 'on';
                % Hide Topic selection button
                dlg(d(1)).DialogControls(d(2)).Visible = 'off';
                % Show MessageType selection button
                dlg(m(1)).DialogControls(m(2)).Visible = 'on';
            else % select topic from ROS network
                 % Disable editing of topic
                maskEnables{obj.MaskParamIndex.ServiceNameEdit} = 'off';
                % Show Topic selection button
                dlg(d(1)).DialogControls(d(2)).Visible = 'on';
                % Hide MessageType selection button
                dlg(m(1)).DialogControls(m(2)).Visible = 'off';
            end

            set_param(gcb,'MaskEnables', maskEnables);
            set_param(gcb,'MaskVisibilities', maskVisibilities);
        end

        % When the user updates the service type and/or service name, updateSubsystem
        % has be called in a callback context (since it is modifying the block).
        %
        % However, serviceSelect() and serviceTypeSelect() cannot call
        % updateSubsystem as the dialog is still open when the function is
        % done (the user hasn't made the selection yet). So serviceSelect and
        % serviceTypeSelect will return without applying any changes.
        % Note - we can't pass around a handle to updateSubsystem  as it
        % can only be called from within the callback context.
        %
        % Solution:
        %   Ensure either serviceEdit() or serviceTypeEdit() callbacks are
        %   configured. Once the user applies the change by clicking on OK/Apply
        %   in the mask dialog, the maskInitialize() will be invoked, and
        %   also the callbacks will be called on the next model update, so these
        %   will invoke updateSubsystem().
        %
        % The above solution also ensures that the service block
        % subsystems will be updated correctly if the user modifies the
        % 'service' or 'serviceType' parameters using SET_PARAM (since
        % the mask callbacks will be invoked during model update)

        function serviceEdit(obj, block)
        %serviceEdit Callback when service name changes

            sysobj_block = [block '/' obj.SysObjBlockName];
            curValue = get_param(sysobj_block, 'ServiceName');
            newValue = get_param(block, 'service');

            % Check for validity and make sure that the name is a valid ROS
            % name.
            if ~ros.internal.Namespace.isValidGraphName(newValue)
                set_param(block, 'service', curValue);
                error(message('ros:slros:svccaller:InvalidServiceName', newValue));
            end

            obj.createServiceBuses(block);
            obj.updateSubsystem(block);
        end

        function serviceSelect(~, block, getDlgFcn)
        %serviceSelect Callback for "Select" button on service name

            try
                svcDlg = feval(getDlgFcn);
                svcDlg.openDialog(@dialogCloseCallback);
            catch ME
                % Send error to Simulink diagnostic viewer rather than a
                % DDG dialog.
                % NOTE: This does NOT stop execution.
                reportAsError(MSLDiagnostic(ME));
            end

            function dialogCloseCallback(isAcceptedSelection, selectedSvc, selectedSvcType)
                if isAcceptedSelection
                    set_param(block, 'service', selectedSvc);
                    set_param(block, 'serviceType', selectedSvcType);
                end
            end
        end

        function serviceTypeSelect(~, block, getDlgFcn)
        %serviceTypeSelect Callback for "Select" button on service type

            currentServiceType = get_param(block, 'serviceType');

            svcDlg = feval(getDlgFcn);
            svcDlg.openDialog(currentServiceType, @dialogCloseCallback);

            function dialogCloseCallback(isAcceptedSelection, selectedSvc)
                if isAcceptedSelection
                    set_param(block, 'serviceType', selectedSvc);
                end
            end
        end
    end

    methods (Static, Access = ?matlab.unittest.TestCase)
        function [serviceNames, serviceTypes] = rosServiceNamesTypes(master)
        %rosServiceNamesTypes Retrieve service names and types from ROS network
        %   MASTER is a ros.slros.internal.sim.ROSMaster object

            [serviceNames, serviceTypes] = master.getServiceNamesTypes;

            if isempty(serviceNames)
                error(message('ros:slros:svcselector:NoServicesAvailable', ...
                              master.MasterURI));
            end
        end
    end

    methods (Static, Access = private)
        function createServiceBuses(block)
        %createServiceBuses Create service-related buses for block
        %   Note that we need two different bus definitions, one for
        %   the service request and one for the service response.

            serviceType = get_param(block, 'serviceType');
            locCreateBusIfNeeded(block, strcat(serviceType,'Request'), bdroot(block));
            locCreateBusIfNeeded(block, strcat(serviceType,'Response'), bdroot(block));
        end
    end
end



function locCreateBusIfNeeded(block, rosMsgType, mdlName)

    hasLegacyStructParam = contains(get_param(block, 'MaskVariables'),'legacyBusStruct');
    if hasLegacyStructParam
        if isequal(get_param(block, 'legacyBusStruct'),'on')
            ros.slros.internal.bus.Util.createBusIfNeeded(rosMsgType, mdlName);
        else
            ros.slroscpp.internal.bus.Util.createBusIfNeeded(rosMsgType, mdlName);
        end
    else
        ros.slros.internal.bus.Util.createBusIfNeeded(rosMsgType, mdlName);
    end
end
