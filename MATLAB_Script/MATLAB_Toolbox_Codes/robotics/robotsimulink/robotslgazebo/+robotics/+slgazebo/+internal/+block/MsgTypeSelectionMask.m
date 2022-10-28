classdef (Abstract) MsgTypeSelectionMask
%This class is for internal use only. It may be removed in the future.

%MsgTypeSelectionMask - Message mask callbacks for Apply Command block

%   Copyright 2019-2021 The MathWorks, Inc.

    properties
        %MessageTypeList Message and command type list
        MessageTypeList

        %MessageTypeSelector Select the message and command type
        MessageTypeSelector

        FixedCommandList = {'ApplyLinkWrench',...
                            'ApplyJointTorque',...
                            'SetLinkWorldPose',...
                            'SetLinkLinearVelocity',...
                            'SetLinkAngularVelocity',...
                            'SetJointPosition',...
                            'SetJointVelocity'};

        MessageGetterType = 'applyCommand';
    end


    methods
        function obj = MsgTypeSelectionMask()
            obj.MessageTypeList = robotics.slgazebo.internal.util.GetBlankCommandList;
            obj.MessageTypeSelector = robotics.slgazebo.internal.dlg.MessageTypeSelector(obj.MessageGetterType,obj.MessageTypeList);
        end

        function editMsgType(obj, block, typeVarName, varargin)

            obj.BusUtil.BusNamePrefix = varargin{1};%busNamePrefix;
            obj.BusUtil.MsgPackageName = varargin{2};%msgPackageName;

            maskValues = get_param(block, typeVarName);

            if any(strcmp(maskValues, obj.MessageTypeList.MsgList))
                % no need to add 'gazebo_msgs/' for custom message

                if any(strcmp(maskValues, obj.FixedCommandList))
                    msgType = ['gazebo_msgs/', maskValues];
                    obj.BusUtil.createBusIfNeeded(msgType, bdroot(block));
                    obj.updateSubsystem(msgType,block);
                else
                    obj.BusUtil.createBusIfNeeded(maskValues, bdroot(block));
                    obj.updateSubsystem(maskValues,block);
                end
            else
                error(message('robotics:robotslgazebo:messagetypeselector:MessageTypeNotFound', maskValues));
            end

        end

        function selectMsgType(obj, block, typeVarName)
            currentMsgType = get_param(block, typeVarName);
            obj.MessageTypeSelector.openDialog(currentMsgType, @dialogCloseCallback);

            function dialogCloseCallback(isAcceptedSelection, selectedMsg)
                if isAcceptedSelection
                    set_param(block, typeVarName, selectedMsg);
                end
            end
        end

        function configureNetworkAddrDlg(obj, block) %#ok<INUSD>
            dlg = robotics.slgazebo.internal.dlg.GazeboPreferenceSpecifier;
            dlg.openDialog;
        end

    end

end
