classdef EntitySelectionMask
%This class is for internal use only. It may be removed in the future.

%EntitySelectionMask - Mask callbacks for select entity block
%   This block selects entities from Gazebo simulation world, including
%   model joints and model links

%   Copyright 2019-2020 The MathWorks, Inc.


    methods

        function modelEdit(obj, block)
        %modelEdit callback when model edit mask parameter is changed

            sourceBlockModel = [block '/' obj.ModelSourceBlockName];
            sourceBlockJointLink = [block '/' obj.JointLinkSourceBlockName];
            curValue = get_param(block, 'modelElemName');

            entityName = split(curValue,"/");
            modelName = entityName{1};
            set_param(sourceBlockModel, 'String', ['"' modelName '"']);

            if numel(entityName) == 2
                jointLinkName = entityName{2};
            else
                jointLinkName = char(join(entityName(2:end),'/'));
            end

            % remove <JOINT0> or <LINK1> tag if present as suffix
            if(contains(jointLinkName,'<') && contains(jointLinkName,'>'))
                jointLinkName = eraseBetween(jointLinkName,'<','>');
                jointLinkName = erase(jointLinkName,'<');
                jointLinkName = erase(jointLinkName,'>');
            end

            set_param(sourceBlockJointLink, 'String', ['"' jointLinkName '"']);

        end

        function modelElemSelect(obj, block)
        %modelElemSelect callback when a model element is selected.
        %This opens the Gazebo model list dialog.

            currentModelElem = get_param(block,'modelElemName');
            msgDlg = obj.ModelElemSelector;
            msgDlg.openDialog(currentModelElem,@dialogCloseCallback);

            function dialogCloseCallback(isAcceptedSelection, selectedModelElem)
                if isAcceptedSelection
                    set_param(block, 'modelElemName', selectedModelElem);
                end
            end
        end

        function configureNetworkAddrDlg(~, ~)
        %configureNetworkAddrDlg Establishes connection to the Gazebo
        %simulator
            dlg = robotics.slgazebo.internal.dlg.GazeboPreferenceSpecifier;
            dlg.openDialog;
        end

    end

end
