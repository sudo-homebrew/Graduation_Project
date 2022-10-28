classdef RBTAlgorithmBlockMasks < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.
    
    %   Copyright 2017-2021 The MathWorks, Inc.
    
    %% Methods shared among all manipulator blocks  
    
    methods (Static)
        function [treeStruct, numBodies, sysName] = updateMaskWorkSpaceVariables(block, rigidBodyTree)
            %updateMaskWorkSpaceVariables update the variables in the mask workspace.
            %   When the mask is initialized, and when the tree is updated,
            %   it is necessary to update the tree structure and number of
            %   bodies that is passed to the MATLAB Function under the
            %   mask. This function also ensures that the mask is correctly
            %   updated if invalid inputs are sent (those inputs are
            %   rejected, the user sees an error, and any dependent fields
            %   are rendered empty).
            
            %If in a library, return empty variables (so nothing is
            %displayed on the block mask) and do not evaluate tree.
            sys = bdroot(block);
            if bdIsLibrary(sys)
                treeStruct = []; numBodies = 0; sysName = '';
                return
            end
            
            sysName = get_param(block, 'RigidBodyTree');
            
            %Validation: Robot must be a RigidBodyTree
            validateattributes(rigidBodyTree,{'rigidBodyTree'},{'nonempty','scalar'},'updateMaskWorkSpaceVariables','rigidBodyTree');
            
            % Skip the collision data since including it affects
            % performance and none of the current Simulink blocks 
            % make use of it
            skipCollisionData = true;
            treeStruct = robotics.manip.internal.RBTRepresentationUtility.populateRigidBodyTreeStruct(rigidBodyTree, skipCollisionData);
            
            %set the data format in the struct to 'column'
            %All Simulink blocks require RigidBodyTree dataformat to be
            %'column'
            treeStruct.DataFormat = robotics.manip.internal.RigidBodyTree.DATA_FORMAT_COLUMN;
            
            numBodies = treeStruct.NumBodies;
        end
        
        function validateRigidBodyTree(block, treeStruct)
            %validateRigidBodyTree Ensure rigid body tree passes basic checks
            
            %Do nothing if called in a library
            sys = bdroot(block);
            if bdIsLibrary(sys)
                return
            end
            
            %Ensure that robot is compatible with blocks: nonzero bodies.
            %Since the attribute is obscured, used a custom error message.
            try
                validateattributes(treeStruct.NumBodies,{'numeric'},{'nonempty','scalar','nonzero'},'updateMaskWorkSpaceVariables','RigidBodyTree.NumBodies');
            catch
                error(message('robotics:robotslmanip:blockmask:NonzeroNumBodies'));
            end
            
            %Ensure that robot is compatible with blocks: nonzero joints
            %Since the attribute is obscured and is actually only visible
            %on the internal RBT (or on the structure), used a custom error
            %message.
            try
                validateattributes(treeStruct.PositionNumber,{'numeric'},{'nonempty','scalar','nonzero'},'updateMaskWorkSpaceVariables','RigidBodyTree.VelocityNumber');
            catch
                error(message('robotics:robotslmanip:blockmask:NoMovingJoints'));
            end
            
        end
        
        function updateBlockInputDimensions(block, treeStruct, varargin)
            %updateBlockInputDimensions Update inport dimensions
            %   This callback changes the dimensions of the block inports
            %   to values that match the associated rigid body tree. This
            %   approach ensures that size mismatches are thrown during the
            %   model update stage (before compilation), and that the model
            %   is able to update, build, and run, even when ports are left
            %   unconnected.
            
            %Do nothing if called in a library
            sys = bdroot(block);
            if bdIsLibrary(sys)
                return
            end
            
            %Validate Rigid body tree
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateRigidBodyTree(block, treeStruct);
            
            %Assign dimensions to input ports
            for i = 1:nargin-2
                inportName = [block '/' varargin{i}];
                switch varargin{i}
                    case 'Config'
                        set_param(inportName, 'PortDimensions', num2str(treeStruct.PositionNumber));
                    case 'JointVel'
                        set_param(inportName, 'PortDimensions', num2str(treeStruct.VelocityNumber));
                    case 'JointAccel'
                        set_param(inportName, 'PortDimensions', num2str(treeStruct.VelocityNumber));
                    case 'JointTorq'
                        set_param(inportName, 'PortDimensions', num2str(treeStruct.VelocityNumber));
                    case 'FExt'
                        set_param(inportName, 'PortDimensions', ['[ 6 ' num2str(treeStruct.NumBodies) ']']);
                    case 'Pose'
                        set_param(inportName, 'PortDimensions', '[ 4 4]');
                    case 'Weights'
                        set_param(inportName, 'PortDimensions', '[ 1 6]');
                    case 'InitialGuess'
                        set_param(inportName, 'PortDimensions', num2str(treeStruct.PositionNumber));                        
                end
            end
            
        end
    end
    
    %% Methods for updating body dialogs
    
    methods(Static)        
        function updateMaskBodyList(block, parameterName)
            %updateMaskBodyList Update the list of available RBT Bodies
            %   When the algorithm block dialog is opened, for dialogs that
            %   allow users to specify bodies (e.g. Get Jacobian and Get
            %   Transform blocks), this function is used to populate the
            %   list of source and target bodies. The parameters to update
            %   are passed as character array inputs to this callback.
            
            %Do nothing if called in a library
            sys = bdroot(block);
            if bdIsLibrary(sys)
                return
            end
            
            %Do nothing if the model is running
            if robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.checkIfRunning(sys)
                return
            end
            
            %Need to load the tree, which may be in several locations.
            %Presently does not include Data Dictionary, which does not
            %support MCOS (as per g993535)
            
            %Use slResolve to convert the RigidBodyTree parameter to an
            %actual value that can be used in the mask. slResolve evaluates
            %a SL starting at the stated level (here: block level).
            maskValue = get_param(block, 'RigidBodyTree');
            tree = slResolve(maskValue, block);
            if isempty(tree)
                %If slresolve fails to resolve, it will return empty
                error(message('robotics:robotslmanip:blockmask:UnresolvedRigidBodyTreeParameter', maskValue));
            end
            
            %Get the currently selected value
            currentBody = get_param(block, parameterName);
            
            %Create a dialog object and populate it
            bodyDlg = robotics.slmanip.internal.dlg.BodySelector();
            bodyDlg.openDialog(currentBody, tree, @dialogCloseCallback);
            
            %Dialog close callback: set associated parameter value
            function dialogCloseCallback(isAcceptedSelection, selectedBody)
                if isAcceptedSelection
                    set_param(block, parameterName, selectedBody);
                end
            end
        end
        
        function bodyWSVar = updateBodyWSVar(block, rigidBodyTree, parameterName)
            %bodyWSVar Update RBT body workspace variables and set default body names
            %   This function verifies that the body in the mask exists in
            %   the associated Rigid Body Tree model and updates the
            %   corresponding value in the mask workspace, a numeric array.
            %   If the values do not exist, the function changes the bodies
            %   in the mask. For end effectors, the default value is the
            %   bodyname with the highest index. For all other bodies, the
            %   default is the name of the base of the associated Rigid
            %   Body Tree model. In either case, the user is notified by a
            %   warning (which has the same message ID for either case).
            
            bodyName = get_param(block, parameterName);
            
            %Check if the body exists in the evaluated Rigid Body Tree
            if bodyInTree(rigidBodyTree, bodyName)
                bodyWSVar = double(bodyName);
            else
                %To get here in mask initialization, the rigidBodyTree must
                %exist. However, it is possible that the user has opened
                %the block dialog and entered a new rigidBodyTree that has
                %not yet been applied, even though the block mask has
                %updated (e.g., the DDG dialogs can update the blockmask).
                %To handle this case, the tree can be evaluated from the
                %value in the block.
                maskValue = get_param(block, 'RigidBodyTree');
                treeInMask = slResolve(maskValue, block);
                if isempty(treeInMask)
                    %If slresolve fails to resolve, it will return empty
                    error(message('robotics:robotslmanip:blockmask:UnresolvedRigidBodyTreeParameter', maskValue));
                end
                
                %Check if the body exists in the Rigid Body Tree in the mask
                if bodyInTree(treeInMask, bodyName)
                    bodyWSVar = double(bodyName);
                else
                    %If the body does not exist, set to a default value
                    if strcmp(parameterName, 'EndEffector')
                        %For end effectors, use the name of the body with
                        %the highest index (the last body in the body
                        %list). This is not always the end effector, but it
                        %is a leaf body.
                        defaultName = treeInMask.BodyNames{end};
                        defaultDescr = message('robotics:robotslmanip:blockmask:EndEffectorDefaultBodyName').getString;
                    else
                        %For other targets, use the base name.
                        defaultName = treeInMask.BaseName;
                        defaultDescr = message('robotics:robotslmanip:blockmask:DefaultBodyName').getString;
                    end
                    %Change the bodies in the mask to values that exist in this RBT
                    set_param(block, parameterName, defaultName);
                    bodyWSVar = double(defaultName);
                     reportAsWarning(MSLDiagnostic(message('robotics:robotslmanip:blockmask:UnresolvedRigidBody',bodyName,parameterName,defaultName,defaultDescr)));
                end
            end
            
            function bodyNameFound = bodyInTree(tree, bodyName)
                %bodyNameFound Check whether a body with name bodyName exists in tree
                
                treeBodyList = [tree.BodyNames tree.BaseName];
                bodyNameFound = ismember(bodyName, treeBodyList);
            end
            
        end
        
        function checkChainForNonFixedJoints(block, tree, eeParameterName)
            %checkChainForNonFixedJoints Ensure that at least one of the joints between base and end-effector is a non-fixed joint
            
            endEffectorName = get_param(block, eeParameterName);
            bodyIndices = kinematicPath(tree.TreeInternal, tree.BaseName, endEffectorName);
            
            for i = 1:length(bodyIndices)
                if ~strcmp(tree.Bodies{bodyIndices(i)+1}.Joint.Type, 'fixed')
                    return;
                end
            end
            
            %Error if only fixed joints are encountered
            error(message('robotics:robotslmanip:blockmask:NoMovingJointsChain', endEffectorName));
        end
        
    end
    
    %% IK Block
    
    methods (Static)
        function busName = createIKInfoBus(block)
            %%createIKInfoBus creates an IK solInfo bus defined in global workspace
            
            elems = Simulink.BusElement.empty;            
            elemNames = {'Iterations', 'PoseErrorNorm',...
                'ExitFlag', 'Status'};
            elemType = {'double', 'double', 'uint16', 'uint8'};
            
            for i = 1:length(elemNames)
                elems(i) = Simulink.BusElement;
                elems(i).Name = elemNames{i};
                elems(i).Dimensions = [1 1];
                elems(i).DimensionsMode = 'Fixed';
                elems(i).DataType = elemType{i};
                elems(i).Complexity = 'real';
                
                descMsgID = sprintf('robotics:robotslmanip:busdescriptions:IKInfoBus%sDescription', elemNames{i});
                elems(i).Description = message(descMsgID).getString;
            end
            
            busName = 'ikblock_info_bus';
            bus = Simulink.Bus;
            bus.Description = message('robotics:robotslmanip:busdescriptions:IKInfoBusDescription').getString;
            bus.Elements = elems;
            
            sys = bdroot(block);
            if ~bdIsLibrary(sys)
                robotics.slcore.internal.util.assigninGlobalScope(sys, busName, bus);
            end
        end
        
        function updateLMParameterVisibility(block)
            %updateLMParameterVisibility Update LM solver parameter dialog visibility
            
            %Do nothing if the model is running
            sys = bdroot(block);
            if robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.checkIfRunning(sys)
                return
            end
            
            p = Simulink.Mask.get(block);
            dampingBiasParam = p.getParameter('DampingBias');
            errorDampingParam = p.getParameter('UseErrorDamping');
            errorChangeTolParam = p.getParameter('ErrorChangeTolerance');
            
            solverName = get_param(block, 'SolverName');
            if strcmp(solverName, 'Levenberg-Marquardt')
                %Levenberg-Marquardt parameters are enabled
                errorDampingParam.Enabled = 'on';
                errorChangeTolParam.Enabled = 'on';
                
                %Damping bias is dependent on UseErrorDamping value
                dampingEnabled = strcmp(get_param(block,'UseErrorDamping'), 'on');
                if dampingEnabled
                    dampingBiasParam.Enabled = 'on';
                else
                    dampingBiasParam.Enabled = 'off';
                end
            else
                %Levenberg-Marquardt parameters are disabled
                dampingBiasParam.Enabled = 'off';
                errorDampingParam.Enabled = 'off';
                errorChangeTolParam.Enabled = 'off';
            end
        end              
        
        function solverParamStruct = updateSolverParameters(block, ...
                MaxIterations, MaxTime, GradientTolerance, SolutionTolerance, StepTolerance, ...
                ErrorChangeTolerance, DampingBias)
                                    
            solverParamStruct = struct;
            
            %Do nothing if called in a library
            sys = bdroot(block);
            if bdIsLibrary(sys)
                return
            end
            
            %Assign 1/0 for check-box on/off shared parameters
            solverParamStruct.EnforceJointLimits = strcmp(get_param(block,'EnforceJointLimits'),'on');
            solverParamStruct.AllowRandomRestart = false;
            
            %Validate edit-field params
            validateattributes(MaxIterations, {'numeric'}, {'nonempty', 'scalar', 'real', 'positive', 'nonnan', 'integer'}, 'updateSolverParameters', 'MaxIterations');
            validateattributes(MaxTime, {'numeric'}, {'nonempty', 'scalar', 'real', 'nonnan', 'positive'}, 'updateSolverParameters', 'MaxTime');
            validateattributes(GradientTolerance, {'numeric'}, {'nonempty', 'scalar', 'real', 'nonnan', 'nonnegative'}, 'updateSolverParameters', 'GradientTolerance');
            validateattributes(SolutionTolerance, {'numeric'}, {'nonempty', 'scalar', 'real', 'nonnan', 'nonnegative'}, 'updateSolverParameters', 'SolutionTolerance');
            validateattributes(StepTolerance, {'numeric'}, {'nonempty', 'scalar', 'real', 'nonnan', 'nonnegative'}, 'updateSolverParameters', 'StepTolerance');
            
            %Provide the shared numeric parameters
            solverParamStruct.MaxIterations = MaxIterations;
            solverParamStruct.MaxTime = MaxTime;
            solverParamStruct.GradientTolerance = GradientTolerance;
            solverParamStruct.SolutionTolerance = SolutionTolerance;
            solverParamStruct.StepTolerance = StepTolerance;
            
            %Provide LM Parameters if LM solver is selected
            if strcmp(get_param(block,'SolverName'),'Levenberg-Marquardt')
                validateattributes(ErrorChangeTolerance, {'numeric'}, {'nonempty', 'scalar', 'real', 'nonnan', 'nonnegative'}, 'updateSolverParameters', 'ErrorChangeTolerance');
                solverParamStruct.ErrorChangeTolerance = ErrorChangeTolerance;
                solverParamStruct.UseErrorDamping = strcmp(get_param(block,'UseErrorDamping'),'on');
                
                if solverParamStruct.UseErrorDamping
                    validateattributes(DampingBias, {'numeric'}, {'nonempty', 'scalar', 'real', 'nonnan', 'nonnegative'}, 'updateSolverParameters', 'DampingBias');
                    solverParamStruct.DampingBias = DampingBias;
                end
            end
            
        end
        
        function busName = updateInfoOutput(block, infoTF)
            %updateInfoOutput Update block configuration to show or disable Info port (outport 2)
            %   This function calls the helper functions that add or remove
            %   the internal outport and line required to enable or disable
            %   the top-level outport for this block (internally, showInfo
            %   triggers the system object to add or remove the outport).
            %   Additionally, this function ensures that the associated bus
            %   is only created when the Info output is enabled. Here,
            %   infoTF is a boolean indicating whether the Info port is
            %   enabled or disabled.
            
            blockObj = get_param(block, 'Object');
            infoOutExists = any(strcmp(blockObj.blocks, 'Info'));
            
            busName = 'ikblock_info_bus';
            
            if strcmp(infoTF,'on')
                %Only assign in global scope when ShowInfo is on
                busName = robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.createIKInfoBus(gcb);
                if ~infoOutExists
                    robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.addIKBlockInfoOutport(block);
                end
            elseif strcmp(infoTF,'off') && infoOutExists
                robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.deleteIKBlockOutport(block);
            end
                
            end
        
        function addIKBlockInfoOutport(block)
            %addIKBlockInfoOutport Add outport under mask for system object
            %   Since the system object adds a second outport whenever the
            %   info output is enabled, that outport must be connected to a
            %   top-level (user-facing) one. This function creates that
            %   outport.
            
            blkSys = getfullname(block);
            outportblk = [blkSys '/Info'];
            add_block('built-in/Outport',outportblk, 'Position', [315 88 345 102])
            add_line(blkSys, 'MATLAB System/2', 'Info/1');
            
        end
        
        function deleteIKBlockOutport(block)
            %deleteIKBlockOutport Add outport under mask for system object
            %   Since the system object removes the second outport whenever
            %   the info output is disabled, the corresponding user-facing
            %   outport and lines must also be removed. This function
            %   performs that action.
            
            blkSys = getfullname(block);
            outportblk = [blkSys '/Info'];
            lh = get_param(outportblk, 'LineHandles');
            delete_line(lh.Inport)
            delete_block(outportblk)
            
        end        
    end
    
    %% Joint Space Motion Control Model Block
    
    properties (Constant)
        JointSpaceMotionTypes = {
            message("robotics:robotslmanip:blockmask:JointSpaceMotionModelMotionTypeCompTorquePopup").getString, ...
            message("robotics:robotslmanip:blockmask:JointSpaceMotionModelMotionTypePDControlPopup").getString, ...
            message("robotics:robotslmanip:blockmask:JointSpaceMotionModelMotionTypeIndJointPopup").getString, ...
            message("robotics:robotslmanip:blockmask:JointSpaceMotionModelMotionTypeOpenLoopPopup").getString, ...
            }
        
        ErrorDynamicsFormats = {
            message("robotics:robotslmanip:blockmask:JointSpaceMotionModelSpecFormatDampingPopup").getString, ...
            message("robotics:robotslmanip:blockmask:JointSpaceMotionModelSpecFormatStepPopup").getString, ...
            }
    end
    
    methods (Static)        
        function updateJSMotionParameters(block)
            %updateJointSpaceControlParameters Update controller parameter dialog visibility
            
            %Do nothing if the model is running
            sys = bdroot(block);
            if robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.checkIfRunning(sys)
                return
            end
            
            % Initialize list of items with variable visibility
            p = Simulink.Mask.get(block);
            paramArray = {...
                p.getParameter('ProportionalGain') ...
                p.getParameter('DerivativeGain') ...
                p.getParameter('ShowExternalForcePort') ...
                p.getParameter('ErrorDynamicsSpecification') ...
                p.getParameter('SettlingTime') ...
                p.getParameter('Overshoot') ...
                p.getParameter('DampingRatio') ...
                p.getParameter('NaturalFrequency') ...
                };
            
            % Initialize parameter arrays
            isVisibleArray = false(size(paramArray));
            isEnabledArray = true(size(paramArray));
            
            %Turn on some visibility by control method
            motionTypes = robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.JointSpaceMotionTypes;
            controlMethod = get_param(block, 'MotionType');
            if strcmp(controlMethod, motionTypes{1}) %Computed Torque Control
                isVisibleArray(3:8) = true;
                isEnabledArray = robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.updateJSMotionModelErrorDynamicsDialog(block, isEnabledArray);
            elseif strcmp(controlMethod, motionTypes{2}) %PD Control
                isVisibleArray(1:3) = true;
            elseif strcmp(controlMethod, motionTypes{3}) %Independent Joint Motion
                isVisibleArray(4:8) = true;
                isEnabledArray = robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.updateJSMotionModelErrorDynamicsDialog(block, isEnabledArray);
            else
                %Open loop dynamics - only external force option
                isVisibleArray(3) = true;
            end
            
            %Update to incorporate the parameter input type & apply changes
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.updateJointSpaceParameterVisibility(paramArray, isVisibleArray);
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.updateJointSpaceParameterEnabledness(paramArray, isEnabledArray);
        end
        
        function isEnabledArray = updateJSMotionModelErrorDynamicsDialog(block, initArray)
            %updateControllerParameterVisibility Update controller parameter dialog visibility
            
            %Initialize outputs
            isEnabledArray = initArray;
            
            inputType = get_param(block, 'ErrorDynamicsSpecification');
            edFormats = robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.ErrorDynamicsFormats;
            switch inputType
                case edFormats{1} %Damping Ratio / Natural Frequency
                    isEnabledArray(5:6) = false;
                case edFormats{2} %Step Response
                    isEnabledArray(7:8) = false;
                otherwise
            end
        end
        
        function updateJointSpaceParameterVisibility(parameterCellArray, isVisibleArray)
            
            for i = 1:length(parameterCellArray)
                if isVisibleArray(i)
                    parameterCellArray{i}.Visible = 'on';
                else
                    parameterCellArray{i}.Visible = 'off';
                end
            end
        end
        
        function updateJointSpaceParameterEnabledness(parameterCellArray, isEnabledArray)
            
            for i = 1:length(parameterCellArray)
                if isEnabledArray(i)
                    parameterCellArray{i}.Enabled = 'on';
                else
                    parameterCellArray{i}.Enabled = 'off';
                end
            end
        end
        
        function updateJSMotionModelInports(block, motionType, hasFExt)
            
            possCmdInportNames = {'qRef', 'qRefDot', 'qRefDDot'};
            motionTypes = robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.JointSpaceMotionTypes;
            switch motionType
                case motionTypes{1} %Computed Torque Control
                    numUserFacingInports = 3;
                case motionTypes{2} %PD Control
                    numUserFacingInports = 2;
                case motionTypes{3} %Independent Joint Motion
                    numUserFacingInports = 3;
                    hasFExt = 'off';
                otherwise           %Open Loop Dynamics
                    numUserFacingInports = 0;
            end
            
            for i = 1:3
                if i <= numUserFacingInports
                    robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.setupBlockInput(block, possCmdInportNames{i}, 'Inport', 2+i);
                else
                    robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.setupBlockInput(block, possCmdInportNames{i}, 'Constant', 2+i);
                end
            end
            
            if strcmp(hasFExt, 'on')
                % Ensure inport is in place
                robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.setupBlockInput(block, 'FExt', 'Inport', 6);
                robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.updateInportNumber(block, 'FExt', numUserFacingInports+1);
            else
                robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.setupBlockInput(block, 'FExt', 'Constant', 6);
            end
        end
        
        function updateErrorParametersFromStep(block, settlingTime, overshoot)
            %updateErrorParametersFromStep Update second order behavior using step response parameters
            %   Update natural frequency and damping of a joint by
            %   providing the corresponding values of settling time
            %   (seconds) and overshoot (relative to a unit step).
            
            % Assume the system is critically damped unless it has been
            % set otherwise
            currentZeta = str2double(get_param(block, 'DampingRatio'));
            zeroOvershootZeta = max(1, currentZeta);
            
            % If either settlingTime or overshoot are provided as vectors,
            % expand the other to be vector-valued as well so that
            % dimensions match    
            [maxNumel, settlingTimeMatchedDim, overshootMatchedDim, zeroOvershootZetaMatchedDim] = robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.matchParameterDimensions(settlingTime, overshoot, zeroOvershootZeta);
            
            % Iterate through all values since omega and zeta are scalar
            % assignments
            zeta = zeros(1, maxNumel);
            omega = zeros(1, maxNumel);
            for i = 1:maxNumel
                [zeta(i), omega(i)] = robotics.manip.internal.SecondOrderSysHelper.getSysParamsFromStepParams(...
                    settlingTimeMatchedDim(i), overshootMatchedDim(i), zeroOvershootZetaMatchedDim(i));
            end
            
            % Update block
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.assignValueToBlock(block, 'NaturalFrequency', omega);
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.assignValueToBlock(block, 'DampingRatio', zeta);
        end
        
        
        function updateStepFromErrorParameters(block, zeta, omega)
            %updateStepFromErrorParameters Update step response parameters from damping & natural frequency
            
            % If either settlingTime or overshoot are provided as vectors,
            % expand the other to be vector-valued as well so that
            % dimensions match    
            [maxNumel, zetaMatchedDim, omegaMatchedDim] = robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.matchParameterDimensions(zeta, omega);
            
            % Iterate through all values since omega and zeta are scalar
            % assignments
            overshoot = zeros(1, maxNumel);
            settlingTime = zeros(1, maxNumel);
            for i = 1:maxNumel
                [overshoot(i), settlingTime(i)] = robotics.manip.internal.SecondOrderSysHelper.getStepParamsFromSysParams(...
                    zetaMatchedDim(i), omegaMatchedDim(i));
            end
            
            % Update block
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.assignValueToBlock(block, 'SettlingTime', settlingTime);
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.assignValueToBlock(block, 'Overshoot', overshoot);
        end
        
        function updateInterdependentErrorParameters(block, settlingTime, overshoot, damping, naturalFrequency)
           
            errorDynamicsInputFormat = get_param(block, 'ErrorDynamicsSpecification');
            if strcmp(errorDynamicsInputFormat, 'Step Response')
                robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.updateErrorParametersFromStep(block, settlingTime, overshoot);
            else
                robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.updateStepFromErrorParameters(block, damping, naturalFrequency);
            end
        end
        
        function validateJSMotionModelParameters(block, treeStruct, proportionalGain, derivativeGain, dampingRatio, naturalFrequency, settlingTime, overshoot, initConfig, initJointVel)
            %validateJSMotionModelParameters Validate joint-space motion model edit-field parameters
            %   This method is called from the block mask and occurs at
            %   set-time. If the mask is opened, the Apply or OK buttons
            %   must be clicked. If done via set_param, the callback will
            %   be triggered immediately. These blocks often call internal
            %   MATLAB methods that perform similar checks, but those
            %   checks will not occur until runtime.
            
            % Do nothing if called in a library
            sys = bdroot(block);
            if bdIsLibrary(sys)
                return
            end
            
            % The sizes are dependent on the number of non-fixed joints in
            % the associated rigid body tree
            numJts = treeStruct.VelocityNumber;
            
            % Validate edit fields that correspond to the selected motion
            % type in the associated pop-up.
            motionTypes = robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.JointSpaceMotionTypes;
            controlMethod = get_param(block, 'MotionType');
            if strcmp(controlMethod, motionTypes{1}) %Computed Torque Control
                robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateErrorDynamicsInputs(block, numJts, dampingRatio, naturalFrequency, settlingTime, overshoot);
            elseif strcmp(controlMethod, motionTypes{2}) %PD Control
                robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validatePDInputs(numJts, proportionalGain, derivativeGain);
            elseif strcmp(controlMethod, motionTypes{3}) %Independent Joint Motion
                robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateErrorDynamicsInputs(block, numJts, dampingRatio, naturalFrequency, settlingTime, overshoot);
            else
                %Open loop dynamics - only external force option
            end
            
            % Validate initial state
            validateattributes(initConfig, {'double'}, {'nonempty', 'vector', 'finite', 'real'}, 'validateJSMotionModelParameters', 'initConfig');
            validateattributes(initJointVel, {'double'}, {'nonempty', 'vector', 'finite', 'real'}, 'validateJSMotionModelParameters', 'initJointVel');
            
            % Validate sizes on inputs that can be scalar or a size that
            % depends on the number of joints in the associate rigid body
            % tree object
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateVariableJointDependentSize(initConfig, numJts, 'vector', 'initConfig');
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateVariableJointDependentSize(initJointVel, numJts, 'vector', 'initJointVel');
        end
        
        function validatePDInputs(numJts, proportionalGain, derivativeGain)
            %validatePDInputs Validate proportional and derivative gains for PD Control
            
            validateattributes(proportionalGain, {'double'}, {'nonempty', 'square', 'real', 'finite'}, 'validatePDInputs', 'proportionalGain');
            validateattributes(derivativeGain, {'double'}, {'nonempty', 'square', 'real', 'finite'}, 'validatePDInputs', 'derivativeGain');
            
            % Validate sizes on inputs that can be scalar or a size that
            % depends on the number of joints in the associate rigid body
            % tree object
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateVariableJointDependentSize(proportionalGain, numJts, 'matrix', 'proportionalGain');
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateVariableJointDependentSize(derivativeGain, numJts, 'matrix', 'derivativeGain');
        end
        
        function validateErrorDynamicsInputs(block, numJts, dampingRatio, naturalFrequency, settlingTime, overshoot)
            %validateErrorDynamicsInputs Validate the error dynamics
            %   This method validates the error dynamics parameters that
            %   correspond to the selected input type pop-up value. These
            %   are either natural frequency and damping or settling time
            %   and overshoot.
            
            inputType = get_param(block, 'ErrorDynamicsSpecification');
            edFormats = robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.ErrorDynamicsFormats;
            switch inputType
                case edFormats{1} %Damping Ratio / Natural Frequency
                    validateattributes(dampingRatio, {'double'}, {'nonempty', 'vector', 'real', 'finite', 'positive'}, 'validateErrorDynamicsInputs', 'dampingRatio');
                    validateattributes(naturalFrequency, {'double'}, {'nonempty', 'vector', 'real', 'finite', 'positive'}, 'validateErrorDynamicsInputs', 'naturalFrequency');

                    % Validate sizes on inputs that can be scalar or a size that
                    % depends on the number of joints in the associate rigid body
                    % tree object
                    robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateVariableJointDependentSize(dampingRatio, numJts, 'vector', 'dampingRatio');
                    robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateVariableJointDependentSize(naturalFrequency, numJts, 'vector', 'naturalFrequency');
                    
                case edFormats{2} %Step Response
                    validateattributes(settlingTime, {'double'}, {'nonempty', 'vector', 'real', 'finite', 'positive'}, 'validateErrorDynamicsInputs', 'settlingTime');
                    validateattributes(overshoot, {'double'}, {'nonempty', 'vector', 'real', 'finite', 'nonnegative', '<', 1}, 'validateErrorDynamicsInputs', 'overshoot');

                    % Validate sizes on inputs that can be scalar or a size that
                    % depends on the number of joints in the associate rigid body
                    % tree object
                    robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateVariableJointDependentSize(settlingTime, numJts, 'vector', 'settlingTime');
                    robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateVariableJointDependentSize(overshoot, numJts, 'vector', 'overshoot');
                otherwise
            end
        end
    end
    
    %% Task Space Motion Model Block
    
    methods (Static)
        function updateTSMotionModelInports(block, hasFExt)
            %updateTSMotionModelInports Update Task-Space motion model inports
            %   This method updates the inports of the task space motion
            %   model, which always accepts reference pose and velocity,
            %   but takes the external force as an optional input. To
            %   change this, that input is converted to either a constant
            %   block or an inport under the hood using the
            %   "setupBlockInput" utility.
                        
            if strcmp(hasFExt, 'on')
                % Add the inport and ensure it is the final inport
                robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.setupBlockInput(block, 'FExt', 'Inport', 5);
                robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.updateInportNumber(block, 'FExt', 3);
            else
                robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.setupBlockInput(block, 'FExt', 'Constant', 5);
            end
        end
        
        function validateTSMotionModelParameters(block, treeStruct, proportionalGain, derivativeGain, jointDamping, initConfig, initJointVel)
            %validateTSMotionModelParameters Validate task-space motion model edit-field parameters
            %   This method is called from the block mask and occurs at
            %   set-time. If the mask is opened, the Apply or OK buttons
            %   must be clicked. If done via set_param, the callback will
            %   be triggered immediately. These blocks often call internal
            %   MATLAB methods that perform similar checks, but those
            %   checks will not occur until runtime.
            
            % Do nothing if called in a library
            sys = bdroot(block);
            if bdIsLibrary(sys)
                return
            end
            
            % The sizes are dependent on the number of non-fixed joints in
            % the associated rigid body tree
            numJts = treeStruct.VelocityNumber;
            
            % Properties
            validateattributes(proportionalGain, {'double'}, {'nonempty', 'square', 'real', 'finite', 'ncols', 6}, 'validateTSMotionModelParameters', 'proportionalGain');
            validateattributes(derivativeGain, {'double'}, {'nonempty', 'square', 'real', 'finite', 'ncols', 6}, 'validateTSMotionModelParameters', 'derivativeGain');
            validateattributes(jointDamping, {'double'}, {'nonempty', 'vector', 'real', 'finite', 'nonnegative'}, 'validateTSMotionModelParameters', 'jointDamping');
            
            % Validate initial state
            validateattributes(initConfig, {'double'}, {'nonempty', 'vector', 'finite', 'real'}, 'validateTSMotionModelParameters', 'initConfig');
            validateattributes(initJointVel, {'double'}, {'nonempty', 'vector', 'finite', 'real'}, 'validateTSMotionModelParameters', 'initJointVel');
            
            % Validate sizes on parameters that can be scalar or a size
            % that depends on the number of joints in the associate rigid
            % body tree object
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateVariableJointDependentSize(jointDamping, numJts, 'vector', 'jointDamping');
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateVariableJointDependentSize(initConfig, numJts, 'vector', 'initConfig');
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.validateVariableJointDependentSize(initJointVel, numJts, 'vector', 'initJointVel');
        end        
    end
    
    %% Utility methods
    
    methods (Static)
        function [maxNumel, varargout] = matchParameterDimensions(varargin)
            %matchParamDimensions Match the dimensions of a number of input values
            %   The parameters varargin{1}...varargin{N} can be either scalar or
            %   vector-valued (with equivalent number of elements). This
            %   method finds the maximum number of elements in either
            %   parameter and then ensures that both values have that
            %   dimension. That is, either both are scalar, or both are
            %   vector-valued with the same number of elements.
            
            numelInputs = cellfun(@(x)numel(x), varargin);
            maxNumel = max(numelInputs);
            varargout = cell(1, nargin);
            for i = 1:nargin
                if maxNumel > numel(varargin{i})
                    varargout{i} = varargin{i}*ones(1,maxNumel);
                else
                    varargout{i} = varargin{i};
                end
            end
        end
        
        function assignValueToBlock(block, paramName, paramValue)
            %assignValueToBlock Assign scalar or vector values to the block via set_param interface
            
            % Add check to ensure only scalars and vectors are passed
            validateattributes(paramValue, {'numeric'}, {'vector'}, 'assignValueToBlock', 'paramValue');
            
            if isscalar(paramValue)
                paramString = num2str(paramValue);
            elseif isrow(paramValue)
                vectorString = ['[' repmat('%g ', 1, length(paramValue)) ']'];
                paramString = sprintf(vectorString, paramValue);
            elseif iscolumn(paramValue)
                vectorString = ['[' repmat('%g; ', 1, length(paramValue)) ']'];
                paramString = sprintf(vectorString, paramValue);
            end
            set_param(block, paramName, paramString);
        end
        
        function setupBlockInput(block, sourceBlockName, replacementBlockType, inportNum)
            %setupBlockInput Set up the source block in question
            %   Given a the name of a source block that feeds into the
            %   primary block and the name of the replacement type, delete
            %   the source block and replace it with the new one.
            
            %Query the original source block and get key parameters
            blkSys = getfullname(block);
            origSrcBlk = [blkSys '/' sourceBlockName];
            pos = get_param(origSrcBlk, 'Position');
            lh = get_param(origSrcBlk, 'LineHandles');
            blkType = get_param(origSrcBlk, 'BlockType');
            
            %Check if the block is the correct type. If not, remove it and
            %replace it with the correct one
            if ~strcmp(replacementBlockType, blkType)
                %Delete the original
                delete_line(lh.Outport)
                delete_block(origSrcBlk)

                %Add in the new one
                newSrcBlk = [blkSys '/' sourceBlockName]; 
                add_block(['built-in/' replacementBlockType], newSrcBlk, 'Position', pos)

                %Connect the new block
                add_line(blkSys, [sourceBlockName '/1'], ['MATLAB System/' num2str(inportNum)]);
            end
        end
        
        function updateInportNumber(block, inportName, portNum)
            
            %Query the original source block and get key parameters
            blkSys = getfullname(block);
            inportBlk = [blkSys '/' inportName];
            set_param(inportBlk, 'Port', num2str(portNum));
            
        end
        
        function TF = checkIfRunning(sys)
            %checkIfRunning Return TRUE the model is running
            
            TF = false;
            
            invalidStatus = {'external','running','compiled','restarting','paused','terminating'};
            if any(strcmpi(get_param(sys,'SimulationStatus'),invalidStatus))
                TF = true;
            end
        end
        
        function validateVariableJointDependentSize(varIn, numJts, varType, varName)
            %validateJointDependentVectorSize Check that VARIN is either a scalar or matrix/vector with dimension N
            %   This method verifies that the input is either a scalar or:
            %   (1) if VARTYPE is "vector", a vector with N elements, and
            %   (2) if VARTYPE is "matrix", a matrix of size [N N], where N
            %   is the number of non-fixed joints in the associated rigid
            %   body tree. A detailed error is thrown if the validation
            %   fails.
            
            if strcmp(varType, 'vector')
                isInvalidSize = (numel(varIn) > 1) && (numel(varIn) ~= numJts);
                if isInvalidSize
                     error(message('robotics:robotslmanip:blockmask:TreeDependentParameterVectorSize', varName, num2str(numJts)));
                end
            else
                isInvalidSize = (numel(varIn) > 1) && any(size(varIn) ~= [numJts numJts]);
                if isInvalidSize
                     error(message('robotics:robotslmanip:blockmask:TreeDependentParameterMatrixSize', varName, num2str(numJts)));
                end
            end
        end
    end
end
