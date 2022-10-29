classdef GazeboModelElementSelector < handle
    %This class is for internal use only. It may be removed in the future.
    
    %  GazeboModelElementSelector opens a DDG dialog that lets the user select
    %  from a list of Gazebo model elements. Once the user accepts the changes
    %  (or cancels the dialog), a callback is invoked with the closure
    %  action and selected message type.
    %
    %  Sample use:
    %   selector = robotics.slgazebo.internal.dlg.GazeboModelElementSelector;
    %   % The first argument is the message type to select by default
    %   selector.openDialog('std_msgs/int32', @(isAccepted,msgType) disp(msgType));
    
    %   Copyright 2019 The MathWorks, Inc.
    
    properties(SetAccess=private)
        GazeboModelElement = ''
        ModelElemList
        CloseFcnHandle = function_handle.empty
        
        %GetModelElemList Object to get the Gazebo entity list
        GetModelEntityList = robotics.slgazebo.internal.util.GetModelElementList;
    end
    
    
    methods
        
        function obj = GazeboModelElementSelector()
        end
        
        function outputEntityType = getOutputEntityType(obj, name)
            %getOutputEntityType get the type of the Gazebo entity
            
            if any(convertCharsToStrings(obj.GetModelEntityList.Links) == name)
                outputEntityType = 1; % output entity type is link
            elseif any(convertCharsToStrings(obj.GetModelEntityList.Joints) == name)
                outputEntityType = 2; % output entity type is joint
            else
                outputEntityType = [];
            end
            
        end
        
        function dlg = openDialog(obj, initialModelSelection, closeFcnHandle)
            %openDialog opens a dialog for user to choose model joints or
            %links
            %
            % closeFcnHandle: handle to function that takes two arguments
            % isAcceptedSelection and selectedModelElem.
            % isAcceptedSelection would be true if user clicked on 'ok',
            % would be false if user clicked on 'cancel' or closed window.
            % selectedModelElem is the last selected gazebo model element
            % (string)
            
            %get the list of model elements
            obj.GetModelEntityList.updateEntityList();
            obj.ModelElemList = sort([obj.GetModelEntityList.Joints(:);obj.GetModelEntityList.Links(:)]);
            
            assert(ischar(initialModelSelection) || isempty(initialModelSelection));
            validateattributes(closeFcnHandle, {'function_handle'}, {'scalar'});
            obj.CloseFcnHandle = closeFcnHandle;
            
            if (isempty(obj.ModelElemList))
                error(message('robotics:robotslgazebo:entityselector:EmptyGazeboWorld'));
            end
            
            dlg = DAStudio.Dialog(obj);
            if isempty(initialModelSelection)
                return;
            end
            
            % Find initial message selection, if any
            index = find(strcmpi(initialModelSelection, obj.ModelElemList));
            if ~isempty(index)
                dlg.setWidgetValue('gazebomodellist', index-1); % zero-based
                obj.GazeboModelElement = obj.ModelElemList{index};
            end
        end
    end
    
    
    methods(Hidden)
        % Called when user selects an item from the list
        function dlgCallback(obj, dlg, tag, value) %#ok<INUSL>
            obj.GazeboModelElement = obj.ModelElemList{value+1}; % value is zero-based
            dlg.refresh;
        end
        
        
        function dlgClose(obj, closeaction)
            % closeaction is 'ok' if user clicked OK
            % 'cancel' if user clicked cancel or closed window
            if ~isempty(obj.CloseFcnHandle)
                isAcceptedSelection = strcmpi(closeaction, 'ok');
                try
                    feval(obj.CloseFcnHandle, isAcceptedSelection, obj.GazeboModelElement);
                catch
                    % Absorb all errors. If they are propagated back to
                    % DDG, this causes MATLAB to crash, (Can't convert to
                    % warnings as they are not displayed either).
                end
            end
        end
        
        
        function dlgstruct = getDialogSchema(obj)
            msglist.Name    = '';
            msglist.Type    = 'listbox';
            msglist.Entries = obj.ModelElemList;
            msglist.Tag     = 'gazebomodellist';
            msglist.MultiSelect = false;
            msglist.ObjectMethod = 'dlgCallback'; % call method on UDD source object
            msglist.MethodArgs = {'%dialog', '%tag', '%value'}; % object handle is implicit first arg
            msglist.ArgDataTypes = {'handle', 'string', 'mxArray'}; % 'handle' is type of %dialog
            msglist.Value = 0;
            msglist.NameLocation = 2; % top left
            
            % Main dialog
            dlgstruct.DialogTitle = message('robotics:robotslgazebo:entityselector:DialogTitle').getString;
            dlgstruct.HelpMethod = 'helpview';
            dlgstruct.HelpArgs =  {fullfile(docroot, 'robotics', 'helptargets.map'), 'rstGazeboSelectEntity'};
            dlgstruct.CloseMethod = 'dlgClose';
            dlgstruct.CloseMethodArgs = {'%closeaction'};
            dlgstruct.CloseMethodArgsDT = {'string'};
            
            % Make this dialog modal wrt to other DDG dialogs
            % (doesn't block MATLAB command line)
            dlgstruct.Sticky = true;
            
            % Buttons to show on dialog (these are options to pass to DDG,
            % not the final strings, so there is no need to use message
            % catalog)
            dlgstruct.StandaloneButtonSet =  ...
                {'Ok', 'Cancel', 'Help'}; % also available: 'Revert', 'Apply'
            
            dlgstruct.Items = {msglist};
        end
    end
end
