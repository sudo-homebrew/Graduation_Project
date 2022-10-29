classdef GazeboModelElemSelectBlockMask < robotics.slgazebo.internal.block.EntitySelectionMask
    %This class is for internal use only. It may be removed in the future.
    
    %GazeboModelElemSelectBlockMask - Block mask callbacks for Gazebo
    %Select Model Element block
    
    %   Copyright 2019 The MathWorks, Inc.
    
    properties (Constant)           
        
        %ModelElementSelector get the Gazebo select entity dialog object
        ModelElemSelector = robotics.slgazebo.internal.dlg.GazeboModelElementSelector();  
        
        %ModelSourceBlockName Block that contains the model name
        ModelSourceBlockName = 'SourceBlockModel';
        
        %JointLinkSourceBlockName Block that contains the joint or link
        %name
        JointLinkSourceBlockName = 'SourceBlockJointLink';
    end  
    
    methods
        function maskInitialize(obj, block)
            % This is invoked after the callbacks
            
            % Lookup the current entity type to decide how to show the icon
            % and output port label
            entityName = get_param(block, 'modelElemName');
            entityType = obj.ModelElemSelector.getOutputEntityType(entityName);
            
            if ~isempty(entityType)
                % Need to set both SelectType and OutputLabel
                % SelectType is a hidden drop-down that controls the icon
                % OutputLabel is a hidden edit field that controls the
                % output port label
                switch entityType
                    case 1
                        set_param(gcb, 'SelectType', 'Link');
                        set_param(gcb, 'OutputLabel', 'Link');
                    case 2
                        set_param(gcb, 'SelectType', 'Joint');
                        set_param(gcb, 'OutputLabel', 'Joint');
                end
            end
            
            arraySize = slResolve(get_param(gcb, 'OutputArraySize'), gcb);
            validateattributes(arraySize, {'numeric'}, {'positive', 'integer','real','scalar','>',1}, gcb, 'OutputVectorSize');
        end
    end
    
    methods(Static)
        
        function dispatch(methodName, varargin)
            obj = robotics.slgazebo.internal.block.GazeboModelElemSelectBlockMask();
            obj.(methodName)(varargin{:});
        end
        
    end
end
