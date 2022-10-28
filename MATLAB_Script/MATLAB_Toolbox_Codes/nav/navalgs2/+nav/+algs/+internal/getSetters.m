classdef getSetters < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%getSetters Helper class used for modifying properties on mobileRobotPropagator components

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties (Abstract)
        SettableParams
    end
    
    methods (Access = ?nav.algs.internal.InternalAccess)
        function propStruct = getParameters(obj)
            propStruct = struct();
            for i = 1:numel(obj.SettableParams)
                propStruct.(obj.SettableParams{i}) = obj.(obj.SettableParams{i});
            end
        end
        
        function setParameters(obj, propagator, propStruct)
            % Standard property update
            names = fieldnames(propStruct);
            for i = 1:numel(names)
                validatestring(names{i},obj.SettableParams,'setParameters',names{i});
                obj.(names{i}) = propStruct.(names{i});
            end
            
            % Post Update Hook
            obj.postParamUpdate(propagator, propStruct);
        end
        
        function postParamUpdate(obj, propagator, propStruct) %#ok<INUSD> 
        %postParamUpdate This defaults to a no-op, but inheriting classes
        %may override
        end
    end
    
    methods (Static, Hidden)
        function result = matlabCodegenNontunableProperties(~)
        %matlabCodegenSoftNontunableProperties Mark properties as nontunable during codegen
        %
        % Marking properties as 'Nontunable' indicates to Coder that
        % the property should be made compile-time Constant.
            result = {'SettableParams'};
        end
    end
end
