classdef SolverOptionCustomDisplay < nav.algs.internal.InternalAccess & ...
                                     matlab.mixin.CustomDisplay
                                     
    %SOLVEROPTIONCUSTOMDISPLAY
    
    %   Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    
    methods (Abstract, Access = protected)
        getDisplayOptions(obj)
        getDetailedSolverName(obj)
    end

    methods (Access = protected)
        function groups = getPropertyGroups(obj)
            %GETPROPERTYGROUPS Return the property groups for the display
            %
            %   This method must be implemented as this class inherits from
            %   matlab.mixin.CustomDisplay.
            %
            %   This method is not used during codegen.
            
            if isscalar(obj) 
                allOptions = getDisplayOptions(obj);
                                
                groups = matlab.mixin.util.PropertyGroup(allOptions);        
            else
                groups = getPropertyGroups@matlab.mixin.CustomDisplay(obj);
            end
        end
        
        function header = getHeader(obj)
            %getHeader Defines the header in MATLAB object display
            %
            %   This method is not used during codegen.
            
            if ~isscalar(obj)
                header = getHeader@matlab.mixin.CustomDisplay(obj);
            else
                clsName = matlab.mixin.CustomDisplay.getClassNameForHeader(obj);
                headerStr = [clsName, ' (', getDetailedSolverName(obj), ') options:'];
                header = sprintf('%s\n',headerStr);
            end
        end
    end
    
    
    methods(Access = private, Static)
        % during codegen, this class is replaced with InternalAccess
        function redirClsName = matlabCodegenRedirect(~)
            redirClsName = 'nav.algs.internal.InternalAccess';
        end
    end
end

