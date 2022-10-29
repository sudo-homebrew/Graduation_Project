classdef CustomDisplayMCOS < matlab.mixin.CustomDisplay
    %   This class is for internal use only. It may be removed in the future.
    
    %   Copyright 2018 The MathWorks, Inc.
    
    %#codegen
    
    methods(Access = private, Static)
        % Redirect to enable codegen. We need this until
        % matlab.mixin.CustomDisplay supports codegen
        function name = matlabCodegenRedirect(~)
            name = 'fusion.internal.mixin.MCOSClass';
        end
    end    
end
