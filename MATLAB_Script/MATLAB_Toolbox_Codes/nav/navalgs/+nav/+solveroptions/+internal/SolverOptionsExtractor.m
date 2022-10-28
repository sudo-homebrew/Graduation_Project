classdef SolverOptionsExtractor < nav.algs.internal.InternalAccess
    %SOLVEROPTIONSEXTRACTOR Utility to extract information from user facing
    %   solver options object.
    
    %   Copyright 2020 The MathWorks, Inc.
    
    %#codegen    
    methods (Hidden, Static)
        function paramStruct = dump(solverOptions)
            %dump
            paramStruct = solverOptions.OptionsInternal.dump();
        end
        
    end
end

