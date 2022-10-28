classdef ClusterViolationHandlingType < uint8
    % This is an internal class and may be removed in a future release.
    % 
    
    % Copyright 2021 The MathWorks, Inc.
    
    %#codegen
    
    enumeration
        Terminate(1)
        SplitAndWarn(2)
        Split(3)
    end
end