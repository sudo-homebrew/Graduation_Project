classdef ConstraintUpdate < uint32
    %ConstraintUpdate Enumeration class for the constraint update type
    
    %   Copyright 2021 The MathWorks, Inc.
    
    enumeration
        NewOverlay         (0)
        DeleteOverlay      (1)
        DataUpdate         (2)
        RefBodySelection   (3)
        EEBodySelection    (4)
    end
end

