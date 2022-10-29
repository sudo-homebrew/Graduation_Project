classdef AppState < uint8
    %APPSTATE Enumeration class for app state
    
    %   Copyright 2021 The MathWorks, Inc.
    
    enumeration
        Startup         (1)
        Initialization  (2)
        RobotGallery    (3)
        InSession       (4)
    end
end

