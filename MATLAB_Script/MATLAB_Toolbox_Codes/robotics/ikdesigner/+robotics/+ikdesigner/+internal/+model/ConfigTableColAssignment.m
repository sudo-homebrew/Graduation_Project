classdef ConfigTableColAssignment < uint8
    %ConfigTableColAssignment Enumeration class to relate configuration table columns to the values they correspond to
    
    %   Copyright 2021 The MathWorks, Inc.
    
    enumeration
        %Name The first column is used to store configuration names
        Name            (1)

        %CollisionState The second column indicates the overall collision state
        CollisionState  (2)

        %Value The last column displays the value of the configuration
        Value           (3)
    end
end

