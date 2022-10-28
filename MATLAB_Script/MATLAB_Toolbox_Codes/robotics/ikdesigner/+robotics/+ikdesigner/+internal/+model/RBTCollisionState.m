classdef RBTCollisionState < uint8
    %RBTCollisionState Enumeration class for the rigid body tree collision state
    %   The rigid body tree can have three collision states:
    %
    %      CollisionFree:    The robot state has been evaluated and IS NOT 
    %                        in collision with itself or any collision
    %                        bodies
    %
    %      InCollision:      The robot state has been evaluated and IS in 
    %                        collision with itself or with collision bodies
    %
    %      NotEvaluated:     The robot collision state has not been 
    %                        evaluated
    
    %   Copyright 2021 The MathWorks, Inc.
    
    enumeration
        CollisionFree  (0)
        InCollision    (1)
        NotEvaluated   (2)
    end
end

