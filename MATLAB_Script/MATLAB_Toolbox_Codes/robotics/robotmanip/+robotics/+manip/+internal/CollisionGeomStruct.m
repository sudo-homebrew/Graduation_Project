classdef CollisionGeomStruct
%CollisionGeomStruct Utility class to convert between Collision objects and struct.
%   This class provides methods that can be used to create a Collision
%   geometry struct from collision data and also to convert back
%   from the struct to collision objects.

%   Copyright 2021 The MathWorks, Inc.

%#codegen
    properties (Constant)
        COLLISION_BOX = uint8(0);
        COLLISION_CYLINDER = uint8(1);
        COLLISION_SPHERE = uint8(2);
        COLLISION_MESH = uint8(3);
    end
    
    methods (Static)
        function collisionGeomStruct = populateCollisionGeomStruct(collPrim, localPose)
            %populateCollisionGeomStruct Create collision geometry struct
            %   Create a struct containing collision type ('box',
            %   'cylinder', 'sphere' or 'mesh'), collision
            %   dimensions([x,y,z], [radius, height], [radius],
            %   [vertices]) and collision local pose.
            %
            %   For example: a collisionBox(1,2,3) with Pose eye(4) would
            %   have the struct:
            %   struct with fields:
            %
            %    CollisionType: uint8(0)
            %    CollisionDims: [1, 2, 3]
            %    CollisionLocalPose: eye(4)
            
            collisionGeomStruct = robotics.manip.internal.CollisionGeomStruct.getEmptyCollisionGeomStruct;
            type = class(collPrim);
            switch type
                case "collisionBox"
                    collisionGeomStruct.CollisionType = robotics.manip.internal.CollisionGeomStruct.COLLISION_BOX;
                    collisionGeomStruct.CollisionDims = [collPrim.X, collPrim.Y, collPrim.Z];
                case "collisionCylinder"
                    collisionGeomStruct.CollisionType = robotics.manip.internal.CollisionGeomStruct.COLLISION_CYLINDER;
                    collisionGeomStruct.CollisionDims = [collPrim.Radius, collPrim.Length, 0];

                case "collisionSphere"
                    collisionGeomStruct.CollisionType = robotics.manip.internal.CollisionGeomStruct.COLLISION_SPHERE;
                    collisionGeomStruct.CollisionDims = [collPrim.Radius, 0, 0];

                case "collisionMesh"
                    collisionGeomStruct.CollisionType = robotics.manip.internal.CollisionGeomStruct.COLLISION_MESH;
                    collisionGeomStruct.CollisionDims = collPrim.Vertices;
            end
            collisionGeomStruct.CollisionLocalPose = localPose;
        end
        
        function collGeom = populateCollisionGeomFromStruct(collisionGeomStruct)
            %populateCollisionGeomFromStruct Create collision objects from
            %collision struct
            %   This method returns a CollisionGeometry created from a
            %   collision object defined by the type in the struct.
            dims = collisionGeomStruct.CollisionDims;
            
            switch collisionGeomStruct.CollisionType
                case robotics.manip.internal.CollisionGeomStruct.COLLISION_BOX
                    collGeom = robotics.manip.internal.CollisionGeometryFactory.makeCollisionGeometryFrom(...,
                        "Box",  [dims(1), dims(2), dims(3)], collisionGeomStruct.CollisionLocalPose);
                case robotics.manip.internal.CollisionGeomStruct.COLLISION_CYLINDER
                    collGeom = robotics.manip.internal.CollisionGeometryFactory.makeCollisionGeometryFrom(...,
                        "Cylinder", [dims(1), dims(2)], collisionGeomStruct.CollisionLocalPose);
                case robotics.manip.internal.CollisionGeomStruct.COLLISION_SPHERE
                    collGeom = robotics.manip.internal.CollisionGeometryFactory.makeCollisionGeometryFrom(...,
                        "Sphere", dims(1), collisionGeomStruct.CollisionLocalPose);
                case robotics.manip.internal.CollisionGeomStruct.COLLISION_MESH
                    collGeom = robotics.manip.internal.CollisionGeometryFactory.makeCollisionGeometryFrom(...,
                        "Mesh", dims, collisionGeomStruct.CollisionLocalPose);
                otherwise
                    collGeom = robotics.manip.internal.CollisionGeometryFactory.makeCollisionGeometryFrom(...,
                        "Box", [0, 0, 0], collisionGeomStruct.CollisionLocalPose);
            end
        end
        
        function emptyCollisionGeomStruct = getEmptyCollisionGeomStruct
        %getEmptyCollisionGeomStruct Create a skeletal struct for collision geometry struct
        
            %Initialize collision struct
            emptyCollisionGeomStruct = struct('CollisionType', uint8(0), ...
                'CollisionDims', zeros(1,3), ...
                'CollisionLocalPose', eye(4));
        end
    end
end