classdef CollisionGeometry < robotics.core.internal.InternalAccess
%CollisionGeometry Wrapper for the collision* primitives.
%   It defines pose with respect to a local frame, and a mutable world
%   pose. It also stores the tags that are populated by the
%   robotics.manip.internal.CollisionGeometryFactory. A
%   robotics.manip.internal.CollisionSet is composed of multiple CollisionGeometry

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

    properties(Access=private)
        %CollisionPrimitive A type of robotics.core.internal.CollisionGeometryBase
        %   This can be an instance of a collisionBox, collisionMesh,
        %   collisionCylinder, or a collisionSphere
        CollisionPrimitive
    end

    properties(SetAccess=private)
        %LocalPose The pose of the geometry in a local frame.
        %   The homogeneous transform of the collision object relative to a
        %   local frame
        LocalPose

        %WorldPose The pose of the geometry in a world frame.
        %   The homogeneous transform of the collision object relative to the
        %   world frame.
        WorldPose

        %Tag is the string tag associated with the geometry
        Tag
    end

    methods
        function obj = CollisionGeometry(collisionPrimitive, localPose, tag)
        %CollisionGeometry Constructor
        %   An instance is created via a collision primitive, a local frame
        %   and the associated tag.
            if(coder.target('MATLAB'))
                obj.CollisionPrimitive = collisionPrimitive;
                obj.Tag = tag;
            else
                % In codegen collisionPrimitive is a struct as this API is
                % internal
                obj.CollisionPrimitive = collisionPrimitive.GeometryInternal;
            end
            obj.LocalPose = localPose;
            obj.WorldPose = localPose;
        end

        function [collisionStatus, sepDist, witnessPts] = ...
                checkCollision(obj, geom)
            %checkCollision Check collision of the collision geometry with the input geom
            %Obtain cell-info which contains information of the primitive
            objInfo = robotics.manip.internal.CollisionGeometry.bundleGeometryInfo(obj);
            geomInfo = robotics.manip.internal.CollisionGeometry.bundleGeometryInfo(geom);
            if(coder.target('MATLAB'))
                [collisionStatus, sepDist, witnessPts] = ...
                    robotics.core.internal.intersect(...
                        objInfo{1}.GeometryInternal,...
                        objInfo{2}, ...
                        objInfo{3}, ...
                        geom.CollisionPrimitive.GeometryInternal,...
                        geomInfo{2}, ...
                        geomInfo{3}, ...
                        1);
                if collisionStatus
                    sepDist = nan;
                    witnessPts = nan(3,2);
                end
            else
                [status, sepDist, witnessPts] = ...
                    robotics.core.internal.coder.CollisionGeometryBuildable.checkCollision(...
                        objInfo{:}, geomInfo{:}, ...
                        true);
                collisionStatus = logical(status);
            end
        end
        
        function setLocalFrameOrigin(obj, tform)
        %setLocalFrameOrigin Set the relative transform of the local frame with respect to world frame
        %   tform is the homogeneous transform of the local frame with
        %   respect to the world frame.
            obj.WorldPose = tform * obj.LocalPose;
        end

        function visInfo = getVisualInfo(obj)
        %getVisualInfo Outputs a struct containing the geometry's visual data
            visInfo.VisualMeshVertices = ...
                obj.CollisionPrimitive.VisualMeshVertices;
            visInfo.VisualMeshFaces = ...
                obj.CollisionPrimitive.VisualMeshFaces;
        end
        
        function structObj = extractStruct(obj)
        %extractStruct Extract collision data to a struct
        %   Extract the CollisionGeometry data to a struct
        %   containing properties to store collision type, collision
        %   dimensions and collision pose.
            structObj = robotics.manip.internal.CollisionGeomStruct.populateCollisionGeomStruct(obj.CollisionPrimitive, obj.LocalPose);
        end

        function serializedData = serialize(obj)
        %serialize Serialize the collision geometry
        %   The serialized data is a string which is a call to a collision type
        %   (collisionBox, collisionCylinder, ...etc), and the local pose of the
        %   collision geometry
        %   E.g. A collision box of dimensions [1, 2, 3] with a local pose of
        %   eul2tform([0, pi, 0]) will be serialized to a string of
        %   ['collisionBox(1, 2, 3)', mat2str(eul2tform([0, pi, 0))]


            collisionType = class(obj.CollisionPrimitive);
            if(collisionType == "collisionBox")
                dimensions = sprintf('(%d, %d, %d)', ...
                                     obj.CollisionPrimitive.X, ...
                                     obj.CollisionPrimitive.Y, ...
                                     obj.CollisionPrimitive.Z);
            elseif(collisionType == "collisionCylinder")
                dimensions = sprintf('(%d, %d)', ...
                                     obj.CollisionPrimitive.Radius, ...
                                     obj.CollisionPrimitive.Length);
            elseif(collisionType == "collisionSphere")
                dimensions = sprintf('(%d)', ...
                                     obj.CollisionPrimitive.Radius);
            elseif(collisionType == "collisionMesh")
                dimensions =  ...
                    sprintf('(%s)', mat2str(obj.CollisionPrimitive.Vertices));
            end
            localPose = sprintf(', %s', mat2str(obj.LocalPose));
            serializedData = string([collisionType, dimensions, localPose]);
        end

        function newObj = copy(obj)
        %copy Copy a collision geometry. 
        %   The "copy" is only used by the internal
        %   robotics.manip.internal.CollisionSet that ties with a
        %   robotics.manip.internal.RigidBody. This copy is different in MATLAB
        %   versus codegen. In codegen, given that the underlying geometry and
        %   its local pose cannot be modified once assigned by a rigidBody during
        %   addCollision this copy can be re-used for collision checking.
            if(coder.target('MATLAB'))
                primitive = copy(obj.CollisionPrimitive);
                tag = obj.Tag;
                localPose = obj.LocalPose;
                newObj = robotics.manip.internal.CollisionGeometry(primitive, ...
                                                                  localPose, ...
                                                                  tag);
                newObj.WorldPose = obj.WorldPose;
            else
                newObj = obj;
            end
        end

    end
       
    methods (Access = private, Static)
        function info = bundleGeometryInfo(geom)
        %bundleGeometryInfo Output a cell containing the primitive and its pose
        %   The function is used to pass the primitive data, the translation
        %   vector, and the quaternion to the intersect function during code
        %   generation

            info = {geom.CollisionPrimitive, ...
                    geom.WorldPose(1:3, end), ...
                    robotics.core.internal.rotm2quat(geom.WorldPose(1:3, 1:3))};
        end
    end

    methods
        function delete(obj)
            if(~coder.target('MATLAB'))
                    robotics.core.internal.coder.CollisionGeometryBuildable.destructGeometry(...
                        obj.CollisionPrimitive);
            end
        end
    end

end
