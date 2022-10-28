classdef CollisionSet < robotics.core.internal.InternalAccess
%CollisionSet Set of collision geometries.
%   A CollisionSet is composed of multiple robotics.manip.internal.CollisionGeometry.
%   The set has utilities to "add" and "transform" collision geometries.

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

    properties(Access = private)
        %CollisionGeometries List of collision primitives
        CollisionGeometries

        %MaxElements The maximum number of elements the set can hold
        MaxElements
    end

    properties(SetAccess=private)
        %Size Number of elements in the set
        Size = 0

        %Tags Tags of the collision geometries
        Tags
    end

    methods
        function obj = CollisionSet(maxElements)
        %CollisionSet Constructor
        %   The set's size is bounded to MaxElements. If the target is
        %   MATLAB, the MaxElements are ignored by the set and the size of
        %   the set is unbounded.
            obj.MaxElements = maxElements;
            obj.CollisionGeometries = coder.nullcopy(cell(1,obj.MaxElements));
            obj.Tags = {};
            if(~coder.target("MATLAB"))
                defaultTag = "EmptyCollisionObject";

                % If the CollisionSet doesn't contain any collision geometries, 
                % this ensures that the default collision object is a
                % placeholder which will be overwritten when a collision
                % geometry is added to the robot. This also ensures that no
                % collision specific codegen API is pulled in for an empty
                % collision set.
                defaultCollisionObj = struct("GeometryInternal", coder.opaque('void*', '0'));
                defaultGeometry = robotics.manip.internal.CollisionGeometry( ...
                    defaultCollisionObj, eye(4), defaultTag);

                %During code generation, the collision set needs to pre-allocate the
                %collision geometry. Each of these geometries will be modified as
                %the size of the set increases, and as elements are added to it.
                for i = 1 : obj.MaxElements
                    obj.CollisionGeometries{i} = copy(defaultGeometry);
                end
            end
        end

        function add(obj, collisionGeometry)
        %add Adds a collision geometry to the set
            if(obj.Size < obj.MaxElements || coder.target("MATLAB"))
                obj.Size = obj.Size + 1;
                obj.CollisionGeometries{obj.Size} = collisionGeometry;
                if(coder.target('MATLAB'))
                    obj.Tags{obj.Size} = collisionGeometry.Tag;
                end
            else
                %For codegen we need to bound the memory and create default
                %values
                error("add:MaxElementsExceeded", "Cannot exceed MaxElements");
            end
        end

        function setTransform(obj, tForm)
        %setTransform Sets the world transform of the collision set
            for i = 1 : obj.Size
                obj.CollisionGeometries{i}.setLocalFrameOrigin(tForm)
            end
        end

        function [inCollision, minSepDist, witnessPts] =  ...
                checkCollision(obj, querySet)
            %checkCollision Checks for collision between two CollisionSets

            %Assume the two sets are far apart; they are not in collision.
            inCollision = false;
            minSepDist = inf;
            witnessPts = inf(3, 2);
            for i = 1 : obj.Size
                for j = 1 : querySet.Size
                    [inCollision, sepDist, wPts] = ...
                        obj.CollisionGeometries{i}.checkCollision(...
                            querySet.CollisionGeometries{j});

                    %If there is a collision, then use the result of
                    %checkCollision of geometries. No need to check for
                    %remaining pairs.
                    if(inCollision)
                        minSepDist = nan;
                        witnessPts = nan(3, 2);
                        return;
                    end
                    %If there is no collision, then we would want to compute the
                    %minimum separation distance, and the corresponding witness
                    %points.
                    if(sepDist < minSepDist)
                        minSepDist = sepDist;
                        witnessPts = wPts;
                    end
                end
            end
        end

        function [inCollision, minSepDist, witnessPts] = ...
                checkCollisionWithGeom(obj, worldObject, worldPose)
            %checkCollisionWithGeom Checks for collision between a CollisionSet and a CollisionGeometry
            if(coder.target('MATLAB'))
                collisionPrimitive = worldObject;
            else
                collisionPrimitive = worldObject.GeometryInternal;
            end
            geom = struct("CollisionPrimitive", collisionPrimitive, ...
                "WorldPose", worldPose);
            %Assume the two sets are far apart; they are not in collision.
            inCollision = false;
            minSepDist = inf;
            witnessPts = inf(3, 2);
            for i = 1 : obj.Size
                [inCollision, sepDist, wPts] = ...
                    obj.CollisionGeometries{i}.checkCollision(geom);

                %If there is a collision, then use the result of
                %checkCollision of geometries. No need to check for
                %remaining pairs.
                if(inCollision)
                    minSepDist = nan;
                    witnessPts = nan(3, 2);
                    return;
                end
                %If there is no collision, then we would want to compute the
                %minimum separation distance, and the corresponding witness
                %points.
                if(sepDist < minSepDist)
                    minSepDist = sepDist;
                    witnessPts = wPts;
                end
            end
        end

        function tforms = getTforms(obj)
            tforms = {};
            for i = 1 : obj.Size
                tforms{i} = obj.CollisionGeometries{i}.LocalPose;
            end
        end

        function vertices = getVertices(obj)
        %getVertices Returns the vertices of the set's geometries
            vertices = {};
            for i = 1 : obj.Size
                vizInfo = obj.CollisionGeometries{i}.getVisualInfo();
                vertices{i} = vizInfo.VisualMeshVertices;
            end
        end

        function faces = getFaces(obj)
        %getFaces Returns the faces of the set's geometries
            faces = {};
            for i = 1 : obj.Size
                vizInfo = obj.CollisionGeometries{i}.getVisualInfo();
                faces{i} = vizInfo.VisualMeshFaces;
            end
        end

        function newObj = clearSet(obj)
        %clearSet Clear the set
        %   ClearSet will set the size of the set to zero, empty the collision
        %   geometries, and the associated tags
            newObj = robotics.manip.internal.CollisionSet(obj.MaxElements);
        end
        
        function structArray = extractStruct(obj)
        %extractStruct Extract CollisionSet data to cell array of struct
        %   Extract the CollisionGeometry data within the CollisionSet to an
        %   array of struct
            emptyCollisionGeomStruct = robotics.manip.internal.CollisionGeomStruct.getEmptyCollisionGeomStruct;
            structArray = repmat(emptyCollisionGeomStruct, 1, obj.Size);
            for i=1:obj.Size
                structArray(i) = obj.CollisionGeometries{i}.extractStruct();
            end
        end

        function serializedData = serialize(obj)
        %serialize Serialize the set
        %   The serialized data is an array of strings which comprises of the
        %   serial data of every collision geometry in the set

            serializedData = repmat("", 1, obj.Size);
            for i = 1:obj.Size
                serializedData(i) = obj.CollisionGeometries{i}.serialize();
            end
        end

        function newObj = copy(obj)
        %copy
            newObj = robotics.manip.internal.CollisionSet(obj.MaxElements);
            newObj.Size = obj.Size;
            newObj.Tags = obj.Tags;
            for i = 1 : obj.Size
                newObj.CollisionGeometries{i} = copy(obj.CollisionGeometries{i});
            end
        end
    end
end
