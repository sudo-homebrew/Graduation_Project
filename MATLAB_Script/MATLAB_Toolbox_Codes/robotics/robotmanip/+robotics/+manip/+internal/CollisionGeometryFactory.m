classdef CollisionGeometryFactory
%CollisionGeometryFactory Factory that creates collision geometries
%   A static factory that creates collision geometries via a geometry type or
%   the collision* objects themselves.

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

    properties(Constant)
        %BoxStr String identifier for a box type
        BoxStr = "Box";

        %SphStr String identifier for a sphere type
        SphStr = "Sphere";

        %CylStr String identifier for a cylinder type
        CylStr = "Cylinder";

        %MeshStr String identifier for a mesh type
        MeshStr = "Mesh";
    end

    methods(Static)
        function [geom, errorCode, geometryFile] = makeCollisionGeometryFrom(geomType, parameters, tform)
        %makeCollisionGeometryFrom Returns an instance of CollisionGeometry and any corresponding error codes.
        %   [GEOM, ERRORCODE, GEOMETRYFILE] = makeCollisionGeometryFrom(GEOMTYPE, PARAMETERS, TFORM)
        %   returns a GEOM which is an instance of
        %   robotics.manip.internal.CollisionGeometry. Additionally, it
        %   returns an integer scalar error code ERRORCODE encountered while
        %   reading the mesh data from the associated file GEOMETRYFILE.
        %   The instance is created using the geometry's type GEOMTYPE, its
        %   size (PARAMETERS), and the local transform TFORM.

            if(coder.target('MATLAB'))
                % Remove this when addressing g2417386 
                parameterCellArray = num2cell(parameters);
            end
            factory = robotics.manip.internal.CollisionGeometryFactory;

            %Default error code: assume that there is no error
            errorCode = 0;
            geometryFile = "";

            %During codegen we cannot use the collisionBox/Sphere/Cylinder/Mesh
            %as the handle goes out of scope at the end of this function and
            %will destroy the underlying memory; this will cause de-referencing
            %the GeometryInternal to segfault. 
            if(strcmpi(geomType, factory.BoxStr))
                if(coder.target('MATLAB'))
                    collisionObj = collisionBox(parameterCellArray{:});
                else
                    collisionObj = struct("GeometryInternal", ...
                        robotics.core.internal.coder.CollisionGeometryBuildable.makeBox(...
                            parameters(1), ...
                            parameters(2), ...
                            parameters(3)));
                end

            elseif(strcmpi(geomType, factory.SphStr))
                if(coder.target('MATLAB'))
                    collisionObj = collisionSphere(parameterCellArray{:});
                else
                    collisionObj = struct("GeometryInternal", ...
                        robotics.core.internal.coder.CollisionGeometryBuildable.makeSphere(...
                            parameters));
                end

            elseif(strcmpi(geomType, factory.CylStr))
                if(coder.target('MATLAB'))
                    collisionObj = collisionCylinder(parameterCellArray{:});
                else
                    collisionObj = struct("GeometryInternal", ...
                        robotics.core.internal.coder.CollisionGeometryBuildable.makeCylinder(...
                            parameters(1), ...
                            parameters(2)));
                end

            elseif(strcmpi(geomType, factory.MeshStr))
                scale = [1 1 1];
                if(~isnumeric(parameters)) %If the parameters are not vertices
                    if ischar(parameters)  %If the parameters are a file path
                        geometryFile = parameters;
                    else
                        geometryFile = parameters{1};%If it is a string
                        if length(parameters)>1      %If the scale is given
                            scale = parameters{2};
                        end
                    end

                    %Check if the geometryFile is an absolute path and it
                    %exists. If it doesn't then we find it on path
                    if ~isempty(which(geometryFile))
                        geometryFile = which(geometryFile);
                    end
                    
                    %read CAD file
                    result = robotics.manip.internal.readCADFile(geometryFile);
                    
                    if result.IsSuccess
                        V = [scale(1)*result.Vertices(:,1), ...
                             scale(2)*result.Vertices(:,2), ...
                             scale(3)*result.Vertices(:,3)];
                        %This is because of g2288676. Now, if the geometry file
                        %doesn't have enough vertices to qualify as an STL file,
                        %it is possible that the V can be empty. In that case,
                        %we should simply create a default mesh

                        if(isempty(V))
                            V = zeros(1, 3);
                        end

                        collisionObj = collisionMesh(V);
                    else
                        geom = [];
                        errorCode = result.ErrorCode;
                        return;
                    end
                else
                    V = parameters;
                    if(coder.target('MATLAB'))
                        collisionObj = collisionMesh(V);
                    else
                        collisionObj = struct("GeometryInternal", ...
                            robotics.core.internal.coder.CollisionGeometryBuildable.makeMesh(...
                            V, size(V, 1)));
                    end
                        
                end
                % Mesh is special for its tag as for a file name tag should have
                % File Name, but for Vertices it should be size of vertices.
                tag = factory.makeTagFrom(geomType, parameters);
                geom = robotics.manip.internal.CollisionGeometry(...,
                    collisionObj, tform, tag);
                return;
            else
                geom = [];
                return;
            end
            tag = factory.makeTagFrom(geomType, parameters);
            geom = robotics.manip.internal.CollisionGeometry(...
                collisionObj, tform, tag);
        end

        function geom = makeCollisionGeometryFromCollisionObj(collisionObj, tform)
        %makeCollisionGeometryFromCollisionObj Returns an instance of CollisionGeometry
        %   The instance is created using the collisionObj, and the transform.
            if(isa(collisionObj, 'collisionBox'))
                geomType = "box";
                parameters = [collisionObj.X, collisionObj.Y, collisionObj.Z];
            elseif(isa(collisionObj, 'collisionSphere'))
                geomType = "sphere";
                parameters = collisionObj.Radius;
            elseif(isa(collisionObj, 'collisionMesh'))
                geomType = "mesh";
                parameters = collisionObj.Vertices;
            elseif(isa(collisionObj, 'collisionCylinder'))
                geomType = "cylinder";
                parameters = [collisionObj.Radius, collisionObj.Length];
            end
            geom = ...
                robotics.manip.internal.CollisionGeometryFactory.makeCollisionGeometryFrom(...
                geomType, parameters, tform);
        end

        function tag = makeTagFrom(geomType, parameters)
        %makeTagFrom Creates a tag for the geometry from geometry's type.
        %   The tag is made based on the geometry's type and the
        %   parameters.
        %   Sample tags:
        %       Box         - "Box Size [1, 2, 3]"
        %       Cylinder    - "Cylinder Radius 42 Length 23"
        %       Sphere      - "Sphere Radius 42"
        %       Mesh        - "Mesh Vertices [90, 3]"
        %                   "Mesh Filename link0.stl"
            factory = robotics.manip.internal.CollisionGeometryFactory;
            tag = [];
            if(coder.target('MATLAB'))
                if(strcmpi(geomType, factory.BoxStr))
                    tag.Type = factory.BoxStr;
                    tag.Parameters = strcat(" Size ", mat2str(parameters));
                elseif(strcmpi(geomType, factory.SphStr))
                    tag.Type = factory.SphStr;
                    tag.Parameters = strcat(" Radius ", mat2str(parameters));
                elseif(strcmpi(geomType, factory.CylStr))
                    tag.Type = factory.CylStr;
                    radiusStr = strcat(" Radius ", mat2str(parameters(1)));
                    lengthStr = strcat(" Length ", mat2str(parameters(2)));
                    tag.Parameters = strcat(radiusStr, lengthStr);
                elseif(strcmpi(geomType, factory.MeshStr))
                    tag.Type = factory.MeshStr;
                    if(isnumeric(parameters))
                        vertStr = mat2str(size(parameters));
                        tag.Parameters = strcat(" Vertices ", vertStr);
                    else
                        if(length(parameters) > 1 && ~ischar(parameters)) % a scale was given
                            tag.Parameters = strcat(" Filename ", parameters{1});
                        else
                            tag.Parameters = strcat(" Filename ", parameters);
                        end

                    end
                end
                tag = strcat(tag.Type, tag.Parameters);
                tag = char(tag);
            end
        end

    end
end
