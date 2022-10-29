classdef URDFCollisionParameterParser < robotics.manip.internal.InternalAccess
%URDFCollisionParameterParser populates type, parameters and transform of a link.
%   The CollisionParameterParser will populate the collision data from a URDF
%   link.

%   Copyright 2020-2021 The MathWorks, Inc.

    methods(Static)
        function [type, parameter, T] = extractCollisionParameters(...
            linkCollision, ...
            meshPath, ...
            pathToURDF, ...
            sourceData, ...
            fileInput)
            %extractCollisionParameters Parses collision parameters from collision link
            %   MESHPATH contains the directory to the mesh files specified by the user.
            %   PATHTOURDF contains the path to the URDF linking these mesh
            %   files. SOURCEDATA specifies the source of the mesh, either
            %   URDF or SDF. FILEINPUT specifies the name of URDF/SDF file provided by the user.

            xyz = linkCollision.Origin.xyz;
            rpy = linkCollision.Origin.rpy;
            ypr = [rpy(3) rpy(2) rpy(1)];
            R = eul2rotm(ypr, "ZYX");
            T = [R xyz(:); [0 0 0 1]];

            type = linkCollision.Geometry.Type;

            switch type
              case "mesh"
                parameter = cell(1,2);
                parameter{2} = linkCollision.Geometry.Scale; % scale
                %This is the absolute path of the mesh file to read from
                fn = linkCollision.Geometry.FileName; % mesh filename                
                parameter{1} = robotics.manip.internal.RobotDataImporter.findMeshFilePath(fn, pathToURDF, meshPath, sourceData, fileInput);
              case "box"
                parameter = linkCollision.Geometry.Size; % [xl, yl, zl]
              case "cylinder"

                %collisionCylinder cannot have zero for radius, hence, the
                %parser outputs an eps value for radius.
                parameter = [max(linkCollision.Geometry.Radius, eps), ...
                             linkCollision.Geometry.Length]; % [radius, length]
              case "sphere"

                %collisionSphere cannot have zero for radius, hence, the
                %parser outputs an eps value for radius.
                parameter = max(linkCollision.Geometry.Radius, eps); % radius
            end
        end
    end
end
