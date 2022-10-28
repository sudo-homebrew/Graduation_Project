classdef CollisionInputValidator < robotics.manip.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.
%CollisionInputValidator Input validator utilities for rigidBody/addCollision

%   Copyright 2020-2021 The MathWorks, Inc.
%#codegen

    methods(Static, Access=?robotics.manip.internal.InternalAccess)

        function validType = validateType(type)
        %validateType Validates the type of collision geometry.
        %   The function validates the types of collision geometries that
        %   are supported by rigidBody/addCollision

            validType = validatestring(type, ...
                                       {'mesh', 'box', 'cylinder', 'sphere'}, ...
                                       'addCollision', 'type');

        end

        function validType = validateTypeParameterInputs(type, parameter, tform)
        %validateTypeParameterInputs Validates the type-parameter input to addCollision
        %   An overload of rigidBody/addCollision is
        %
        %       addCollision(TYPE, PARAMETER, TFORM) where TYPE can be a
        %       "box", "mesh", "cylinder", or "sphere". PARAMETER
        %       corresponds to the TYPE of the collision -  it can be the
        %       size of a box/sphere/cylinder, or the vertices/filename of
        %       a mesh
        %
        %   The function validates the inputs to the above overload

        %Validate the input local transform
            robotics.manip.internal.CollisionInputValidator.validateTransform(tform);

            %Validate the type of geometries
            validType = ...
                robotics.manip.internal.CollisionInputValidator.validateType(type);


            switch validType
              case 'mesh'

                %A mesh requires either one of two parameters:
                %   - filename
                %   - [N, 3] vertices where N is the number of vertices
                %
                %The validation of the numeric data will be handled by the
                %collisionMesh object

                %If the parameter is not numeric
                if(~isnumeric(parameter))
                    robotics.internal.validation.validateString(...
                        parameter,...
                        false,...
                        'addCollision', ...
                        'filename')
                end

              case 'box'

                %A box requires [X, Y, Z]. The X, Y, and Z will be passed on
                %the collisionBox, hence no validation on their values is
                %required.
                validateattributes(parameter, ...
                                   {'single','double', 'nonempty', 'finite'}, ...
                                   {'vector', 'numel', 3}, ...
                                   'addCollision', 'parameter');

              case 'cylinder'

                %A cylinder requires [Radius, Length]. The Radius and Length
                %will be passed on to the collisionCylinder, hence no
                %validation on their values is required.
                validateattributes(parameter, ...
                                   {'single','double', 'nonempty', 'finite'}, ...
                                   {'vector', 'numel', 2}, ...
                                   'addCollision', 'parameter');

              case 'sphere'

                %A sphere requires Radius. The Radius will be passed on to
                %the collisionSphere, hence no validation on their values is
                %required.
                validateattributes(parameter, ...
                                   {'single','double', 'nonempty', 'finite'}, ...
                                   {'scalar'}, ...
                                   'addCollision', 'parameter');
            end
        end

        function validateCollisionObjectInput(collisionObj, tform)
        %validateCollisionObjectInput Validates the collision object input to addCollision
        %   An overload of rigidBody/addCollision is
        %
        %       addCollision(COLLISIONOBJ, TFORM) where COLLISIONOBJ can be a collisionMesh/Cylinder/Box/Sphere.
        %
        %   The function validates the inputs to the above overload

            robotics.manip.internal.CollisionInputValidator.validateTransform(tform);

            %Validate that the input collisionObj is a
            %collisionMesh/Box/Sphere/Cylinder
            validateattributes(collisionObj, ...
                               {'collisionBox', 'collisionSphere', 'collisionCylinder', 'collisionMesh'}, ...
                               {'scalar'}, ...
                               'addCollision', 'collisionObj');
        end

        function validateTransform(tform)
        %validateTransform Validates that the local frame is a homogeneous transform
            validateattributes(tform, ...
                               {'single','double'}, ...
                               {'nonempty', 'nonnan', 'finite', 'size',[4, 4]}, ...
                               'addCollision', 'tform');
        end

        function throwInvalidMeshError(errorCode, geometryFile)
        %throwInvalidMeshError Throws an error based on addition of a collision mesh
            prefix = 'robotics:robotmanip:rigidbody:';
            switch(errorCode)
                case 0 %No CAD read error
                case 1 %Invalid file extension
                    coder.internal.error([prefix, 'InvalidMeshFileExtension'], geometryFile, 'Collision');
                case 2 %File cannot be opened
                    coder.internal.error(...
                        [prefix 'MeshFileNotOpened'], geometryFile, 'Collision');
                case 3 %Mesh file is empty
                    coder.internal.error([prefix 'EmptyMeshFile'], geometryFile, 'Collision');
                case 4 %Invalid STL file
                    coder.internal.error([prefix 'InvalidSTLFileFormat'], geometryFile, 'Collision');
                case -2 %Invalid CAD file
                    coder.internal.error('robotics:robothoops:hoopsconvert:CorruptedInputCADFile', geometryFile);
                case -1 %Unsuccessful read
                    coder.internal.error('robotics:robothoops:hoopsconvert:CannotSuccessfullyReadInputCADFile', geometryFile);
                otherwise
                    %For all other cases:
                    coder.internal.error(...
                        "MATLAB:polyfun:stlFailedToRead", geometryFile);
            end
        end

    end
end
