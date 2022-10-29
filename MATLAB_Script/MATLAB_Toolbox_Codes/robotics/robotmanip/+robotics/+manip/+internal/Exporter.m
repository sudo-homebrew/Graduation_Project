classdef Exporter < robotics.manip.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%EXPORTER Internal class for exporting from rigidBodyTree to URDFShared

%   Copyright 2018-2022 The MathWorks, Inc.

    properties(Constant)
        %RobotBuildDataDir Unix style path during build
        %   The build for robot MAT files happens on Linux, hence, the
        %   build data directory for the robots is Unix based
        RobotBuildDataDir = 'robotmanip/data';

        %RobotShippedDataDir
        RobotShippedDataDir = fullfile('toolbox', 'robotics', 'robotmanip', 'exampleRobots');
    end

    methods (Static)
        function [model, translationData] = exportRobot(tree)
        %exportRobot Export matlabshared.multibody.internal.urdf.Model from rigidBodyTree
        %   This outputs a model and a struct, translationData,
        %   containing information on the translation between RBT and
        %   Model that could not be stored in the model output. Note
        %   that the translationData presently contains
        %   homeConfiguration, which is also contained in the joint
        %   homePosition.

        %Make a copy of the tree and convert it to correct format
        %(column) to ensure that all configurations extracted have the
        %correct format for use with downstream functions. Since the
        %tree is a handle class object, it is necessary to convert the
        %data type back at the end of the function so that the tree the
        %user sees (that was passed as input) does not change.
            inputDataFormat = tree.DataFormat;
            tree.DataFormat = 'column';

            model = matlabshared.multibody.internal.urdf.Model;
            model.Name = 'modelFromTree';
            model.RootLink = tree.BaseName;
            model.Gravity = tree.Gravity;

            %Use cell arrays for better readability in loop
            linkArray = cell(tree.NumBodies+1,1);
            jointArray = cell(tree.NumBodies,1);

            %Populate link associated with the base
            linkArray{1} = robotics.manip.internal.Exporter.createURDFLink(tree.Base);
            model.Links(tree.BaseName) = linkArray{1};
            
            %Initialize warning outputs
            hasNonShippedMeshes = false;
            unfoundMeshPaths = {};            

            %Populate links and joints associated with bodies in tree
            for i = 1:tree.NumBodies
                body = tree.Bodies{i};
                joint = body.Joint;
                [linkArray{i+1}, inclNonShippedMeshes, newUnfoundMeshPaths] = robotics.manip.internal.Exporter.createURDFLink(body);
                jointArray{i} = robotics.manip.internal.Exporter.populateURDFJoint(joint, body.Name, body.Parent.Name);

                model.Links(body.Name) = linkArray{i+1};
                model.Joints(joint.Name) = jointArray{i};
                
                %Update warning criteria
                hasNonShippedMeshes = hasNonShippedMeshes || inclNonShippedMeshes;
                unfoundMeshPaths = [unfoundMeshPaths newUnfoundMeshPaths]; %#ok<AGROW> 
            end

            %Include information that is not stored in URDFShared
            translationData = struct();
            translationData.HomeConfiguration = tree.homeConfiguration;

            %Convert the tree back to the correct data format
            tree.DataFormat = inputDataFormat;
            
            %Throw warnings based on the output criteria
            if hasNonShippedMeshes
                robotics.manip.internal.warning('urdfimporter:CustomRobotMeshes');
            elseif ~isempty(unfoundMeshPaths)
                robotics.manip.internal.warning('urdfimporter:SourceMeshesNotFound', sprintf('%s\n', unfoundMeshPaths{:}));
            end
        end
    end

    methods (Static, Access=public)
        function [urdfLink, hasNonShippedMeshes, unfoundMeshPaths] = createURDFLink(body)
        %createURDFLink Create an URDFShared link from a source rigid body

        %Initialize outputs
        urdfLink = matlabshared.multibody.internal.urdf.Link(body.Name);
        hasNonShippedMeshes = false; %#ok<NASGU> 
        unfoundMeshPaths = {}; %#ok<NASGU> 

        %As long as the body is not the base, populate joint names
        if (body.BodyInternal.ParentIndex > -1)
            urdfLink.ParentJoint = body.Joint.Name;
            isBase = false;
        else
            isBase = true;
        end

        if ~isempty(body.Children)
            urdfLink.ChildJoints = cellfun(@(x)(x.Joint.Name),body.Children,'UniformOutput',false);
        end

        if ~isBase
            % The base has no inertial attributes
            robotics.manip.internal.Exporter.setURDFLinkInertial(urdfLink, body);
        end

        % Assign visual and collision meshes
        [hasNonShippedMeshes, unfoundMeshPaths] = robotics.manip.internal.Exporter.setURDFLinkVisual(urdfLink, body);
        robotics.manip.internal.Exporter.setURDFLinkCollision(urdfLink, body);

        end

        function setURDFLinkInertial(link, body)
        %setURDFLinkInertial Set URDFShared inertial properties given a source rigid body

            link.Inertial = matlabshared.multibody.internal.urdf.Inertial;
            link.Inertial.Mass = body.Mass;

            %The Origin xyz and rpy have to be set directly in the
            %constructor; if that is not done, and Origin is declared as an
            %object with no inputs, it will not create a new object between
            %calls and each subsequent assignment to Origin.xyz and
            %Origin.rpy will reference the first instance of the origin.
            link.Inertial.Origin = matlabshared.multibody.internal.urdf.Origin(body.CenterOfMass, [0 0 0]);

            %The RST Inertia is defined at body frame origin, whereas in
            %URDF, the inertial frame collocates with the COM. However, in
            %this direction (from RST to URDF), there is no need to assume
            %any rotation of the frame, just translation:
            R = eye(3);

            %Shift inertia frame to CoM & populate URDF Inertial from result
            Io = body.BodyInternal.InertiaInternal;
            sp = robotics.manip.internal.skew(body.CenterOfMass);

            %Note that since R = I (see above), it's usage below is really
            %just a formality to preserve the generality of the formula.
            Icm = R*Io*R' - body.Mass*(sp*sp');

            link.Inertial.Inertia = matlabshared.multibody.internal.urdf.Inertia;
            link.Inertial.Inertia.Ixx = Icm(1,1);
            link.Inertial.Inertia.Iyy = Icm(2,2);
            link.Inertial.Inertia.Izz = Icm(3,3);
            link.Inertial.Inertia.Ixy = Icm(1,2);
            link.Inertial.Inertia.Ixz = Icm(1,3);
            link.Inertial.Inertia.Iyz = Icm(2,3);
        end
        
        function [filename, hasNonshippedMeshes] = validateMeshFile(inputFileName)
            %validateMeshFile Check if the source mesh file exists 
            %   This function accepts a source file and checks whether it
            %   exists. If the file references a build directory, it checks
            %   whether the file instead exists in a shipped directory and
            %   updates the file path accordingly. If the file cannot be
            %   found anywhere, filename returns empty. Additionally, two
            %   error flags are returned. First, if the file cannot be
            %   found because it lies in a build directory and there is no
            %   corresponding shipped mesh, hasNonShippedMeshes will be
            %   TRUE.
            
            hasNonshippedMeshes = false;
            
            % Check if the specified file exists. If it does, return this
            % file; no errors are needed.
            if isfile(inputFileName)
                filename = inputFileName;
                return;
            end
            
            % If the file doesn't exist, try to find it in the shipping
            % directories
            [filename, isBuildDirectoryFile] = robotics.manip.internal.Exporter.findInShippedDataDir(inputFileName);
            
            if ~isempty(filename)
                % If the file was found in a shipping directory, the issue
                % is now resolved
                return;
            elseif isBuildDirectoryFile && isempty(filename)
                % In this case the mesh file doesn't exist because the robot was
                % created using loadrobot, and the original mesh file isn't
                % shipping in RST, but it may be installed in the Robot
                % Library Data Support Package

                %Check if the Robot Library Data SPKG is installed and if the file can be found there. 
                filename = robotics.manip.internal.Exporter.findInSPKGDir(inputFileName);

                if ~isempty(filename)
                    return;
                else
                    hasNonshippedMeshes = true;
                end
            else
                % If the filename is empty but the mesh isn't a build
                % directory file, then it's just a custom mesh that
                % couldn't be found
            end
        end

        function [filename, isBuildDirectoryFile] = findInShippedDataDir(filename)
        %findInShippedDataDir Static helper that tries to find the filename in the RobotShippedDataDir
        %   The helper looks for the file in the RobotShippedDataDir. The
        %   filename is assumed to be an absolute path and should have
        %   contents in the RobotBuildDataDir directory which contains the
        %   robot descriptions and meshes used to create robot MAT files
        %   for loadrobot. When the mesh is found to be in a build
        %   directory, the flag isBuildDirectoryFile is set to TRUE.
        
            robotBuildDataDirName =...
                robotics.manip.internal.Exporter.RobotBuildDataDir;

            % Find the substring robotBuildDataDirName in the filename
            robotBuildDirPos = strfind(filename, robotBuildDataDirName);
            isBuildDirectoryFile = ~isempty(robotBuildDirPos);

            % Populate the robot description directory isolated from the
            % the build data directory. This will be searched for in the
            % shipped data directory
            robotDescriptionDir =...
                filename(robotBuildDirPos+length(robotBuildDataDirName):end);
            potentialFile = fullfile(matlabroot, ...
                                     robotics.manip.internal.Exporter.RobotShippedDataDir, ...
                                     robotDescriptionDir);

            % If potentialFile is a file and exists in the specified
            % path, then we have found the file in the shipped directory.
            if(isfile(potentialFile))
                filename = potentialFile;
            else
                filename = '';
            end
        end

        function filename = findInSPKGDir(filename)
        %findInShippedDataDir Static helper that tries to find the filename in the installed directory of the Robot Library Data SPKG
        %   The helper checks if the support package is installed. If it is
        %   installed it then looks for the file in the appropriate path in
        %   the support package.
        
        %Check if the SPKG is installed
            spkgPath = fileparts(which('robotlibraryspkgdirectory.txt'));
            isMeshSPKGInstalled = ~isempty(spkgPath);

            if ~isMeshSPKGInstalled
                filename = '';
                return;
            end

            robotBuildDataDirName =...
                robotics.manip.internal.Exporter.RobotBuildDataDir;

            % Populate the robot description directory isolated from the
            % the build data directory. This will be searched for in the
            % SPKG directory
            robotDescriptionDir =...
                extractAfter(filename, robotBuildDataDirName);
            [path, filename,ext] = fileparts(robotDescriptionDir);

            %If the file is a DAE, then use the corresponding STL that also
            %ships with the SPKG. This is because SM doesn't support
            %reading DAE files.
            if strcmpi(ext, '.dae')
                robotDescriptionDir = fullfile(path, filename, 'stl');
            end
            potentialFile = fullfile(spkgPath, ...
                                     'Robots', ...
                                     robotDescriptionDir);

            % If potentialFile is a file and exists in the specified
            % path, then we have found the file in the SPKG directory.
            if(isfile(potentialFile))
                filename = potentialFile;
            else
                filename = '';
            end
        end

        function [hasNonShippedMeshes, unfoundMeshPaths] = setURDFLinkVisual(link, body)
            %setURDFLinkVisual Assign the URDF link's visual properties

            %Determine source and use it to set URDF parameters
            visuals = body.BodyInternal.VisualsInternal;
            
            %Initialize output
            hasNonShippedMeshes = false;
            unfoundMeshPaths = {};

            %Set to nonempty if there are no visuals (otherwise, leave
            %empty)
            if ~isempty(visuals)
                link.Visual = matlabshared.multibody.internal.urdf.Visual;
            end

            % Start an index and increment it only when the visual is valid
            validVisualsIdx = 0;
            for k = 1:length(visuals)
                rbgeo = visuals{k};
                sourceType =  validatestring(rbgeo.SourceData{1}, {'Mesh', 'Box', 'Cylinder', 'Sphere'});
                sourceParam = rbgeo.SourceData{2};

                switch sourceType
                    case 'Mesh'
                        % Get the mesh path. The path stored by the rigid
                        % body tree will be the absolute path to the STLs,
                        % as referenced by the rigid body tree when it was
                        % first created.
                        [filename, isNonShippedBuiltMesh] = robotics.manip.internal.Exporter.validateMeshFile(sourceParam);
                        hasNonShippedMeshes = hasNonShippedMeshes || isNonShippedBuiltMesh;
                        if isempty(filename) && ~isNonShippedBuiltMesh
                            % If the filename is empty and it isn't because
                            % it's a robot that includes mesh files sourced
                            % in build directories that aren't shipped,
                            % then the file just cannot be found. Add to a
                            % list to be used in a downstream error.
                            unfoundMeshPaths = [unfoundMeshPaths sourceParam]; %#ok<AGROW>
                        end

                        % If the file name has been found, assign it as a mesh,
                        % otherwise, ensure that the visual is not counted as
                        % valid
                        if ~isempty(filename)
                            validVisualsIdx = validVisualsIdx + 1;
                            link.Visual(validVisualsIdx).Geometry = matlabshared.multibody.internal.urdf.Mesh(filename);
                            link.Visual(validVisualsIdx).Geometry.Scale = rbgeo.Scale;
                        end
                    case 'Box'
                        validVisualsIdx = validVisualsIdx + 1;
                        link.Visual(validVisualsIdx).Geometry = matlabshared.multibody.internal.urdf.Box(sourceParam);
                    case 'Cylinder'
                        validVisualsIdx = validVisualsIdx + 1;
                        link.Visual(validVisualsIdx).Geometry = matlabshared.multibody.internal.urdf.Cylinder(sourceParam(1), sourceParam(2));
                    case 'Sphere'
                        validVisualsIdx = validVisualsIdx + 1;
                        link.Visual(validVisualsIdx).Geometry = matlabshared.multibody.internal.urdf.Sphere(sourceParam);
                end

                if validVisualsIdx > 0
                    % As long as the link visual is valid, update its
                    % shared properties, such as material, color, and pose
                    link.Visual(validVisualsIdx).Material = matlabshared.multibody.internal.urdf.Material('');
                    link.Visual(validVisualsIdx).Material.Color = matlabshared.multibody.internal.urdf.Color(visuals{validVisualsIdx}.Color);
    
                    Tform = visuals{validVisualsIdx}.Tform;
                    ypr = rotm2eul(Tform(1:3,1:3), 'ZYX');
                    rpy = [ypr(3) ypr(2) ypr(1)];
                    xyz = Tform(1:3,4)';
                    link.Visual(validVisualsIdx).Origin = matlabshared.multibody.internal.urdf.Origin(xyz, rpy);
                end
            end
            
            % If none of the visuals are valid, remove the visual element
            if validVisualsIdx == 0
                link.Visual = matlabshared.multibody.internal.urdf.Visual.empty;
            end
        end

        function setURDFLinkCollision(link, body)
        %setURDFLinkCollision Assigns the Collision element of the link
        %   The function will assign the Collision element of the link based
        %   on the tag of the collision that exists on the rigidBody body.
        %   Each tag has the dimensions associated with the geometry given
        %   it's type. For instance, a "Box" type collision of size [1, 2,
        %   3] will have the following tag: "Box Size [1, 2, 3]". The
        %   Collision data is populated by parsing this tag.

            Collisions = body.BodyInternal.CollisionsInternal;

            % If there is collision data, create a Collision element
            if(Collisions.Size)
                link.Collision = matlabshared.multibody.internal.urdf.Collision;
            end

            tforms = Collisions.getTforms();

            %For every collision geometry in the rigidBody, based on its tag,
            %create a Collision element in the link
            for k = 1 : Collisions.Size
                tag = Collisions.Tags{k};
                if(contains(tag, "Box"))
                    sourceParam = sscanf(tag, 'Box Size [%f %f %f]');
                    link.Collision(k).Geometry = ...
                        matlabshared.multibody.internal.urdf.Box(sourceParam(:)');
                elseif(contains(tag, "Cylinder"))
                    sourceParam = sscanf(tag, 'Cylinder Radius %f Length %f');
                    link.Collision(k).Geometry = ...
                        matlabshared.multibody.internal.urdf.Cylinder(sourceParam(1), sourceParam(2));
                elseif(contains(tag, "Sphere"))
                    sourceParam = sscanf(tag, 'Sphere Radius %f');
                    link.Collision(k).Geometry = ...
                        matlabshared.multibody.internal.urdf.Sphere(sourceParam);
                elseif(contains(tag, 'Mesh'))
                    % Currently the shared interface doesn't support
                    % parsing of meshes with vertices, but meshes can be
                    % passed using source files

                    pat = caseInsensitivePattern("Mesh" + whitespacePattern + "Filename" + whitespacePattern);
                    sourceParam = extractAfter(tag, pat);

                    link.Collision(k).Geometry = matlabshared.multibody.internal.urdf.Mesh(sourceParam);
                    link.Collision(k).Geometry.Scale = [1 1 1];
                end

                % Obtain the local pose of the Collision element with respect to
                % the link
                localPose = tforms{k};

                % Populate the Collision element's frame data with respect to
                % the link
                ypr = rotm2eul(localPose(1:3,1:3), 'ZYX');
                rpy = [ypr(3) ypr(2) ypr(1)];
                xyz = localPose(1:3,4)';
                link.Collision(k).Origin = matlabshared.multibody.internal.urdf.Origin(xyz, rpy);
            end
        end

        function urdfJoint = populateURDFJoint(joint, bodyName, parentBodyName)
        %populateURDFJoint Create an URDFShared joint from a source rigid body joint

        %Initialize joint
            urdfJoint = matlabshared.multibody.internal.urdf.Joint(joint.Name, 'RBT');
            urdfJoint.ParentLink = parentBodyName;
            urdfJoint.ChildLink = bodyName;

            %Set joint type, axis, limits, and home position
            switch joint.Type
              case 'fixed'
                urdfJoint.Type = 'fixed';
              case 'revolute'
                urdfJoint.Type = 'revolute';
                urdfJoint.Axis = matlabshared.multibody.internal.urdf.Axis(joint.JointAxis);
                urdfJoint.Limit = matlabshared.multibody.internal.urdf.Limit;
                urdfJoint.Limit.Lower = joint.PositionLimits(1);
                urdfJoint.Limit.Upper = joint.PositionLimits(2);
                urdfJoint.HomePosition = joint.HomePosition;
              case 'prismatic'
                urdfJoint.Type = 'prismatic';
                urdfJoint.Axis = matlabshared.multibody.internal.urdf.Axis(joint.JointAxis);
                urdfJoint.Limit = matlabshared.multibody.internal.urdf.Limit;
                urdfJoint.Limit.Lower = joint.PositionLimits(1);
                urdfJoint.Limit.Upper = joint.PositionLimits(2);
                urdfJoint.HomePosition = joint.HomePosition;
            end

            %Get joint transform and convert it to URDF Joint Origin format
            if any(any(joint.ChildToJointTransform ~= eye(4)))
                robotics.manip.internal.error('urdfimporter:ChildToJointTransform');
            end
            tf = joint.JointToParentTransform;
            ypr = rotm2eul(tf(1:3,1:3),'ZYX');
            xyz = tf(1:3,4)';
            rpy = fliplr(ypr);
            urdfJoint.Origin = matlabshared.multibody.internal.urdf.Origin(xyz, rpy);
        end
    end
end
