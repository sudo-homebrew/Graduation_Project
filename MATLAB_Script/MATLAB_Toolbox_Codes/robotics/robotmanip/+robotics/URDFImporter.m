classdef URDFImporter < robotics.manip.internal.RobotDataImporter

%URDFIMPORTER Create a URDF importer object.
%   URDF is the Unified Robot Description Format. The importer object
%   provides an importrobot method that can create a
%   rigidBodyTree object by parsing a user-supplied URDF file
%   or string.
%
%   IMPORTER = robotics.URDFIMPORTER() returns a URDF importer object
%   IMPORTER with default property settings
%
%   IMPORTER = robotics.URDFIMPORTER('PropertyName', PropertyValue, ...)
%   sets each specified property of the URDF importer object IMPORTER
%   to the specified value upon construction.
%
%   URDFIMPORTER properties
%      ParseInertial    - Indicates whether to parse inertial parameters
%      ParseVisual      - Indicates whether to parse visual geometry information
%      ParseCollision   - Indicates whether to parse collision geometry information
%      DataFormat       - Defines data format for the RigidBodyTree object
%      MaxNumBodies     - Maximum number of bodies on the RigidBodyTree object in codegen
%
%   URDFIMPORTER methods
%      importrobot     - Import a RigidBodyTree model from URDF
%
%   Example
%
%      % Create a URDF importer
%      importer = robotics.URDFImporter();
%
%      % Do not parse Inertial elements in URDF
%      importer.ParseInertial = false
%
%      % Do not parse visual geometry information in URDF
%      importer.ParseVisual = false
%
%      % Do not parse collision geometry information in URDF
%      importer.ParseCollision = true
%
%      % Set the data format to 'row'
%      importer.DataFormat = 'row'
%
%      % Parse and generate rigidBodyTree object from a URDF
%      % file
%      robot = importrobot(importer, 'iiwa14.urdf');
%
%   See also importrobot, robotics.SDFImporter, robotics.SMImporter

%   Copyright 2016-2021 The MathWorks, Inc.

    methods
        function obj = URDFImporter(varargin)
        %URDFImporter Constructor
            robotDataObj = robotics.manip.internal.RobotDataImporter(varargin{:});

            obj.ParseInertial = robotDataObj.ParseInertial;
            obj.ParseVisual = robotDataObj.ParseVisual;
            obj.ParseCollision = robotDataObj.ParseCollision;
            obj.DataFormat = robotDataObj.DataFormat;
            obj.MaxNumBodies = robotDataObj.MaxNumBodies;
        end

        function robot = importrobot(obj, urdfInput, varargin)
        %importrobot Import a RigidBodyTree model by parsing URDF
        %   ROBOT = IMPORTROBOT(IMPORTER, URDFINPUT) returns a
        %   rigidBodyTree object by parsing the Unified Robot
        %   Description Format (URDF) robot description URDFINPUT.
        %   URDFINPUT can either be a file or a string.
        %
        %   ROBOT = IMPORTROBOT(IMPORTER, ___, Name, Value) provides
        %   additional options specified by Name-Value pair arguments.
        %   Available parameter name:
        %
        %      'MeshPath'       - Search directories for mesh files,
        %                         specified as a char vector or a cell
        %                         array of char vectors. This parameter
        %                         is only used if the ParseVisual
        %                         property of IMPORTER is true.
        %
        %    Assume the raw mesh path retrieved from URDF is denoted as
        %    FN. If FN contains ROS-style "package:\\<pkg_name>", that
        %    part of path is removed. Then the mesh file is searched in
        %    the following way:
        %    1) If FN is an absolute path, it is checked directly with
        %       no modification. If nothing is found, no further
        %       attempt is made.
        %    2) If FN is a relative path, the following folders to
        %    which FN might be relative to are tested in order:
        %       - User-specified MeshPath
        %       - Current directory
        %       - MATLAB path
        %       - The folder that has the URDF file
        %       - One level above the URDF folder
        %    3) If 1) and 2) both return nothing, the parser further
        %    strips FN to only contain the file name and extension, and
        %    appends it directly after each provided MeshPath.
        %
        %    If the mesh file is still not found, the parser ignores the
        %    mesh file path and returns a rigidBody object without
        %    visuals.
        %
        %   Example:
        %      % Create a URDF importer
        %      importer = robotics.URDFImporter();
        %
        %      % Do not parse Inertial elements in URDF
        %      importer.ParseInertial = false
        %
        %      % Import an LBR iiwa manipulator model from URDF file
        %      lbr = importrobot(importer, 'iiwa14.urdf')
        %      show(lbr)
        %
        %      % Import an LBR iiwa manipulator model from URDF, but also
        %      % specify search directories for the mesh files
        %      lbr = importrobot(importer, 'iiwa14.urdf', 'MeshPath', {'meshes', '../meshes'})
        %
        %      % Import from a minimalistic URDF string
        %      s = '<?xml version="1.0" ?><robot name="min"><link name="L0"/></robot>';
        %      mini = importrobot(importer, s)

        % add 'SourceData' type if varargin does not contain
            if ~any(strcmp([varargin{:}],'SourceData'))
                varargin{end+1} = 'SourceData';
                varargin{end+1} = 'URDF';
            end
            robot = obj.importrobotShared(urdfInput, varargin{:});

        end
    end

end
