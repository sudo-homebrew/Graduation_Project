classdef SDFImporter < robotics.manip.internal.RobotDataImporter

%SDFImporter Create a SDF importer object.
%   SDF is the Simulation Description Format. The importer object
%   provides an importrobot method that can create a
%   rigidBodyTree object by parsing a user-supplied SDF file
%   or string.
%
%   IMPORTER = robotics.SDFIMPORTER() returns a SDF importer object
%   IMPORTER with default property settings
%
%   IMPORTER = robotics.SDFIMPORTER('PropertyName', PropertyValue, ...)
%   sets each specified property of the SDF importer object IMPORTER
%   to the specified value upon construction.
%
%   SDFIMPORTER properties
%      ParseInertial    - Indicates whether to parse inertial parameters
%      ParseVisual      - Indicates whether to parse visual geometry information
%      ParseCollision   - Indicates whether to parse collision geometry information
%      DataFormat       - Defines data format for the RigidBodyTree object
%      MaxNumBodies     - Maximum number of bodies on the RigidBodyTree object in codegen
%
%   SDFIMPORTER methods
%      importrobot     - Import a RigidBodyTree model from SDF
%
%   Example
%
%      % Create a SDF importer
%      importer = robotics.SDFImporter();
%
%      % Do not parse Inertial elements in SDF
%      importer.ParseInertial = false
%
%      % Do not parse visual geometry information in SDF
%      importer.ParseVisual = false
%
%      % Do not parse collision geometry information in SDF
%      importer.ParseCollision = true
%
%      % Set the data format to 'row'
%      importer.DataFormat = 'row'
%
%   See also importrobot, robotics.URDFImporter, robotics.SMImporter

%   Copyright 2021 The MathWorks, Inc.

    methods
        function obj = SDFImporter(varargin)
        %SDFImporter Constructor
            robotDataObj = robotics.manip.internal.RobotDataImporter(varargin{:});

            obj.ParseInertial = robotDataObj.ParseInertial;
            obj.ParseVisual = robotDataObj.ParseVisual;
            obj.ParseCollision = robotDataObj.ParseCollision;
            obj.DataFormat = robotDataObj.DataFormat;
            obj.MaxNumBodies = robotDataObj.MaxNumBodies;
        end

        function robot = importrobot(obj, sdfInput, varargin)
        %importrobot Import a RigidBodyTree model by parsing SDF
        %   ROBOT = IMPORTROBOT(IMPORTER, SDFINPUT) returns a
        %   rigidBodyTree object by parsing the Simulation Description
        %   Format (SDF) model description SDFINPUT.
        %   SDFINPUT can either be a file or a string.
        %
        %   ROBOT = IMPORTROBOT(IMPORTER, ___, Name, Value) provides
        %   additional options specified by Name-Value pair arguments.
        %   Available parameter name:
        %
        %
        %      'MeshPath'     - Search directories for mesh files,
        %                       specified as a char vector or string,
        %                       or a cell array of char vectors or strings.
        %
        %      'DataFormat'   - Data format for the RigidBodyTree object, specified as
        %                       "struct", "row", or "column". To use dynamics
        %                       methods, you must use either "row" or "column". The
        %                       default data format is "struct".
        %
        %      'SDFModel'     - Select model name for SDF input containing multiple
        %                       models.
        %
        %    Assume the raw mesh path retrieved from SDF is denoted as
        %    FN. If FN contains SDF-style "model:\\<pkg_name>", that
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
        %       - The folder that has the SDF file
        %       - One level above the SDF folder
        %    3) If 1) and 2) both return nothing, the parser further
        %    strips FN to only contain the file name and extension, and
        %    appends it directly after each provided MeshPath.
        %
        %    If the mesh file is still not found, the parser ignores the
        %    mesh file path and returns a rigidBody object without
        %    visuals.
        %
        %   Example:
        %      % Create a SDF importer
        %      importer = robotics.SDFImporter();
        %
        %      % Do not parse Inertial elements in SDF
        %      importer.ParseInertial = false
        %
        %      % Import from a minimalistic SDF string
        %      s = '<?xml version="1.0" ?><sdf version="1.6"><model name="min"><link name="L0"/></model></sdf>';
        %      mini = importrobot(importer, s)

        % add 'SourceData' type if varargin does not contain
            if ~any(strcmp([varargin{:}],'SourceData'))
                varargin{end+1} = 'SourceData';
                varargin{end+1} = 'SDF';
            end
            robot = obj.importrobotShared(sdfInput, varargin{:});
        end
    end

end
