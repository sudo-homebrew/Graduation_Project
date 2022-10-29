classdef RobotDataImporter < robotics.manip.internal.Importer ...
        & matlab.mixin.Copyable
    %Base class for URDF and SDF importer object

    %   Copyright 2021 The MathWorks, Inc.

    properties
        %ParseInertial Indicates whether to parse inertial parameters
        %   This is a Boolean property. Inertial parameters include body
        %   mass, center of mass and inertia.
        %   tensor.
        %
        %   Default: true
        ParseInertial = true;

        %ParseVisual Indicates whether to parse rigid body visual information
        %   This is a Boolean property. Visual information includes
        %   the shape definition and the its poses relative
        %   to the body frame.
        %
        %   Default: true
        ParseVisual = true;

        %ParseCollision Indicates whether to parse rigid body collision information
        %   This is a Boolean property. Collision information includes the shape
        %   definition and its poses relative to the body frame.
        %
        %   Default: true
        ParseCollision = true;

        %DataFormat Defines data format for the RigidBodyTree object
        %   DataFormat is specified as "struct", "row", or "column". To use
        %   dynamics methods, you must use either "row" or "column".
        %
        %   Default: 'struct'
        DataFormat  = 'struct'

        %MaxNumBodies Maximum number of bodies on the RigidBodyTree object
        %   This property is only required during codegen.
        %
        %   Default: 0
        MaxNumBodies = 0
    end

    methods
        function obj = RobotDataImporter(varargin)
        %URDFImporter Constructor
            parser = inputParser;
            parser.addParameter('ParseInertial', true);
            parser.addParameter('ParseVisual', true);
            parser.addParameter('ParseCollision', true);
            parser.addParameter('DataFormat', 'struct');
            parser.addParameter('MaxNumBodies', 0);
            parser.parse(varargin{:});

            obj.ParseInertial = parser.Results.ParseInertial;
            obj.ParseVisual = parser.Results.ParseVisual;
            obj.ParseCollision = parser.Results.ParseCollision;
            obj.DataFormat = parser.Results.DataFormat;
            obj.MaxNumBodies = parser.Results.MaxNumBodies;
        end

        function set.ParseInertial(obj, b)
        %set.ParseInertial
            validateattributes(b, {'logical', 'numeric'}, {'nonempty','scalar'}, ...
                               'RobotDataImporter','ParseInertial');
            obj.ParseInertial = logical(b);
        end

        function set.ParseCollision(obj, b)
        %set.ParseCollision
            validateattributes(b, {'logical', 'numeric'}, {'nonempty','scalar'}, ...
                               'RobotDataImporter','ParseCollision');
            obj.ParseCollision = logical(b);
        end

        function set.ParseVisual(obj, b)
        %set.ParseVisual
            validateattributes(b, {'logical', 'numeric'}, {'nonempty','scalar'}, ...
                               'RobotDataImporter','ParseVisual');
            obj.ParseVisual = logical(b);
        end

        function set.DataFormat(obj, dataformat)
        %set.DataFormat
            validateattributes(dataformat,{'char','string'},{'nonempty','scalartext'},'robotics.RobotDataImporter','DataFormat');
            obj.DataFormat = validatestring(dataformat, {'struct', 'row', 'column'}, 'robotics.RobotDataImporter', 'DataFormat');
        end

    end

    methods (Access = ?robotics.manip.internal.InternalAccess)

        function robot = importrobotShared(obj, fileInputString, varargin)
        %importrobotShared shared method for URDF and SDF importer

        % validate string and convert to chars
            validFileInput = robotics.internal.validation.validateString(fileInputString, false, 'importrobot', 'textInput');

            [meshPath, sourceData, sdfModelName] = robotics.manip.internal.RobotDataImporter.validateMeshPath(varargin{:});

            % validFileInput is always string but can be 'file' string (
            % "abc.urdf" ) or 'URDF'/'SDF' string input ( "<robot name=" ).
            if contains(validFileInput, '</') || contains(validFileInput, '/>')
                source = 'string';
                pathToDataFile = '';
            else
                source = 'file';
                % get URDF/SDF file path
                pathToDataFile = fileparts(which(validFileInput)); % check if the file is on the MATLAB path
                if isempty(pathToDataFile)
                    if isRelativePath(validFileInput)
                        pathToDataFile = fileparts(fullfile(pwd, validFileInput));
                    else
                        pathToDataFile = fileparts(validFileInput);
                    end
                end
            end

            if strcmp(sourceData,'SDF')
                % validate 'SDFModel' name input
                validateSDFModelInputs(source, validFileInput, sdfModelName);
            end

            model = matlabshared.multibody.internal.urdf.Model(...
                validFileInput, 'ParseInertial', obj.ParseInertial,...
                'ParseVisual', obj.ParseVisual,...
                'ParseCollision', obj.ParseCollision,...
                'Source', source,'SourceData',sourceData,...
                'SDFModel',sdfModelName);

            robot = obj.importURDFShared(model, meshPath, pathToDataFile, sourceData, validFileInput);

            %Set the rigidBodyTree DataFormat
            robot.DataFormat = obj.DataFormat;

        end
    end

    methods (Static, Access = {?robotics.manip.internal.InternalAccess, ?robotics.SDFImporter, ?robotics.URDFImporter})
        function [meshPath,sourceData, inputSDFModelName] = validateMeshPath(varargin)
        %validateMeshPath validate MeshPath, source type and SDF model name input

        % Meshpath can be a cell array of character and/or string inputs
            [varargin{:}] = convertStringsToChars(varargin{:});
            if nargin > 1 && isa(varargin{2},'cell')
                %If the input is a cell of string inputs,
                %convertStringsToChars must be called on the components,
                %not the cell as a whole
                meshCell = varargin{2};
                cellData = cell(size(varargin{2}));
                [cellData{:}] = convertStringsToChars(meshCell{:});
                varargin{2} = cellData;
            end

            parser = inputParser;
            parser.addParameter('MeshPath', {});
            parser.addParameter('SourceData', '');
            parser.addParameter('SDFModel', '');
            parser.parse(varargin{:});

            meshPath = parser.Results.MeshPath;
            if ~ischar(meshPath) && ~iscellstr(meshPath)
                error(message('robotics:robotmanip:urdfimporter:MustBeCharOrCharCellArray', 'MeshPath'));
            end
            meshPath = cellstr(meshPath);

            % get 'sourceData', 'SDF' or 'URDF'
            sourceData = parser.Results.SourceData;
            if isempty(sourceData)
                % defualt sourceData is 'URDF'
                sourceData = 'URDF';
            end

            % get SDF model name
            inputSDFModelName = parser.Results.SDFModel;
            if strcmp(sourceData,'SDF')
                if(~isempty(inputSDFModelName))
                    if(iscellstr(inputSDFModelName) || iscell(inputSDFModelName))
                        inputSDFModelName = inputSDFModelName{:};
                    end
                    % validate input SDF model name
                    inputSDFModelName = convertStringsToChars(inputSDFModelName);
                    validateattributes(inputSDFModelName, {'char','string'}, {'scalartext','nonempty'}, 'URDFImporter', 'SDFModel');
                end
            end

        end


        function meshFilePath = findMeshFilePath(pathStr, urdfDir, suggestedMeshDirs, varargin)
        %findMeshFilePath returns Mesh file path based input type
        %   VARARGIN holds the SOURCEDATA which is either URDF or SDF and the 
        %   URDF/SDF filename FILEINPUT provided by the user.

        % search logic:
        % 1) Preprocess raw pathStr from URDF/SDF and get:
        %    a) mesh path without the ROS package header/SDF model uri, and
        %    b) only the barebone mesh filename with extension
        % 2) Substitute the extensions in 1a) and 1b) with all possible
        %    upper/lower case combinations of "stl"
        % 3) For each possible path from 2), try the following in order
        %    - if it's an absolute path, try to open it directly (i.e.
        %      the "open file" test).
        %    - if it's a relative path and it's derived from 1b),
        %      combine it with the user-specified directory, then
        %      perform the "open file" test.
        %    - if it's a relative path and it's derived from 1a), combine
        %      it with several different path prefixes, respectively,
        %      and perform the "open file" test for each one.
        %      Possible prefixes are: user-specified directory, current
        %      folder, urdf file folder, one level up of the urdf
        %      folder, and no prefix at all.
        % 4) The search will terminate with the first successful "open
        %    file" test, or until all possibilities are tried.

        % preprocessing
            fnRaw = strrep(pathStr,'/',filesep);
            fnRaw = strrep(fnRaw,'\',filesep);

            [pth, nm, ext]= fileparts(fnRaw);

            meshFilePath = '';
            if isempty(ext)
                return;
            end

            % Assemble the path
            fnBase = fullfile(pth, nm);
            fnNoDirBase = nm;

            % default SourceData is 'URDF'
            if ~isempty(varargin)
                sourceData = varargin{1};
                if nargin == 5
                    %Description file (URDF/SDF) name
                    fileInput = varargin{2};
                else
                    fileInput = '';
                end
            else
                sourceData = 'URDF';
                fileInput = '';
            end

            % Remove ROS package header or SDF model uri, if it's there. Have to
            % search for both "package://" or "model://" and "package:/"or
            % "model:/" because fullfile pre-processes items it perceives
            % as URLs and reduces the others to a single /.
            switch sourceData
              case 'URDF'
                pkgHint1 = ['package:',filesep,filesep];
                pkgHint2 = ['package:',filesep];
              case 'SDF'
                pkgHint1 = ['model:',filesep,filesep];
                pkgHint2 = ['model:',filesep];
            end

            if contains(fnBase, pkgHint1)
                fnBase = strrep(fnBase, pkgHint1, '');
                [~, fnBase] = strtok(fnBase, filesep); % strip off the package/model name
                fnBase = regexprep(fnBase, ['^' filesep], '');
            elseif contains(fnBase, pkgHint2)
                fnBase = strrep(fnBase, pkgHint2, '');
                [~, fnBase] = strtok(fnBase, filesep); % strip off the package/model name
                fnBase = regexprep(fnBase, ['^' filesep], '');

            end

            % Get the path of the preprocessed file
            filePath = fileparts(fnBase);

            % If the file is an absolute path we try to find it using
            % all possible .stl suffixes
            if ~isRelativePath(filePath)

                % Check if the file exists. If it does, return the path
                % to the file.
                meshFilePath = fopenTestWithAllSuffixes(fnBase, ext);
                if ~isempty(meshFilePath)
                    return
                end

            else

                % If the file is not an absolute path, we first search the
                % user specified paths for files matching ONLY the name of
                % the file and not any relative path components it may
                % have.

                for suggestedDirIndex = 1:length(suggestedMeshDirs)
                    currentSearchDirectory = suggestedMeshDirs{suggestedDirIndex};

                    % Construct the file base to test using the search
                    % directory path and the name of the file.
                    fileBaseToTest = fullfile(currentSearchDirectory, fnNoDirBase);

                    % Check if the file exists. If it does, return the path
                    % to the file.
                    meshFilePath = fopenTestWithAllSuffixes(fileBaseToTest, ext);
                    if ~isempty(meshFilePath)
                        return
                    end

                end

                % We then search over all possible paths using the relative
                % path information. We give any user specified mesh paths a
                % higher priority in the search by placing it first in the
                % array of search directories.

                dirsToSearch = [suggestedMeshDirs{:}, {pwd, '', urdfDir, ...
                    fullfile(urdfDir,'..')}, getSPKGDirToSearch(urdfDir, fileInput)];               

                for directoryIndex = 1:length(dirsToSearch)
                    currentSearchDirectory = dirsToSearch{directoryIndex};

                    % Construct the file name with the search directory
                    % path and the filename including the relative path
                    fileBaseToTest = fullfile(currentSearchDirectory, fnBase);

                    % Check if the file exists. If it does, return the path
                    % to the file.
                    meshFilePath = fopenTestWithAllSuffixes(fileBaseToTest, ext);
                    if ~isempty(meshFilePath)
                        return
                    end
                end
            end
        end
    end
end

function validateSDFModelInputs(source, textInput, sdfModelName)
%validateSDFModelInputs validates SDF Model name input
%   SDF file can contain multiple models. The user should specify or select
%   single model name with 'SDFModel' name-value pair. The input SDFModel
%   should be provided for multiple models and it should be from valid
%   model names.

% Get all SDF model names from SDF file
    validSDFModelNames = robotics.manip.internal.sdfSupport.getSDFModelNames(textInput);
    if strcmp(source, 'file') && ~isempty(sdfModelName) &&...
            ~any(contains(validSDFModelNames,sdfModelName))
        % For a file containing, multiple SDF models, need to
        % validate modelname provided by user. Error out if input
        % 'SDFModel' name is not present in the SDF model name list
        error(message('robotics:robotmanip:urdfimporter:InvalidSDFModel',...
                      sdfModelName, textInput, join(validSDFModelNames,", ") ));
    end

    if isempty(sdfModelName) && numel(validSDFModelNames) > 1
        % If the model contains multiple models, then the user
        % must select one with 'SDFModel'. If the user did not
        % specify a model to select, then throw an error.
        error(message('robotics:robotmanip:urdfimporter:MultipleSDFModel',...
                      textInput, join(validSDFModelNames,", ") ));
    end
end


function filename = testWithFOpen(fn)
%testWithFOpen - Test for file existence using fopen
%   We use fopen because it can search the matlab path for files matching
%   the name we specify, and it also can return the full path to the file
%   by passing the file id.

    fid = fopen(fn, 'r');
    filename = '';

    if fid > 0
        filename = fopen(fid);
        fclose(fid);
    end

end


function isRel = isRelativePath(pth)
%isRelativePath

% absolute path examples:
% \\server_name\...
% \\?\UNC\server_name\...
% C:\...
% C:
% /home/...

    isRel = isempty(pth) || ( pth(1) ~= filesep && ~contains(pth, ':') );
end

function options = getSTLSuffixPossibles()
%getSTLSuffixPossibles
    options = {'.stl', '.STL', '.Stl', '.sTl', '.sTL', '.STl', '.StL', '.stL'};

end

function options = getDAESuffixPossibles()
%getDAESuffixPossibles
options = {'.dae', '.DAE', '.Dae', '.dAe', '.dAE', '.DAe', '.DaE', '.daE'};

end

function filePath = fopenTestWithAllSuffixes(fileBaseToTest, ext)
% fopenTestWithAllSuffixes - Test for file existence with all supported CAD extension possibilities using fopen
%   Test for file existence by iterating over all possible supported CAD suffixes,
%   appending them to fileBaseToTest, and then attempting to open the file
%   using fopen. Returns an empty char if no file is found

filePath = '';
allSuffixOptions = {getSTLSuffixPossibles, getDAESuffixPossibles};
extIdx = cellfun(@(x)(any(strcmpi(x,ext))), allSuffixOptions);

if ~any(extIdx)
    % If extension is not supported, try with all suffix options
    suffixOptions = [getSTLSuffixPossibles, getDAESuffixPossibles];
else
    % If extension is supported, try with all matching variants first, followed
    % by all the remaining suffix variants
    suffixOptions = [allSuffixOptions{extIdx}, allSuffixOptions{1:extIdx-1}, allSuffixOptions{extIdx+1:end}];
end

for suffixIndex = 1:length(suffixOptions)
    
    % Construct the full file name with the path and suffix
    suffix = suffixOptions{suffixIndex};
    fileNameToTest = [fileBaseToTest, suffix];
    
    % Test if the file exists. If it does, return the path
    % to the file.
    filePath = testWithFOpen(fileNameToTest);
    if ~isempty(filePath)
        return
    end
    
end

end

function folder = getRawDescFolderName(fileName)
%getRawDescFolderName Return the description folder that contains the filename URDF/SDF

    descFolderStruct = robotics.manip.internal.RobotList.RawDescriptionFolders;
    folderList = [descFolderStruct.Folder];
    descFilesList = {descFolderStruct.DescFiles};
    folder = "";
    for i=1:length(descFilesList)
        if any(strcmp(fileName,descFilesList{i}))
            folder = folderList(i);
            return;
        end
    end
end

function spkgfolderToSearch = getSPKGDirToSearch(urdfDir, descFileInput)
%getSPKGDirToSearch Path to Robot Library Data SPKG if it is installed

%RST directory containing Robot Library URDFs
toolboxURDFDir = fullfile('robotics', 'robotmanip', 'robotModels', 'roboturdf');

%Use the robotlibraryspkgdirectory.txt file installed through the support
%package to determine if it is installed or not
spkgPath = fileparts(which('robotlibraryspkgdirectory.txt'));
isMeshSPKGInstalled = ~isempty(spkgPath);

%Check if the URDF is from the Robot Library
isLibraryURDF = contains(urdfDir, toolboxURDFDir);

%Add the SPKG robot description folder path to the
%directories to search
if isMeshSPKGInstalled && isLibraryURDF && ~isempty(descFileInput)
    [~, fileName, ext] = fileparts(descFileInput);
    rawDescFolder = getRawDescFolderName([fileName, ext]);
    spkgfolderToSearch = {char(fullfile(spkgPath, 'Robots', rawDescFolder))};
else
    spkgfolderToSearch = {};
end
end
