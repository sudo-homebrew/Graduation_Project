classdef (Abstract) ROSProjectBuilder < handle
% This class is for internal use only. It may be removed in the future.

% ROSProjectBuilder is an abstract class for generating Catkin/Colcon
% projects. This base classes created from this abstract class are used by
% ProjectTool to build ROS modules

% Copyright 2019-2021 The MathWorks, Inc.

    properties (Abstract, Constant, Hidden)
        CMAKEMINVERSION
        FUNCTIONNAME
        CMAKETEMPLATE
        PKGXMLTEMPLATE
        VISIBILITYHTEMPLATE
        CUSTOMMSG_FOLDER_SUFFIX
        CUSTOMSRV_FOLDER_SUFFIX
    end

    properties (Constant, Hidden)
        PKGINFONAME = 'packageInfo.mat';
        MSVCVERSION = [20170000, 20190000];
        MSVCNAME = 'Microsoft';
    end

    % Properties
    properties (Hidden, SetAccess=protected)
        LocalPythonPath %Local Python for use with VENV
        ActivatePath    %Path to the activate command
        PyEnvDir        %VENV Dir
        CMakePath       %CMakePath returned through static method
        Parser          %Input parser that is used for value pairs
        MexInfo         %Info from mex.getCompilerConfigurations('C++')
        GenCodeOnly    = false; %Generate code only flag (to avoid cmake/python errors)
    end

    properties (SetAccess = protected)
        Packages         %PackageInfos for given packages
        RootDir          %Directory under which src/<package> is created
    end

    methods (Abstract, Access=protected)
        runBuildSystemCommand();

    end

    methods (Abstract, Static, Hidden)
        setupPythonAndCmakeTools();
        getCMAKEMinimumVersion();
        createDotPrinter();
        formatCommand();
        getCMakePath();
        getBuildCommand(rootFolder);
    end

    % Setters for the class
    methods
        function set.RootDir(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            %check for the folder
            if ~isfolder(value)
                error(message('ros:utilities:util:RootDirDoesNotExist',value));
            end
            h.RootDir = convertStringsToChars(value);
        end

        function set.Packages(h, value)
            value = value(~cellfun(@isempty,value)); %remove empty values
            validateattributes(value,{'cell'},{'nonempty'}); %check if value is a non empty cell array
            cellfun(@(x)validateattributes(x,{'ros.internal.PackageInfo'},{'nonempty'}),value);
            h.Packages = value;
        end
    end

    % Helpers
    methods (Hidden)
        function createInputParser(h)
        %createInputParser creates the inputParser and uses the static
        %method of PackageInfo to add its parameters as users can pass
        %them to base-class builder
            h.Parser = inputParser;
            h.Parser.FunctionName = h.FUNCTIONNAME;
            h.Parser.addRequired('rootDir',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            ros.internal.PackageInfo.addParamsToParser(h.Parser); %add required params of PackageInfo
            h.Parser.addParameter('GenCodeOnly',false,@(x)islogical(x));
            h.Parser.addParameter('forceRecreateVENV',false,@(x)validateattributes(x,{'logical'}));
        end

        function [status, result] = runCommand(h, varargin)
        %runBuildSystemCommand Use shell script to run a command for ROS
        %build system
            buildProgress = h.createTextProgressBar();
            buildProgress.printMessage(message('ros:utilities:util:BuildInProgress').getString());
            [status, result] = h.runBuildSystemCommand(varargin{:});
            if isequal(status,0)
                buildProgress.printMessage(message('ros:utilities:util:BuildSucceeded').getString());
            end
            %get all status files
            statusFiles = dir(fullfile(h.RootDir, 'log','build*','*','stdout_stderr.log'));
            [~,ind] = sort([statusFiles(:).datenum],'descend');
            if ~isempty(ind)
                theStatusFile = statusFiles(ind(1)); %the latest file
                result = sprintf('<a href="file:///%s">build log</a>',strrep(fullfile(theStatusFile.folder,theStatusFile.name),'\','/'));
            end
        end

        function idx = findPackage(h, packageName)
        %findPackage looks for a packageName in h.Packages
            idx = [];
            if ~isempty(h.Packages)
                matchingIdxs = cellfun(@(x)isequal(x.PackageName,packageName),h.Packages);
                idx = find(matchingIdxs);
            end
        end

        function pkgInfo = getPackageInfo(h, pkgName)
            validateattributes(pkgName,{'char','string'},{'nonempty'});
            pkgName = convertStringsToChars(pkgName);
            idx = h.findPackage(pkgName);
            if isempty(idx)
                error(message('ros:utilities:util:PackageNotFound', pkgName));
            end
            pkgInfo = h.Packages{idx};
        end

        function copyFiles(~, srcFiles, destDir)
        %local utility to copy srcFiles to a destDir
        %if the copy does not succeed, throws an error
            if ~isempty(srcFiles)
                for f = 1:numel(srcFiles)
                    [status, msg, msgid] = ...
                        copyfile(srcFiles{f}, destDir,'f');
                    if ~status
                        error(msgid, msg);
                    end
                end
            end
        end

        function chkAndCreateDir(~, pathToCreate)
        %local utility to check if the
            if ~isfolder(pathToCreate)
                [status, msg, msgid] = mkdir(pathToCreate);
                if ~status
                    error(msgid, msg);
                end
            end
        end

        function clnUp = createAndChangeToSrcDir(h)
        %create the src directory
            rootDir = fullfile(h.RootDir,'src');
            h.chkAndCreateDir(rootDir);
            curDir = cd (rootDir);
            clnUp = onCleanup(@()cd(curDir));
        end

        function makeDirStructure(h, pkgInfo, pkgDir, pkgSrc, pkgMsg, pkgSrv, pkgAction, MATLABFileDir, MsgClassDir)
        %create the directory structure for the package
            h.chkAndCreateDir(pkgDir);
            h.chkAndCreateDir(pkgSrc);
            pkgInc = fullfile(pkgDir,'include',pkgInfo.PackageName);
            h.chkAndCreateDir(pkgInc);
            if ~isempty(pkgInfo.MessageFiles)
                h.chkAndCreateDir(pkgMsg);
            end
            if ~isempty(pkgInfo.ServiceFiles)
                h.chkAndCreateDir(pkgSrv);
            end
            if ~isempty(pkgInfo.ActionFiles)
                h.chkAndCreateDir(pkgAction);
            end
            if ~isempty(pkgInfo.MATLABFiles)
                h.chkAndCreateDir(MATLABFileDir);
            end
            if ~isempty(pkgInfo.MsgClassFiles)
                h.chkAndCreateDir(MsgClassDir);
            end
        end

        function pkgInfoStruct = getAndUpdatePkgInfo(obj, pkgInfo)
        %convert PackageInfo into a struct that can be used for
        %generation using templates
        %Update the structure so it is a relative path
            lastWarnState = warning('OFF','MATLAB:structOnObject');
            clnup = onCleanup(@()warning(lastWarnState));
            pkgInfoStruct = struct(pkgInfo);
            clnup = []; %#ok<NASGU>
            for i = 1:numel(pkgInfoStruct.SourceFiles)
                [~,fname, fext] = fileparts(pkgInfoStruct.SourceFiles{i});
                pkgInfoStruct.SourceFiles{i} = ['src/',fname,fext];
            end
            for i = 1:numel(pkgInfoStruct.OtherFiles)
                [~,fname, fext] = fileparts(pkgInfoStruct.OtherFiles{i});
                pkgInfoStruct.OtherFiles{i} = [fname,fext];
            end
            for i = 1:numel(pkgInfoStruct.ImportedLibraries)
                [~,fname, fext] = fileparts(pkgInfoStruct.ImportedLibraries{i});
                pkgInfoStruct.ImportedLibraries{i} = ['src/',fname,fext];
            end
            for i = 1:numel(pkgInfoStruct.IncludeFiles)
                [~,fname, fext] = fileparts(pkgInfoStruct.IncludeFiles{i});
                pkgInfoStruct.IncludeFiles{i} = ['include/',pkgInfoStruct.PackageName,'/',fname,fext];
            end
            for i = 1:numel(pkgInfoStruct.LibSourceFiles)
                [~,fname, fext] = fileparts(pkgInfoStruct.LibSourceFiles{i});
                pkgInfoStruct.LibSourceFiles{i} = ['src/',fname,fext];
            end
            for i = 1:numel(pkgInfoStruct.LibIncludeFiles)
                [~,fname, fext] = fileparts(pkgInfoStruct.LibIncludeFiles{i});
                pkgInfoStruct.LibIncludeFiles{i} = ['include/',pkgInfoStruct.PackageName,'/',fname,fext];
            end
            for i = 1:numel(pkgInfoStruct.MessageFiles)
                [~,fname, fext] = fileparts(pkgInfoStruct.MessageFiles{i});
                pkgInfoStruct.MessageFiles{i} = [obj.CUSTOMMSG_FOLDER_SUFFIX, fname,fext];
            end
            for i = 1:numel(pkgInfoStruct.ServiceFiles)
                [~,fname, fext] = fileparts(pkgInfoStruct.ServiceFiles{i});
                pkgInfoStruct.ServiceFiles{i} = [obj.CUSTOMSRV_FOLDER_SUFFIX, fname,fext];
            end
            for i = 1:numel(pkgInfoStruct.ActionFiles)
                [~,fname, fext] = fileparts(pkgInfoStruct.ActionFiles{i});
                pkgInfoStruct.ActionFiles{i} = [obj.CUSTOMMSG_FOLDER_SUFFIX, fname,fext];
            end
        end

        function createFileFromTemplate(~, pkgInfoStruct, template, filePath)
        %helper utility to generate the given file using the given
        %template
            templ = ros.internal.emitter.MLTemplate;
            templ.loadFile(template);
            templ.outFile = filePath;
            templ.render(pkgInfoStruct, 2);
        end

        function result = createGivenPackage(h, pkgName, force)
        %Create a given package
            clnUp = h.createAndChangeToSrcDir; %#ok<NASGU>

            validateattributes(force,{'logical'},{'nonempty'});
            validateattributes(pkgName,{'char','string'},{'nonempty'});
            pkgName = convertStringsToChars(pkgName);

            pkgInfo = h.getPackageInfo(pkgName);
            pkgDir = fullfile(h.RootDir,'src',pkgName);
            cmakeFilePath = fullfile(pkgDir,'CMakeLists.txt');
            pkgXMLPath = fullfile(pkgDir,'package.xml');
            visibilityH = fullfile(pkgDir,'include',pkgName,'visibility_control.h');

            %if force, then recreate everything
            %the best way is to remove the package dir
            if force
                if isfolder(pkgDir)
                    [status, msg, msgid] = rmdir(pkgDir,'s');
                    if status ~= 1
                        error(msgid,msg);
                    end
                end
            else
                if isfile(cmakeFilePath) ...
                        && isfile(pkgXMLPath) ...
                        && isfile(visibilityH)
                    result = pkgDir;
                    return;
                end
            end

            %Get the paths
            sourceDir = fullfile(pkgDir,'src');
            messagePath = fullfile(pkgDir,'msg');
            servicePath = fullfile(pkgDir,'srv');
            actionPath = fullfile(pkgDir,'action');
            MATLABDestinationPath = fullfile(pkgDir,pkgInfo.MATLABDestPath);
            MsgClassDestinationPath = fullfile(pkgDir,pkgInfo.MsgClassDestPath);
            packageName = fullfile(pkgDir,'include',pkgName);
            packageInfo = fullfile(pkgDir,h.PKGINFONAME);

            % Make the directory structure
            h.makeDirStructure(pkgInfo, pkgDir, sourceDir, messagePath, servicePath, actionPath, MATLABDestinationPath, MsgClassDestinationPath);

            %copy source files
            h.copyFiles(pkgInfo.SourceFiles, sourceDir);
            h.copyFiles(pkgInfo.OtherFiles, sourceDir);
            h.copyFiles(pkgInfo.ImportedLibraries, sourceDir);
            h.copyFiles(pkgInfo.LibSourceFiles, sourceDir);
            h.copyFiles(pkgInfo.MessageFiles, messagePath);
            h.copyFiles(pkgInfo.ServiceFiles, servicePath);
            h.copyFiles(pkgInfo.ActionFiles, actionPath);
            h.copyFiles(pkgInfo.MATLABFiles, MATLABDestinationPath);
            h.copyFiles(pkgInfo.MsgClassFiles, MsgClassDestinationPath);
            %copy include files
            h.copyFiles(pkgInfo.IncludeFiles, packageName);
            h.copyFiles(pkgInfo.LibIncludeFiles, packageName);

            %get the pkgInfoStruct with relative paths
            pkgInfoStruct = h.getAndUpdatePkgInfo(pkgInfo);

            h.createFileFromTemplate(pkgInfoStruct, h.CMAKETEMPLATE, cmakeFilePath);
            h.createFileFromTemplate(pkgInfoStruct, h.PKGXMLTEMPLATE, pkgXMLPath);
            h.createFileFromTemplate(pkgInfoStruct, h.VISIBILITYHTEMPLATE, visibilityH);
            save(packageInfo,'pkgInfo');
            result = pkgDir;
        end

        function [result, pkgSrc, pkgMsg, pkgSrv, pkgAction, MATLABFileDir, MsgClassDir] = createGivenPackageFolder(h, pkgName, force)
        %Create a given package
            clnUp = h.createAndChangeToSrcDir; %#ok<NASGU>

            validateattributes(force,{'logical'},{'nonempty'});
            validateattributes(pkgName,{'char','string'},{'nonempty'});
            pkgName = convertStringsToChars(pkgName);

            pkgInfo = h.getPackageInfo(pkgName);
            pkgDir = fullfile(h.RootDir,'src',pkgName);
            cmakeFilePath = fullfile(pkgDir,'CMakeLists.txt');
            pkgXMLPath = fullfile(pkgDir,'package.xml');
            visibilityH = fullfile(pkgDir,'include',pkgName,'visibility_control.h');

            %Get the paths
            pkgSrc = fullfile(pkgDir,'src');
            pkgMsg = fullfile(pkgDir,'msg');
            pkgSrv = fullfile(pkgDir,'srv');
            pkgAction = fullfile(pkgDir,'action');
            MATLABFileDir = fullfile(pkgDir,pkgInfo.MATLABDestPath);
            MsgClassDir = fullfile(pkgDir,pkgInfo.MsgClassDestPath);

            %if force, then recreate everything
            %the best way is to remove the package dir
            if force
                if isfolder(pkgDir)
                    [status, msg, msgid] = rmdir(pkgDir,'s');
                    if status ~= 1
                        error(msgid,msg);
                    end
                end
            else
                if isfile(cmakeFilePath) ...
                        && isfile(pkgXMLPath) ...
                        && isfile(visibilityH)
                    result = pkgDir;
                    return;
                end
            end

            % Make the directory structure
            h.chkAndCreateDir(pkgDir);
            h.chkAndCreateDir(pkgSrc);
            pkgInc = fullfile(pkgDir,'include',pkgInfo.PackageName);
            h.chkAndCreateDir(pkgInc);
            h.chkAndCreateDir(pkgMsg);
            h.chkAndCreateDir(pkgSrv);
            h.chkAndCreateDir(pkgAction);
            h.chkAndCreateDir(MATLABFileDir);
            h.chkAndCreateDir(MsgClassDir);

            result = pkgDir;
        end

        function updateFilesInPath(h, pkgDir, pkgInfo, msgDir, srvDir, actionDir, generate)
        %copy files in package to build path
            packageName = fullfile(pkgDir,'include',pkgInfo.PackageName);
            %copy messages and service files
            h.copyFiles(pkgInfo.MessageFiles, msgDir);
            h.copyFiles(pkgInfo.ServiceFiles, srvDir);
            h.copyFiles(pkgInfo.ActionFiles, actionDir);

            %copy include files
            h.copyFiles(pkgInfo.IncludeFiles, packageName);
            h.copyFiles(pkgInfo.LibIncludeFiles, packageName);

            %get the pkgInfoStruct with relative paths
            pkgInfoStruct = h.getAndUpdatePkgInfo(pkgInfo);
            cmakeFilePath = fullfile(pkgDir,'CMakeLists.txt');
            h.createFileFromTemplate(pkgInfoStruct, h.CMAKETEMPLATE, cmakeFilePath);
            pkgXMLPath = fullfile(pkgDir,'package.xml');
            h.createFileFromTemplate(pkgInfoStruct, h.PKGXMLTEMPLATE, pkgXMLPath);
            packageInfo = fullfile(pkgDir,h.PKGINFONAME);
            save(packageInfo,'pkgInfo');
            %For rebuild (subsequent build), visibility_control.h is not
            %regenerated, rather utilized from the previous build
            if generate
                visibilityH = fullfile(pkgDir,'include',pkgInfo.PackageName,'visibility_control.h');
                h.createFileFromTemplate(pkgInfoStruct, h.VISIBILITYHTEMPLATE, visibilityH);
            end
        end
    end

    % Public methods
    methods (Access=protected)
        function h = ROSProjectBuilder(varargin)
        %ROSProjectBuilder(ROOTDIR, PKGNAME) Creates a ROSProjectBuilder
        %object with one package. Add other packages using addPackage Use
        %PackageInfo to get information about packages.
        %Parse Inputs
            h.createInputParser();
            h.Parser.parse(varargin{:});
            h.RootDir = h.Parser.Results.rootDir;

            %Add the default package
            h.addPackage(h.Parser);

            h.GenCodeOnly = h.Parser.Results.GenCodeOnly;

            if h.GenCodeOnly
                % Do not create Python environment, or validate CMAKE and
                % Python versions when generate code only is turned on.
                return;
            end

            %Setup Python and CMake
            [h.LocalPythonPath, h.ActivatePath, h.PyEnvDir] = h.setupPythonAndCmakeTools(h.Parser.Results.forceRecreateVENV);
            h.PyEnvDir = ['"' h.PyEnvDir '"'];
            h.CMakePath = h.getCMakePath();

            %Check for compiler
            h.MexInfo = mex.getCompilerConfigurations('C++');
            if isempty(h.MexInfo)
                error(message('ros:mlros2:util:NeedCPPCompiler'))
            end
            if ispc
                verNum = ros.internal.utilities.getVersionVal(h.MexInfo.Name);
                %Compiler Support for both VS2017 and VS2019
                if ~contains(h.MexInfo.Manufacturer,h.MSVCNAME) || ~ismember(verNum, h.MSVCVERSION)
                    error(message('ros:mlros2:util:NeedCompatibleCPPCompiler',h.MexInfo.Name));
                end

                sourcePath = fullfile(getenv('USERPROFILE'),'source');
                if isfolder(sourcePath) && ~isequal(sourcePath, h.RootDir)
                    error(message('ros:utilities:util:RemoveSourceFolder',...
                                  fullfile(getenv('USERPROFILE'),'source'), ...
                                  h.RootDir));
                end
            end
        end
    end

    methods
        function addPackage(h, varargin)
        %addPackage(PACKAGENAME) adds the packages
        %addPackage(PARSER) adds the package found in parser.Results
            pkg = ros.internal.PackageInfo(varargin{:});

            %check for duplicate name
            idx = h.findPackage(pkg.PackageName);
            if ~isempty(idx)
                error(message('ros:utilities:util:DuplicatePackageName', pkg.PackageName));
            end

            %Create a cell array of PackageInfos
            if isempty(h.Packages)
                h.Packages{1} = pkg;
            else
                h.Packages{end+1} = pkg;
            end
        end

        function rmPackage(h, pkgName)
        %rmPackage(PACKAGENAME) removes the package
            validateattributes(pkgName,{'char','string'},{'nonempty'});
            pkgName = convertStringsToChars(pkgName);
            idx = h.findPackage(pkgName);
            if isempty(idx)
                error(message('ros:utilities:util:PackageNotFound', pkgName));
            end
            h.Packages{idx} = [];
        end

        function updatePackage(h, pkgInfo)
            validateattributes(pkgInfo,{'ros.internal.PackageInfo'},{'nonempty'});
            idx = h.findPackage(pkgInfo.PackageName);
            if isempty(idx)
                error(message('ros:utilities:util:PackageNotFound', pkgInfo.PackageName));
            end
            h.Packages{idx} = pkgInfo;
        end

        function results = createPackage(h, pkgNames, force)
        % createPackage(PKGNAMES) creates the packages listed in pkgNames
        % If no name is given, creates all packages

            if nargin < 3
                force = false;
            else
                validateattributes(force,{'logical'},{'nonempty'});
            end
            if nargin < 2 || isempty(pkgNames)
                %if no pkgNames provides just get all the package names
                pkgNames = cellfun(@(x)x.PackageName,h.Packages,'UniformOutput',false);
            else
                %we need cell array of pkgNames
                if ~iscell(pkgNames)
                    pkgNames = {pkgNames};
                end
                %check for valid pkgNames
                cellfun(@(x)validateattributes(x,{'char','string'},{'nonempty'}),pkgNames);
                pkgNames = cellfun(@(x)convertStringsToChars(x),pkgNames,'UniformOutput',false);

                for i = 1:numel(pkgNames)
                    idx = h.findPackage(pkgNames{i});
                    if isempty(idx)
                        error(message('ros:utilities:util:PackageNotFound', pkgNames{i}));
                    end
                end
            end

            results = cell(1,numel(pkgNames));
            for i = 1:numel(pkgNames)
                results{i} = h.createGivenPackage(pkgNames{i}, force);
            end
        end

        function [result, pkgSrc, pkgMsg, pkgSrv, pkgAction, MATLABFileDir, MsgClassDir] = createPackageFolder(h, pkgName, force)
        % createPackageFolder(PKGNAMES) creates the packages listed in pkgNames
        % If no name is given, creates all packages
            if nargin < 3
                force = false;
            else
                validateattributes(force,{'logical'},{'nonempty'});
            end
            [result, pkgSrc, pkgMsg, pkgSrv, pkgAction, MATLABFileDir, MsgClassDir] = h.createGivenPackageFolder(pkgName, force);
        end

        function [result, installDir] = buildPackage(h, pkgNames, varargin)
        %buildPkg(PACKAGENAME) builds the package
        %If no name is provided, builds all packages

        %as a trick we could use h.buildPackage({},'--packages-upto pkgName')
            cmd = h.getBuildCommand(h.RootDir);

            if nargin < 2
                pkgNames = {};
            end

            if ~isempty(pkgNames)
                %we need cell array of pkgNames
                if ~iscell(pkgNames)
                    pkgNames = {pkgNames};
                end
                %check for valid pkgNames
                cellfun(@(x)validateattributes(x,{'char','string'},{'nonempty'}),pkgNames);
                pkgNames = cellfun(@(x)convertStringsToChars(x),pkgNames,'UniformOutput',false);
                cmd = h.formatCommand(cmd);
                for i = 1:numel(pkgNames)
                    idx = h.findPackage(pkgNames{i});
                    if isempty(idx)
                        error(message('ros:utilities:util:PackageNotFound', pkgNames{i}));
                    end
                    %append to the build command
                    cmd = [cmd ' ' pkgNames{i}]; %#ok<AGROW>
                end
            end

            curDir = cd (h.RootDir);
            clnUp = onCleanup(@()cd(curDir));
            [status, result] = h.runCommand(cmd, varargin{:});
            if status ~= 0
                error(message('ros:utilities:util:ErrorBuildingPackage', result));
            end
            installDir = fullfile(h.RootDir,'install');
            if nargout < 1
                disp(result);
            end
        end
    end

end
