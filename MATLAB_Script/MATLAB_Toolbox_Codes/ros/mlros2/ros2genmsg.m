function ros2genmsg(folderPath, varargin)
%ros2genmsg Generate custom messages from ROS 2 definitions
%   ros2genmsg(FOLDERPATH) generates MATLAB interfaces to ROS 2 custom
%   messages. FOLDERPATH is the path to a folder that contains the
%   definitions of the custom messages (.msg files). This function expects
%   one or more ROS 2 package folders inside the FOLDERPATH.
%
%   Building the message libraries requires Python and a C++ compiler. See
%   the ROS 2 Custom Messages documentation for more information.
%
%   ros2genmsg(FOLDERPATH,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments.
%
%      "BuildConfiguration" - Allows for selecting different compiler
%                             optimizations when building the message
%                             libraries. Options are:
%                                "fasterbuilds" - Prioritize reducing
%                                                 build and rebuild
%                                                 times. This is the
%                                                 default configuration.
%                                "fasterruns"   - Prioritize speed of 
%                                                 execution using the
%                                                 libraries.
% 
%   It is safe to call any of these functions multiple times. All messages under
%   FOLDERPATH will be rebuilt. It is also safe to switch between different
%   build configurations.
%
%   Built-in message types may be overwritten by calling ros2genmsg on a
%   directory with new message definition files for the built-in message
%   types.
%
%   After the messages are built, you can send and receive your custom
%   messages in MATLAB like any other supported messages. They can be
%   created using ros2message or viewed in the list of messages by calling
%   ros2 msg list.
%
%   See also ros2message, ros2.

% Copyright 2019-2021 The MathWorks, Inc.

% Input processing
    if nargin < 1
        folderPath = pwd;
    end
    folderPath = convertStringsToChars(folderPath);
    folderPath = ros.internal.Parsing.validateFolderPath(folderPath);
    
    %Parse inputs to function
    defaultVal = 'fasterbuilds';
    parser = inputParser;

    addParameter(parser, 'BuildConfiguration', defaultVal, ...
        @(x) validateStringParameter(x, ...
                                     {'fasterbuilds', 'fasterruns'},...
                                     'ros2genmsg',...
                                     'BuildConfiguration'));
    
    function validateStringParameter(value, options, funcName, varName)
        % Separate function to suppress output and just validate
        validatestring(value, options, funcName, varName);
    end

    % Parse the input and assign outputs
    parse(parser, varargin{:});

    %Extract full matched value if there was a partial match
    buildConfiguration = validatestring(parser.Results.BuildConfiguration, ...
                                        {'fasterbuilds', 'fasterruns'});
    %get ros2 install dir
    amentPrefixPath = ros.ros2.internal.getAmentPrefixPath;
    ros2msgSrcPathRoot = fullfile(amentPrefixPath,'share');

    % Place where we will generate cpp files
    msgFullName = {};
    srvFullName = {};
    srvFullNameRequest = {};
    srvFullNameResponse = {};
    modifiedFolderPath = strrep(folderPath,'\','/');
    dotprinter = ros.internal.DotPrinter('ros:utilities:util:IdentifyingMessages', modifiedFolderPath); %#ok<NASGU>
    pkgDirs = ros.internal.custommsgs.getPkgDirs(folderPath);
    clear dotprinter;

    ros.ros2.internal.createOrGetLocalPython(); %ensure python is available

    pkgInfos = cell(1,numel(pkgDirs));
    dotprinter = ros.internal.DotPrinter('ros:utilities:util:ValidatingMessages', ...
                                         modifiedFolderPath); %#ok<NASGU>
    [pkgMsgFiles, pkgSrvFiles] = ros.internal.custommsgs.checkValidityOfMessages(pkgDirs,folderPath,'ros2', pkgInfos);

    ros.ros2.internal.validateMsg(folderPath);

    clear dotprinter;

    %for pkg we will create two packages
    includeDirs = { fullfile(matlabroot, 'extern','include'), ...
                    fullfile(matlabroot, 'extern','include','MatlabDataArray'), ...
                    fullfile(matlabroot, 'toolbox','ros','include','ros2') };
    [libDirs, libMap] = ros.internal.custommsgs.getLibInfo;

    % Location where all files will be modified
    genDir = fullfile(folderPath, 'matlab_msg_gen', computer('arch'));
    [status, errorMsg, errorMsgId] = mkdir(genDir);
    if ~status && ~isequal(errorMsgId, 'MATLAB:MKDIR:DirectoryExists')
        error(errorMsgId, errorMsg);
    end

    %create an empty COLCON_IGNORE so it does not reread by colcon
    colconIgnorePath = fullfile(folderPath,'matlab_msg_gen','COLCON_IGNORE');
    catkinIgnorePath = fullfile(folderPath,'matlab_msg_gen','CATKIN_IGNORE');
    if ~isfile(colconIgnorePath)
        [fID, errorMsg] = fopen(colconIgnorePath,'w');
        assert(fID >= 0, message('ros:utilities:custommsg:CreateColconIgnore', colconIgnorePath, errorMsg));
        fclose(fID);
    end

    if ~isfile(catkinIgnorePath)
        [fID, errorMsg] = fopen(catkinIgnorePath,'w');
        assert(fID >= 0, message('ros:utilities:custommsg:CreateCatkinIgnore', catkinIgnorePath, errorMsg));
        fclose(fID);
    end

    builder = ros.ros2.internal.ColconBuilder.empty;
    msgGenProgress = ros.internal.TextProgressBar('ros:utilities:util:GenMATLABInterfaceInProgress');
    msgGenProgress.printProgress(0,numel(pkgDirs)); %initialize progress with 0%
    
    % Generate publisher and subscriber files, and MATLAB interfaces
    for iPkg = 1:numel(pkgDirs)
        msgFiles = pkgMsgFiles{iPkg};
        srvFiles = pkgSrvFiles{iPkg};
        pkgInfos{iPkg} = ros.internal.PackageInfo(pkgDirs{iPkg}, ...
                                                  'cppLibraryName', [pkgDirs{iPkg} '_matlab'], ...
                                                  'libFormat', 'SHARED', ...
                                                  'includeDirs', includeDirs, ...
                                                  'libDirs', {libDirs}, ...
                                                  'libs', {libMap(computer('arch'))}, ...
                                                  'matlabDestPath', fullfile('m','+ros','+internal','+ros2','+custommessages',['+' pkgDirs{iPkg}]), ...
                                                  'dependencies', {'class_loader', 'console_bridge', 'rclcpp', 'rcutils'});
        %TODO: Read package.xml in that directory for other properties (g1998282)

        % Set up folders for MATLAB file generation
        if iPkg == 1
            builder = ros.ros2.internal.ColconBuilder(genDir, pkgInfos{iPkg});
        else
            addPackage(builder, pkgInfos{iPkg});
        end

        % Md5checksum.mat - contains cached values of checkSumMap which is
        % reused for smart build of ros2genmsg if there is a change in few message definitions.
        msgMd5ChecksumStoragePath = fullfile(builder.RootDir,'src',pkgDirs{iPkg},'Md5checksum.mat');
        generateFiles = ~isfile(msgMd5ChecksumStoragePath);
        [pkgDir, srcGenDir, pkgMsg, pkgSrv, ~, structGenDir] = createPackageFolder(builder, pkgInfos{iPkg}.PackageName, generateFiles);

        %refMsgTypeCheckSumMap - contains the calculated Md5 values in current build.
        %checkSumMap - contains Md5 values calculated during previous build (used in case of rebuild).
        refMsgTypeCheckSumMap = containers.Map('KeyType', 'char', 'ValueType', 'char');
        checkSumMap = containers.Map('KeyType', 'char', 'ValueType', 'char');
        registry = ros.internal.CustomMessageRegistry.getInstance('ros2');

        %if Md5checksum.mat exists, load checkSumMap variable into
        %the current workspace
        if isfile(msgMd5ChecksumStoragePath)
           load(msgMd5ChecksumStoragePath, 'checkSumMap');
        end

        for iMsg = 1:numel(msgFiles)
            [~, msgName] = fileparts(msgFiles(iMsg).name);
            msgFullName{end+1} = [pkgDirs{iPkg} '/' msgName]; %#ok<AGROW>

            % If we want to update any built-in message, preference should be given to the custom message directory first.
            [genFiles, dependencies] = ros.internal.pubsubEmitter(msgFullName{end}, ...
                                                              {folderPath, ros2msgSrcPathRoot}, ...
                                                              'ros2', refMsgTypeCheckSumMap, registry, checkSumMap,...
                                                              srcGenDir, structGenDir, structGenDir);
            %remove this pkgName from dependencies
            dependencies = setdiff(dependencies,pkgDirs{iPkg});
            for i = 1:numel(genFiles)
                [~,~,fext] = fileparts(genFiles{i});
                if isequal(fext,'.cpp')
                    pkgInfos{iPkg}.LibSourceFiles = [pkgInfos{iPkg}.LibSourceFiles {fullfile(srcGenDir,genFiles{i})}];
                elseif isequal(fext,'.m')
                    pkgInfos{iPkg}.MATLABFiles = [pkgInfos{iPkg}.MATLABFiles genFiles(i)];
                end
            end
            pkgInfos{iPkg}.MessageFiles = [ pkgInfos{iPkg}.MessageFiles {fullfile(msgFiles(iMsg).folder,msgFiles(iMsg).name)} ];
            for i = 1:numel(dependencies)
                dependency = fileparts(dependencies{i});
                if ~isequal(dependency, pkgDirs{iPkg})   % Avoid reflective dependencies
                    pkgInfos{iPkg}.Dependencies = unique([ pkgInfos{iPkg}.Dependencies {dependency} ]);
                    pkgInfos{iPkg}.MsgDependencies = unique([ pkgInfos{iPkg}.MsgDependencies {dependency} ]);
                end
            end
        end
        
        for iSrv = 1:numel(srvFiles)
            [~, srvName] = fileparts(srvFiles(iSrv).name);
            srvFullName{end+1} = [pkgDirs{iPkg} '/' srvName]; %#ok<AGROW>
            srvFullNameRequest{end+1} = [srvFullName{end} 'Request']; %#ok<AGROW>
            srvFullNameResponse{end+1} = [srvFullName{end} 'Response']; %#ok<AGROW>

            % If we want to update any built-in message, preference should be given to the custom message directory first.
            [genFiles, requestDependencies, responseDependencies] = ros.internal.clientserverEmitter(...
                srvFullName{end}, {folderPath, ros2msgSrcPathRoot}, ...
                'ros2', refMsgTypeCheckSumMap, registry, checkSumMap, srcGenDir, structGenDir, structGenDir);

            dependencies = union(requestDependencies,responseDependencies);
            %remove this pkgName from dependencies
            dependencies = setdiff(dependencies,pkgDirs{iPkg});
            for i = 1:numel(genFiles)
                [~,~,fExt] = fileparts(genFiles{i});
                if isequal(fExt,'.cpp')
                    pkgInfos{iPkg}.LibSourceFiles = [pkgInfos{iPkg}.LibSourceFiles {fullfile(srcGenDir,genFiles{i})}];
                elseif isequal(fExt,'.m')
                    pkgInfos{iPkg}.MATLABFiles = [pkgInfos{iPkg}.MATLABFiles genFiles(i)];
                end
            end
            pkgInfos{iPkg}.ServiceFiles = [ pkgInfos{iPkg}.ServiceFiles {fullfile(srvFiles(iSrv).folder,srvFiles(iSrv).name)} ];
            for i = 1:numel(dependencies)
                dependency = fileparts(dependencies{i});
                if ~isequal(dependency, pkgDirs{iPkg})   % Avoid reflective dependencies
                    pkgInfos{iPkg}.Dependencies = unique([ pkgInfos{iPkg}.Dependencies {dependency} ]);
                    pkgInfos{iPkg}.MsgDependencies = unique([ pkgInfos{iPkg}.MsgDependencies {dependency} ]);
                end
            end
        end

        %copy msg and srv files from package directory to
        %matlab_msg_gen path and create package.xml, CMakeLists.txt
        %and visibility_control.h files from templates.
        updateFilesInPath(builder, pkgDir, pkgInfos{iPkg}, pkgMsg, pkgSrv, [], generateFiles);

        %update the Md5 values of current build in checkSumMap
        checkSumMap = refMsgTypeCheckSumMap;
        save(msgMd5ChecksumStoragePath, 'checkSumMap');
        msgGenProgress.printProgress(iPkg, numel(pkgDirs));
    end

    if isempty(pkgInfos)
        error(message('ros:utilities:custommsg:NoPackage', folderPath));
    end

    % Build the messages
    % Use Ninja build tool that ships with MATLAB to generate messages
    ninjaBuildTool = fullfile(matlabroot, 'toolbox', 'shared', 'coder', 'ninja', computer('arch'));
    originalPathEnv = getenv('PATH');
    resetPath = onCleanup(@()setenv('PATH',originalPathEnv));
    setenv('PATH',[ninjaBuildTool, pathsep, originalPathEnv]);
    colconMakeArgsMap = containers.Map();
    
    %Flags for Faster Builds, Faster Runs and Debug build configurations.
    switch (buildConfiguration)
        case {'fasterruns'}
            %Build with FasterRuns support
            config = ' -DCMAKE_BUILD_TYPE=Release';
            optimizationFlagsForWindows = ' -DCMAKE_CXX_FLAGS_RELEASE="/MD /O2 /Ob2 /DNDEBUG" ';
            optimizationFlagsForUnix = ' -DCMAKE_CXX_FLAGS_RELEASE=-O3 ';
            %case {'debug'}
            %%Debug Builds
            %config = ' -DCMAKE_BUILD_TYPE=Debug';
            %optimizationFlagsForWindows = ' -DCMAKE_CXX_FLAGS_DEBUG="/MDd /Zi /Ob0 /Od /RTC1" ';
            %optimizationFlagsForUnix = ' -DCMAKE_CXX_FLAGS_DEBUG=-g ';
        otherwise
            %Build with FasterBuilds support by default
            config = ' -DCMAKE_BUILD_TYPE=Release';
            optimizationFlagsForWindows = ' -DCMAKE_CXX_FLAGS_RELEASE="/MD /Od /Ob2 /DNDEBUG" ';
            optimizationFlagsForUnix = ' -DCMAKE_CXX_FLAGS_RELEASE=-O0 ';
    end
    
    colconMakeArgsMap('win64')   = [' --cmake-args -G Ninja ',...
        ' --cmake-args', config,' -DBUILD_TESTING=OFF ', ...
        ' -DCMAKE_CXX_COMPILER="cl.exe" -DCMAKE_C_COMPILER="cl.exe" ', ...
        ' -DCMAKE_LINKER="link.exe" -DMSVC_TOOLSET_VERSION=141 ',optimizationFlagsForWindows];
    colconMakeArgsMap('glnxa64') = [' --cmake-args -G Ninja ',...
        ' --cmake-args', config,' -DBUILD_TESTING=OFF ',optimizationFlagsForUnix];
    colconMakeArgsMap('maci64')  = [' --cmake-args -G Ninja ',...
        ' --cmake-args', config,' -DBUILD_TESTING=OFF ',optimizationFlagsForUnix];
    colconMakeArgs = colconMakeArgsMap(computer('arch'));
    %build packages with colcon
    buildPackage(builder, [], ' --merge-install', colconMakeArgs); %other messages might need to be present in the same directory

    % Update preferences with folder information
    reg = ros.internal.custommsgs.updatePreferences(msgFullName,srvFullNameRequest,srvFullNameResponse,srvFullName,{},'ros2',genDir); %#ok<NASGU>

    %Get the persistent map for ROS2.
    msgMapROS2 = ros.internal.utilities.getPersistentMapForROS2('clear'); %#ok<NASGU> 
end

% LocalWords:  custommessages custommsg srv DCMAKE DBUILD CXX DMSVC TOOLSET Od DNDEBUG
