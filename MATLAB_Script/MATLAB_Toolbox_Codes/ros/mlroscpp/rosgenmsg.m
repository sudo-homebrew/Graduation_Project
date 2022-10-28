function rosgenmsg(folderPath, varargin)
%rosgenmsg Generate custom messages from ROS definitions
%   rosgenmsg(FOLDERPATH) generates MATLAB interfaces to ROS custom
%   messages. FOLDERPATH is the path to a folder that contains the
%   definitions of the custom messages (.msg files). This function expects
%   one or more ROS package folders inside the FOLDERPATH.
%
%   Building the message libraries requires Python and a C++ compiler. See
%   the ROS Custom Messages documentation for more information.
% 
%   rosgenmsg(FOLDERPATH,Name,Value) provides additional options
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
%   FOLDERPATH will be rebuilt.It is also safe to switch between different
%   build configurations.
%
%   Built-in message types may be overwritten by calling rosgenmsg on a
%   directory with new message definition files for the built-in message
%   types.
%
%   After calling rosgenmsg, you have to modify the MATLAB path in order to
%   use the custom messages. rosgenmsg displays the concrete steps that you
%   have to follow.
%   You will only have to make these changes once and anytime that the
%   custom message definitions change or if you want to add support for
%   additional custom messages.
%
%   After the initial setup, you can send and receive your custom messages
%   in MATLAB like any other supported messages. They can be created using
%   rosmessage or viewed in the list of messages by calling "rosmsg list".
%
%   See also rosmessage, rosmsg.

% Copyright 2015-2021 The MathWorks, Inc.

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
                                     'rosgenmsg',...
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
    %get ros install dir
    catkinPrefixPath = ros.internal.getCatkinPrefixPath;
    rosmsgSrcPathRoot = fullfile(catkinPrefixPath,'share');
    extraROSMessagePath = fullfile(matlabroot,'toolbox','ros', ...
                                'mlroscpp','custom_messages','share');

    % If we want to update any built-in message, preference should be given to the custom message directory first.                                   
    msgFilesDirs = {folderPath,rosmsgSrcPathRoot,extraROSMessagePath};
    modifiedFolderPath = strrep(folderPath,'\','/');
    dotprinter = ros.internal.DotPrinter('ros:utilities:util:IdentifyingMessages', modifiedFolderPath); %#ok<NASGU>
    pkgDirs = ros.internal.custommsgs.getPkgDirs(folderPath);
    clear dotprinter;
    ros.internal.createOrGetLocalPython(); %ensure python is available

    pkgInfos = cell(1,numel(pkgDirs));
    dotprinter = ros.internal.DotPrinter('ros:utilities:util:ValidatingMessages', ...
                                         modifiedFolderPath); %#ok<NASGU>
    [pkgMsgFiles, pkgSrvFiles, pkgActionFiles] = ros.internal.custommsgs.checkValidityOfMessages(pkgDirs,folderPath,'ros', pkgInfos);

    clear dotprinter;

    [boostRootPath, boostIncludeDirs] = ros.internal.getBoostRootPath;
    libBoostDirs = fullfile(boostRootPath,'lib');

    %for each pkg we will create a pkgInfo.
    includeDirs = { fullfile(matlabroot, 'extern','include'), ...
                    fullfile(matlabroot, 'extern','include','MatlabDataArray'), ...
                    fullfile(matlabroot, 'toolbox','ros','include','ros1'), ...
                    fullfile(matlabroot, 'sys','ros1',computer('arch'),'ros1','include'),...
                    fullfile(matlabroot, 'sys','ros1',computer('arch'),'ros1','include','class_loader'),...
                    fullfile(matlabroot, 'sys','ros1',computer('arch'),'ros1','console_bridge','include'),...
                    fullfile(matlabroot, 'toolbox','ros','mlroscpp','custom_messages','include'),...
                    fullfile(matlabroot, 'sys','ros1',computer('arch'),'ros1','bzip2','include'),...
                    fullfile(matlabroot, 'sys','ros1',computer('arch'),'ros1','lz4','include'),...
                    boostIncludeDirs
                  };

    [libDirs, libMap] = ros.internal.custommsgs.getLibInfo;

    libRos1DirsMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                    { ...
                                        fullfile(matlabroot,'sys','ros1','win64','ros1','lib') ...
                                        fullfile(matlabroot,'sys','ros1','maci64','ros1','lib'),...
                                        fullfile(matlabroot,'sys','ros1','glnxa64','ros1','lib'),...
                   });

    
    libConsoleBridgeDirsMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                             { ...
                                                 fullfile(matlabroot,'sys','ros1','win64','ros1','console_bridge','lib') ...
                                                 fullfile(matlabroot,'sys','ros1','maci64','ros1','console_bridge','lib'),...
                                                 fullfile(matlabroot,'sys','ros1','glnxa64','ros1','console_bridge','lib'),...
                   });

    libRos1Dirs = libRos1DirsMap(computer('arch'));

    libConsoleBridgeDirs = libConsoleBridgeDirsMap(computer('arch'));

    libConsoleBridgeMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                         { ...
                                             fullfile(libConsoleBridgeDirs,'console_bridge.lib'), ...
                                             fullfile(libConsoleBridgeDirs,'libconsole_bridge.dylib'),...
                                             fullfile(libConsoleBridgeDirs,'libconsole_bridge.so'),...
                   });

    libClassLoaderMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                       { ...
                                           fullfile(libRos1Dirs,'ros1_class_loader.lib'), ...
                                           fullfile(libRos1Dirs,'libros1_class_loader.dylib'),...
                                           fullfile(libRos1Dirs,'libros1_class_loader.so'),...
                   });

    libRosconsoleMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                      { ...
                                          fullfile(libRos1Dirs,'rosconsole.lib'), ...
                                          fullfile(libRos1Dirs,'librosconsole.dylib'),...
                                          fullfile(libRos1Dirs,'librosconsole.so'),...
                   });

    libRoscppMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                  { ...
                                      fullfile(libRos1Dirs,'roscpp.lib'), ...
                                      fullfile(libRos1Dirs,'libroscpp.dylib'),...
                                      fullfile(libRos1Dirs,'libroscpp.so'),...
                   });

    libRoscppSerializationMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                               { ...
                                                   fullfile(libRos1Dirs,'roscpp_serialization.lib'), ...
                                                   fullfile(libRos1Dirs,'libroscpp_serialization.dylib'),...
                                                   fullfile(libRos1Dirs,'libroscpp_serialization.so'),...
                   });
    % lib for rostime dependency
    libRosTime = containers.Map({'win64','maci64','glnxa64'},...
                                {...
                                    fullfile(libRos1Dirs,'rostime.lib'),...
                                    fullfile(libRos1Dirs,'librostime.dylib'),...
                                    fullfile(libRos1Dirs,'librostime.so'),...
                   });
    
    % rosbag changes
    librosbag_storage = containers.Map({'win64','maci64','glnxa64'},...
                                {...
                                    fullfile(libRos1Dirs,'rosbag_storage.lib'),...
                                    fullfile(libRos1Dirs,'librosbag_storage.dylib'),...
                                    fullfile(libRos1Dirs,'librosbag_storage.so'),...
                   });
    libroslz4 = containers.Map({'win64','maci64','glnxa64'},...
                                {...
                                    fullfile(libRos1Dirs,'roslz4.lib'),...
                                    fullfile(libRos1Dirs,'libroslz4.dylib'),...
                                    fullfile(libRos1Dirs,'libroslz4.so'),...
                   });
    libtopic_tools = containers.Map({'win64','maci64','glnxa64'},...
                                {...
                                    fullfile(libRos1Dirs,'topic_tools.lib'),...
                                    fullfile(libRos1Dirs,'libtopic_tools.dylib'),...
                                    fullfile(libRos1Dirs,'libtopic_tools.so'),...
                   });
    libmatlab_rosbag = containers.Map({'win64','maci64','glnxa64'},...
                                {...
                                    fullfile(libRos1Dirs,'matlab_rosbag.lib'),...
                                    fullfile(libRos1Dirs,'libmatlab_rosbag.dylib'),...
                                    fullfile(libRos1Dirs,'libmatlab_rosbag.so'),...
                   });
    libactionlib = containers.Map({'win64','maci64','glnxa64'},...
                                {...
                                    fullfile(libRos1Dirs,'actionlib.lib'),...
                                    fullfile(libRos1Dirs,'libactionlib.dylib'),...
                                    fullfile(libRos1Dirs,'libactionlib.so'),...
                   });
    libboostthread = containers.Map({'win64','maci64','glnxa64'},...
                                {...
                                    fullfile(libBoostDirs,'mwboost_thread-vc142-mt-x64-1_75.lib'),...
                                    fullfile(libBoostDirs,'libmwboost_thread.dylib'),...
                                    fullfile(libBoostDirs,'libmwboost_thread.so'),...
                   });

    % Generate publisher, subscriber, service files, and MATLAB interfaces
    msgFullName = {};
    srvFullName = {};
    srvFullNameRequest = {};
    srvFullNameResponse = {};
    actionFullName = {};
    actionFullNameGoal = {};
    actionFullNameFeedback = {};
    actionFullNameResult = {};
    actionFullNameActionGoal = {};
    actionFullNameActionFeedback = {};
    actionFullNameActionResult = {};
    actionFullNameAction = {};
    % Use custom message registry constants for consistency between tools
    customRegistry = ros.internal.CustomMessageRegistry.getInstance('ros', false);

    % Location where all files will be modified
    genDir = fullfile(folderPath, 'matlab_msg_gen_ros1', computer('arch'));
    [status, errorMsg, errorMsgId] = mkdir(genDir);
    if ~status && ~isequal(errorMsgId, 'MATLAB:MKDIR:DirectoryExists')
        error(errorMsgId, errorMsg);
    end

    %create an empty CATKIN_IGNORE so it does not reread by catkin
    catkinIgnorePath = fullfile(folderPath,'matlab_msg_gen_ros1','CATKIN_IGNORE');
    colconIgnorePath = fullfile(folderPath,'matlab_msg_gen_ros1','COLCON_IGNORE');
    if ~isfile(catkinIgnorePath)
        [fID, errorMsg] = fopen(catkinIgnorePath,'w');
        assert(fID >= 0, message('ros:utilities:custommsg:CreateCatkinIgnore', catkinIgnorePath, errorMsg));
        fclose(fID);
    end

    if ~isfile(colconIgnorePath)
        [fID, errorMsg] = fopen(colconIgnorePath,'w');
        assert(fID >= 0, message('ros:utilities:custommsg:CreateColconIgnore', colconIgnorePath, errorMsg));
        fclose(fID);
    end

    builder = ros.internal.CatkinBuilder.empty;    
    msgGenProgress = ros.internal.TextProgressBar('ros:utilities:util:GenMATLABInterfaceInProgress');
    msgGenProgress.printProgress(0,numel(pkgDirs)); %initialize progress with 0%
    
    for iPkg = 1:numel(pkgDirs)
        msgFiles = pkgMsgFiles{iPkg};
        srvFiles = pkgSrvFiles{iPkg};
        actionFiles = pkgActionFiles{iPkg};
        pkgInfos{iPkg} = ros.internal.PackageInfo(pkgDirs{iPkg}, ...
                                                  'cppLibraryName', [pkgDirs{iPkg} '_matlab'], ...
                                                  'libFormat', 'SHARED', ...
                                                  'includeDirs', includeDirs, ...
                                                  'libDirs', {libDirs, libConsoleBridgeDirs, libRos1Dirs, libBoostDirs}, ...
                                                  'libs', {libMap(computer('arch')), libConsoleBridgeMap(computer('arch')), ...
                                                   libClassLoaderMap(computer('arch')), libRosconsoleMap(computer('arch')), ...
                                                   libRoscppMap(computer('arch')), libRoscppSerializationMap(computer('arch')), ...
                                                   libRosTime(computer('arch')), ...
                                                   librosbag_storage(computer('arch')), ...
                                                   libroslz4(computer('arch')), ...
                                                   libtopic_tools(computer('arch')), ...
                                                   libmatlab_rosbag(computer('arch')), ...
                                                   libactionlib(computer('arch')), ...
                                                   libboostthread(computer('arch'))}, ...
                                                  'matlabDestPath', fullfile(customRegistry.FUNCTIONFILEPATH, ['+' pkgDirs{iPkg}]), ...
                                                  'msgClassDestPath', fullfile(customRegistry.CLASSFILEPATH, ['+' pkgDirs{iPkg}]), ...
                                                  'dependencies', {'roscpp'}, ...
                                                  'BuildDependencies', {'message_generation'},...
                                                  'RunDependencies', {'message_runtime'});

        % Set up folders for MATLAB file generation
        if iPkg == 1
            builder = ros.internal.CatkinBuilder(genDir, pkgInfos{iPkg});
        else
            addPackage(builder, pkgInfos{iPkg});
        end

        % Md5checksum.mat - contains cached values of checkSumMap which is
        % reused for smart build of rosgenmsg if there is a change in few message definitions.
        msgMd5ChecksumStoragePath = fullfile(builder.RootDir,'src',pkgDirs{iPkg},'Md5checksum.mat');
        generateFiles = ~isfile(msgMd5ChecksumStoragePath);
        [pkgDir, srcGenDir, pkgMsg, pkgSrv, pkgAction, structGenDir, classGenDir] = createPackageFolder(builder, pkgInfos{iPkg}.PackageName, generateFiles);

        %refMsgTypeCheckSumMap - contains the calculated Md5 values in current build.
        %checkSumMap - contains Md5 values calculated during previous build (used incase of rebuild).
        refMsgTypeCheckSumMap = containers.Map('KeyType', 'char', 'ValueType', 'char');
        checkSumMap = containers.Map('KeyType', 'char', 'ValueType', 'char');       
        registry = ros.internal.CustomMessageRegistry.getInstance('ros');

        %if this Md5checksum.mat exists, we load checkSumMap variable into
        %the current workspace
        if isfile(msgMd5ChecksumStoragePath)
            load(msgMd5ChecksumStoragePath, 'checkSumMap');
        end

        for iMsg = 1:numel(msgFiles)
            [~, msgName] = fileparts(msgFiles(iMsg).name);
            msgFullName{end+1} = [pkgDirs{iPkg} '/' msgName]; %#ok<AGROW>
            [genFiles, dependencies] = ros.internal.pubsubEmitter(...
                msgFullName{end}, msgFilesDirs, ...
                'ros', refMsgTypeCheckSumMap, registry, checkSumMap, srcGenDir, structGenDir, classGenDir);

            %remove this pkgName from dependencies
            dependencies = setdiff(dependencies,pkgDirs{iPkg});
            for i = 1:numel(genFiles)
                [fPath,~,fExt] = fileparts(genFiles{i});
                if isequal(fExt,'.cpp')
                    pkgInfos{iPkg}.LibSourceFiles = [pkgInfos{iPkg}.LibSourceFiles {fullfile(srcGenDir,genFiles{i})}];
                elseif isequal(fExt,'.m')
                    if contains(fPath, '+msggen') % Generate message class name always matches message name
                        pkgInfos{iPkg}.MsgClassFiles = [pkgInfos{iPkg}.MsgClassFiles genFiles(i)];
                    else                       % Generated ROS message struct functions have different name
                    pkgInfos{iPkg}.MATLABFiles = [pkgInfos{iPkg}.MATLABFiles genFiles(i)];
                    end
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
            [genFiles, requestDependencies, responseDependencies] = ros.internal.clientserverEmitter(...
                srvFullName{end}, msgFilesDirs, ...
                'ros', refMsgTypeCheckSumMap, registry, checkSumMap, srcGenDir, structGenDir, classGenDir);

            dependencies = union(requestDependencies,responseDependencies);
            %remove this pkgName from dependencies
            dependencies = setdiff(dependencies,pkgDirs{iPkg});
            for i = 1:numel(genFiles)
                [fPath,~,fExt] = fileparts(genFiles{i});
                if isequal(fExt,'.cpp')
                    pkgInfos{iPkg}.LibSourceFiles = [pkgInfos{iPkg}.LibSourceFiles {fullfile(srcGenDir,genFiles{i})}];
                elseif isequal(fExt,'.m')
                    if contains(fPath, '+msggen')
                        pkgInfos{iPkg}.MsgClassFiles = [pkgInfos{iPkg}.MsgClassFiles genFiles(i)];
                    else
                        pkgInfos{iPkg}.MATLABFiles = [pkgInfos{iPkg}.MATLABFiles genFiles(i)];
                    end
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
        
        for iAction = 1:numel(actionFiles)
            [~, msgName] = fileparts(actionFiles(iAction).name);
            actionFullName{end+1} = [pkgDirs{iPkg} '/' msgName]; %#ok<AGROW>
            actionFullNameGoal{end+1} = [actionFullName{end} 'Goal']; %#ok<AGROW>
            actionFullNameResult{end+1} = [actionFullName{end} 'Result']; %#ok<AGROW>
            actionFullNameFeedback{end+1} = [actionFullName{end} 'Feedback']; %#ok<AGROW>
            actionFullNameActionGoal{end+1} = [actionFullName{end} 'ActionGoal']; %#ok<AGROW>
            actionFullNameActionResult{end+1} = [actionFullName{end} 'ActionResult']; %#ok<AGROW>
            actionFullNameActionFeedback{end+1} = [actionFullName{end} 'ActionFeedback']; %#ok<AGROW>
            actionFullNameAction{end+1} = [actionFullName{end} 'Action']; %#ok<AGROW>
            [genFiles, goalDependencies, resultDependencies, feedbackDependencies] = ros.internal.actionEmitter(...
                actionFullName{end}, msgFilesDirs, ...
                'ros', refMsgTypeCheckSumMap,registry,checkSumMap, ...
                srcGenDir, structGenDir, classGenDir);
            dependencies = union(goalDependencies,resultDependencies);
            dependencies = union(dependencies,feedbackDependencies);

            %remove this pkgName from dependencies
            dependencies = setdiff(dependencies,pkgDirs{iPkg});
            for i = 1:numel(genFiles)
                [~,fName,fExt] = fileparts(genFiles{i});
                if isequal(fExt,'.cpp')
                    pkgInfos{iPkg}.LibSourceFiles = [pkgInfos{iPkg}.LibSourceFiles {fullfile(srcGenDir,genFiles{i})}];
                elseif isequal(fExt,'.m')
                    if isequal(fName, msgName) % Generate message class name always matches message name
                        pkgInfos{iPkg}.MsgClassFiles = [pkgInfos{iPkg}.MsgClassFiles genFiles(i)];
                    else                       % Generated ROS message struct functions have different name
                        pkgInfos{iPkg}.MATLABFiles = [pkgInfos{iPkg}.MATLABFiles genFiles(i)];
                    end
                end
            end
            pkgInfos{iPkg}.ActionFiles = [ pkgInfos{iPkg}.ActionFiles {fullfile(actionFiles(iAction).folder,actionFiles(iAction).name)} ];
            for i = 1:numel(dependencies)
                dependency = fileparts(dependencies{i});
                if ~isequal(dependency, pkgDirs{iPkg})   % Avoid reflective dependencies
                    pkgInfos{iPkg}.Dependencies = unique([ pkgInfos{iPkg}.Dependencies {dependency} ]);
                    pkgInfos{iPkg}.MsgDependencies = unique([ pkgInfos{iPkg}.MsgDependencies {dependency} ]);
                end
            end
            pkgInfos{iPkg}.Dependencies = unique([ pkgInfos{iPkg}.Dependencies {'actionlib_msgs'} ]);
            pkgInfos{iPkg}.MsgDependencies = unique([ pkgInfos{iPkg}.MsgDependencies {'actionlib_msgs'} ]);
        end

        %copy msg and srv files from package directory to
        %matlab_msg_gen_ros1 path and create package.xml, CMakeLists.txt
        %and visibility_control.h files from templates.
        updateFilesInPath(builder, pkgDir, pkgInfos{iPkg}, pkgMsg, pkgSrv, pkgAction, generateFiles);

        %update the MD5 values of current build in checkSumMap
        checkSumMap = refMsgTypeCheckSumMap;
        save(msgMd5ChecksumStoragePath, 'checkSumMap');
        msgGenProgress.printProgress(iPkg, numel(pkgDirs));
    end

    %Append CMAKE_PREFIX_PATH with extra ros message packages part of
    %toolbox
    extraROSMessageCmakePath = fullfile(matlabroot,'toolbox','ros','mlroscpp','custom_messages');
    extraROSMessageCmakePath = strrep(extraROSMessageCmakePath,'\','/');
    
    cmake_prefix_path = ['"' catkinPrefixPath ';' extraROSMessageCmakePath '"'];    
   
    if isempty(pkgInfos)
        error(message('ros:utilities:custommsg:NoPackage', folderPath));
    end

    % Build the messages
    % Use Ninja build tool that ships with MATLAB to generate messages
    ninjaBuildTool = fullfile(matlabroot, 'toolbox', 'shared', 'coder', 'ninja', computer('arch'));
    originalPathEnv = getenv('PATH');
    resetPath = onCleanup(@()setenv('PATH',originalPathEnv));
    setenv('PATH',[ninjaBuildTool, pathsep, originalPathEnv]);
    catkinMakeArgsMap = containers.Map();
                          
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

    catkinMakeArgsMap('win64')   = [' --use-ninja --cmake-args ', ...
        ' -DALIAS_ROS1_CLASS_LOADER=1 ',...
        ' -DBoost_NAMESPACE=mwboost -DCATKIN_ENABLE_TESTING=off ', ...
        ' -DCMAKE_CXX_COMPILER="cl.exe" -DCMAKE_C_COMPILER="cl.exe" ', ...
        ' -DCMAKE_LINKER="link.exe" -DMSVC_TOOLSET_VERSION=141 -DCMAKE_PREFIX_PATH=' cmake_prefix_path, ...
        config, optimizationFlagsForWindows];
    catkinMakeArgsMap('glnxa64') = [' --use-ninja --cmake-args ', ...
        ' -DALIAS_ROS1_CLASS_LOADER=1 ',...
        ' -DBoost_NAMESPACE=mwboost -DCATKIN_ENABLE_TESTING=off -DCMAKE_PREFIX_PATH=' cmake_prefix_path, ...
        config, optimizationFlagsForUnix];
    catkinMakeArgsMap('maci64')  = [' --use-ninja --cmake-args ', ...
        ' -DALIAS_ROS1_CLASS_LOADER=1 ',...
        ' -DBoost_NAMESPACE=mwboost -DCATKIN_ENABLE_TESTING=off -DCMAKE_PREFIX_PATH=' cmake_prefix_path, ...
        config, optimizationFlagsForUnix];
    catkinMakeArgs = catkinMakeArgsMap(computer('arch'));

    %build packages with catkin 
    buildPackage(builder, [], 'install', catkinMakeArgs); %other messages might need to be present in the same directory

    msgFullName = [msgFullName actionFullNameGoal actionFullNameResult actionFullNameFeedback ...
        actionFullNameActionGoal actionFullNameActionResult actionFullNameActionFeedback actionFullNameAction];
    % Update preferences with folder information
    reg = ros.internal.custommsgs.updatePreferences(msgFullName,srvFullNameRequest,srvFullNameResponse,srvFullName,actionFullName,'ros',genDir); %#ok<NASGU>
    
    %Clear the persistent map.
    msgMapROS1 = ros.internal.utilities.getPersistentMapForROS('clear'); %#ok<NASGU> 

    % Final instructions to the user to be able to use message classes
    disp(' ');
    disp(message('ros:mlroscpp:rosgenmsg:ToUseCustomMessages').getString)
    disp(' ');
    disp(message('ros:mlroscpp:rosgenmsg:Step1AddPath').getString);
    disp(' ')
    msgClassFolder = fullfile(genDir, 'install', 'm');
    disp(strcat('addpath(''', msgClassFolder, ''')'));
    disp('savepath');
    disp(' ');
    disp(message('ros:mlroscpp:rosgenmsg:Step2ClearRehash').getString)
    disp(' ');
    disp('clear classes')
    disp('rehash toolboxcache')
    disp(' ');
    disp(message('ros:mlroscpp:rosgenmsg:Step3Verify').getString);
    disp(' ')
end

% LocalWords:  incase CMake Od DNDEBUG librosbag roslz libroslz libtopic
% LocalWords:  libmatlab
