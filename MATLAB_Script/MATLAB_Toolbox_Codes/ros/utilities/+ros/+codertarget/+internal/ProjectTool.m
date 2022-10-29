classdef (Abstract) ProjectTool < handle
% This class is for internal use only. It may be removed in the future.

% Project tool class for ROSProjectBuilder. Uses ROS ProjectBuilder object
% to create, build and run

% Copyright 2020-2022 The MathWorks, Inc.

    properties (Abstract, Constant)
        ProjectName
        ROSVersion
        DefaultDependencies
        % LinkReferenceLibraries Explicitly add dependent libraries from
        % the referenced model to target_link_libraries list
        LinkReferenceLibraries
    end

    methods (Abstract,Hidden)
        ret = getProjectData(obj);
        ret = getProjectBuilder(obj, anchorDir, pkgName, varargin);
        [res, installDir] = runBuildCommand(obj, context);
    end

    methods
        function [ret, context] = initialize(h, buildInfo, context, varargin)
            ret = true;

            if isequal(buildInfo.getBuildName,'rtwshared')
                % Skip sharedutils - these will be covered by the
                % helper called in createProject
                return;
            end
            mycontext.modelName = buildInfo.getBuildName;
            % MLC_TARGET_NAME is added only for MATLAB codegen
            mycontext.isMDL = isempty(findBuildArg(buildInfo,'MLC_TARGET_NAME'));
            mycontext.anchorDir = buildInfo.Settings.LocalAnchorDir;
            mycontext.ActiveBuildConfiguration = context.ActiveBuildConfiguration;
            mycontext.isROSControlBuild = false;

            % check for C++
            if mycontext.isMDL
                lang = get_param(mycontext.modelName,'TargetLang');
                assert(isequal(lang,'C++'), ...
                       "ros:slros:cgen:CppLanguageRequired", ...
                       getString(message("ros:slros:cgen:CppLanguageRequired",lang)));
                mycontext.isROSControlBuild = ros.codertarget.internal.Util.isROSControlEnabled(mycontext.modelName);
                % generate code-only if ROS Control generation is enabled
                % or user has explicitly set GenerateCodeOnly flag
                genCodeOnly = isequal(get_param(mycontext.modelName,'GenCodeOnly'), 'on');
            else
                % Otherwise MATLAB Coder
                genCodeOnly = false;
            end
            % code-generator changes the current working folder to build
            % folder

            pjtData = getProjectData(h);
            if ~isempty(pjtData)
                mycontext.ProjectData = pjtData;
            end
            generateCodeOnly = h.isRemoteBuild(mycontext) || genCodeOnly || ...
                mycontext.isROSControlBuild;
            pkgName = h.getValidPackageName(mycontext.modelName);
            mycontext.projectBuilder = h.getProjectBuilder(mycontext.anchorDir, pkgName, 'GenCodeOnly', generateCodeOnly);
            context = mycontext;
        end

        function [ret, context] = createProject(h, buildInfo, context, varargin)
            import ros.codertarget.internal.Util
            type = varargin{2};
            comp = varargin{3};
            if type == coder.make.enum.BuildOutput.EXECUTABLE
                context.isLib            = false;
                context.isSharedUtil     = false;
            elseif type == coder.make.enum.BuildOutput.STATIC_LIBRARY
                context.isLib            = true;
                context.isSharedUtil     = ~isempty(strfind(comp.FinalProduct.TargetInfo.TargetFile, 'rtwshared')); %TODO CHECK
            else
                assert(false);
            end

            context.libs = {};
            context.Libraries = {};
            context.LibraryDirectories = {};
            libSize = length(comp.Libraries);


            if libSize > 1
                libs = comp.Libraries;
            elseif libSize == 1
                libs = {comp.Libraries};
            end

            archiveName = [context.modelName,'.tgz'];
            sDir = getSourcePaths(buildInfo, true, {'StartDir'});
            if isempty(sDir)
                sDir = {pwd};
            end
            archive = fullfile(sDir{1}, archiveName);

            if h.skipProjectGeneration(h.isRemoteBuild(context),archive,context)
                ret  = h.getValidPackageName(context.modelName);
                % skip project and archive generation if the generated
                % code has not changed
                disp(message('ros:slros:deploy:ArchiveUpToDate',context.modelName,archive).getString);
                return
            end

            % get pre-compiled libraries
            [context, linkOnlyObjs, preCompiledObjs] = Util.addLinkObjects(buildInfo,context);

            for i=1:libSize
                libstruct = libs{i};
                if ~isempty(libstruct.value)
                    % iterate over the libraries contained in value
                    % value is a cell-array of libraries
                    % We need to revisit this code when PIL/SIL is supported
                    % For SIL/PIL, we need to also consider the 'Type' field of
                    % libstruct.
                    for numLibs = 1:numel(libstruct.value)
                        pathToLib = buildInfo.formatPaths(libstruct.value{numLibs}, 'replaceStartDirWithRelativePath', true);
                        if ispc && endsWith(pathToLib,{'.o','.obj','.O','.OBJ'})
                            % Convert 8.3 path name to full path name
                            [~,libName,ext] = fileparts(builtin('_canonicalizepath',libstruct.value{numLibs}));
                        else
                            [~,libName,ext] = fileparts(libstruct.value{numLibs});
                        end
                        if  isequal(libName,'rtwshared') || ...
                                ismember([libName, ext], ...
                                         {preCompiledObjs.Name, linkOnlyObjs.Name})
                            % Shared utility source files are added to the
                            % referenced model project directly
                            continue;
                        end
                        if(coder.make.internal.isRelativePath(pathToLib))
                            context.libs{end+1} = fullfile('..',pathToLib);
                        else
                            context.libs{end+1} = fullfile(pathToLib);
                        end
                    end
                end
            end

            mdlName = buildInfo.getBuildName;
            %the following should always be true
            %verifying that they are and no accidents happened
            assert(isequal(context.modelName, mdlName));
            pkgName = h.getValidPackageName(mdlName);
            pkgInfo = context.projectBuilder.getPackageInfo(pkgName);
            pkgInfo = h.setPackageInfoFromContext(pkgInfo, context);
            if strcmp(context.ActiveBuildConfiguration,'Specify')
                pkgInfo = h.setPackageInfoFromToolchain(pkgInfo, comp.ToolchainFlags);
            end
            % Create a cmake pkg-config parser object
            pkgInfo = Util.addPackageConfigModules(buildInfo,pkgInfo);

            % ignoreParseError converts parsing errors from
            % findIncludeFiles into warnings
            warnState = warning('off','RTW:buildInfo:unableToFindMinimalIncludes');
            try
                findIncludeFiles(buildInfo, ...
                                 'extensions', {'*.h' '*.hpp'}, ...
                                 'ignoreParseError', true);
            catch EX
                warning(warnState);
                rethrow(EX);
            end
            warning(warnState);

            defines = buildInfo.getDefines();
            [sharedSrcFiles, sharedIncFiles] = h.getSharedUtilsSources(buildInfo);
            srcFiles = unique([buildInfo.getSourceFiles(true,true), sharedSrcFiles]);
            incFiles = unique([buildInfo.getIncludeFiles(true,true), sharedIncFiles]);
            otherFiles = buildInfo.getFiles('other',true,true);
            % Remove defines.txt from the list
            otherFiles(contains(otherFiles,'defines.txt')) = [];
            % Remove mwboost - MATLAB boost SO/DYLIB/DLLs from being packaged
            otherFiles(contains(otherFiles,'mwboost_')) = [];

            % Cleanup include paths
            incPaths = buildInfo.getIncludePaths(true);
            incPaths = replace(incPaths,context.projectBuilder.RootDir,''); %remove root dir
            buildDirList = buildInfo.getBuildDirList;
            for i = 1:numel(buildDirList)
                incPaths = replace(incPaths,buildDirList{i},'');
                [~,buildDir,~] = fileparts(buildDirList{i});
                incPaths = replace(incPaths,['/',buildDir],'');
            end
            installdirWithForwardSlash = replace(matlabroot,'\','/');
            incPathsWithInstallRoot = cellfun(@(x)startsWith(x,installdirWithForwardSlash),incPaths);
            incPaths(incPathsWithInstallRoot) = [];
            incPaths = incPaths(~cellfun(@isempty,incPaths)); %remove empty values

            concatDefines = sprintf('%s\n  ', defines{:});
            % get compile flags
            compFlags = getCompileFlags(buildInfo);
            concatCompileFlags = sprintf('%s  ', compFlags{:});
            pkgInfo.CppFlags = [pkgInfo.CppFlags concatDefines];
            if ~any(contains(srcFiles,'.cu'))
                % Append C++ compile flags from buildInfo for non-GPU code-gen only
                pkgInfo.CppFlags = [pkgInfo.CppFlags concatCompileFlags];
            end

            if isfield(context, 'ProjectData') && ...
                    isfield(context.ProjectData, 'GPUFlags')
                if isempty(context.ProjectData.GPUFlags)
                    pkgInfo.CUDAFlags = '-arch sm_35';
                else
                    pkgInfo.CUDAFlags = context.ProjectData.GPUFlags;
                end
            else
                pkgInfo.CUDAFlags = '-arch sm_35';
            end

            mdlRefLibraries = {};
            if context.isMDL
                % Exclude slros_busmsg_conversion.h and slros_initialize.h
                % include files as each Simulink model can contain
                % different message types and ROS/ROS2 blocks
                commonMdlRefIncludes = {};
                commonFilesToExclude = {'slros_busmsg_conversion', 'slros_initialize'};
                modelRefNames = ros.codertarget.internal.Util.uniqueModelRefNames(buildInfo);
                if ~isempty(modelRefNames)
                    % Get all model ref paths
                    allMdlRefPaths = arrayfun(@(b)b.formatPaths(b.Path),buildInfo.ModelRefs,'UniformOutput',false);
                    % Get all model ref include files; for windows incFiles
                    % list has a mixture of '\' and '/' file-separators, so
                    % replace all '\' with '/'
                    mdlRefIncFiles = incFiles(contains(...
                        strrep(incFiles,'\','/'),...
                        strrep(allMdlRefPaths,'\','/')));
                    % Filter busmsg conversion files and slros_initialize
                    % files from those mdlRefIncFiles
                    commonMdlRefIncludes = mdlRefIncFiles(contains(mdlRefIncFiles, ...
                        commonFilesToExclude)); 
                end
                incFiles = setdiff(incFiles, commonMdlRefIncludes);
                if (ros.codertarget.internal.Util.isTopLevelModel(buildInfo)) ...
                        && h.LinkReferenceLibraries
                    mdlRefLibraries = modelRefNames;
                end
            end

            if context.isROSControlBuild
                pkgInfo = ros.codertarget.internal.Util.setROSControlPkgInfo(mdlName,pkgInfo,srcFiles,incFiles);
            else
                if context.isLib
                    pkgInfo.LibSourceFiles = srcFiles;
                    pkgInfo.LibIncludeFiles = incFiles;
                    pkgInfo.CppLibraryName = mdlName;
                    if h.isRemoteBuild(context)
                        pkgInfo.LibFormat = '     ';
                    else
                        pkgInfo.LibFormat = 'STATIC';
                    end
                else
                    pkgInfo.SourceFiles = srcFiles;
                    pkgInfo.IncludeFiles = incFiles;
                    pkgInfo.CppNodeName = mdlName;
                end
            end

            if ~isempty(otherFiles)
                pkgInfo.OtherFiles = otherFiles;
            end
            if ~isempty(incPaths)
                pkgInfo.IncludeDirectories = unique([pkgInfo.IncludeDirectories incPaths]);
            end
            %add dependencies based on context.libs
            if ~isempty(context.libs)
                [~,libNames,~] = cellfun(@(x)fileparts(x),context.libs,'UniformOutput',false);
                otherPkgs = replace(libNames,'_rtwlib','');
                pkgInfo.Dependencies = h.getValidPackageName(otherPkgs);
                %we do not expect any message dependencies as they had to be specified during rosXgenmsg
                %If there any they will come through when we build msgDepends
                %hence we are not adding them to MsgDependencies
            end

            if ~isempty(context.Libraries)
                pkgInfo.Libraries = unique([pkgInfo.Libraries context.Libraries]);
            end

            if ~isempty(context.LibraryDirectories)
                pkgInfo.LibraryDirectories = unique([pkgInfo.LibraryDirectories context.LibraryDirectories]);
            end
            % Append extra model-ref libraries, if any
            if ~isempty(mdlRefLibraries)
                pkgInfo.Libraries =  [pkgInfo.Libraries mdlRefLibraries];
            end
            if isfield(context,'ImportedLibs') && ~isempty(context.ImportedLibs)
                pkgInfo.ImportedLibraries = unique([pkgInfo.ImportedLibraries, context.ImportedLibs]);
            end


            if isfield(context,'LinkerFlags') && ~isempty(context.LinkerFlags)
                pkgInfo.LinkerFlags = ...
                    strjoin(unique([pkgInfo.LinkerFlags, ...
                                    context.LinkerFlags]));
            end

            msgDepends = {};
            msgDataStructArr = {};
            imageDepends = {};
            actionDep = h.getActionDependencies(buildInfo.getIncludeFiles('',''));
            tfDep = h.getTransformationDependencies(buildInfo.getIncludeFiles('',''));
            if isfield(context,'ProjectData')
                msgDataStructArr = h.getMsgInfoFromProjectData(context.ProjectData);
                msgDepends = [msgDepends, cellfun(@(x)x.pkgName,msgDataStructArr, 'UniformOutput', false)];
                imageDepends = context.ProjectData.ImageDepends;
            end
            pkgInfo.Dependencies = unique([pkgInfo.Dependencies, msgDepends, h.DefaultDependencies{:}, imageDepends, actionDep, tfDep]);
            if ~isempty(msgDepends)
                pkgInfo.MsgDependencies = unique([pkgInfo.MsgDependencies, msgDepends, actionDep]);
            end
            context.projectBuilder.updatePackage(pkgInfo);

            %now iterate through all publishers and subscribers and get any
            %custom messages
            customMsgMap = containers.Map(); %key: pkgName and value cell array of msgSrcs
            msgDependsMap = containers.Map(); %key: pkgName and value cell array of msg names that this model depends on in that package
            registry = ros.internal.CustomMessageRegistry.getInstance(h.ROSVersion);

            if isfield(context,'ProjectData') && ~isempty(msgDataStructArr)
                for i = 1:numel(msgDataStructArr)
                    msgDependsMap = ros.internal.utilities.findMsgDepends(...
                        [msgDataStructArr{i}.pkgName,'/',msgDataStructArr{i}.msgName], ...
                        msgDependsMap, ...
                        h.ROSVersion);
                end
                if isKey(msgDependsMap,'ros')
                    % ros/Time and ros/Duration are pseudo messages
                    msgDependsMap.remove('ros');
                end
                %from this get all dependent packages and messages
                depPkgs = msgDependsMap.keys;
                depMsgsWithPkgNames = {};
                for i = 1:numel(depPkgs)
                    depMsgsWithPkgNames = unique([depMsgsWithPkgNames cellfun(@(msgName)[depPkgs{i},'/',msgName], msgDependsMap(depPkgs{i}),'UniformOutput', false)]);
                end
                %append to allMsgInfo. This is super set of all msgInfos
                allMsgInfos = [msgDataStructArr cellfun(@(x)ros.internal.(h.ROSVersion).getMessageInfo(x, registry), depMsgsWithPkgNames,'UniformOutput',false)];
                customIdx = cellfun(@(x)x.custom,allMsgInfos,'UniformOutput',false);
                allCustomMsgInfos = allMsgInfos(cell2mat(customIdx));
                for i = 1:numel(allCustomMsgInfos)
                    % add the packageName as key and message source as values
                    pkgName = allCustomMsgInfos{i}.pkgName;
                    msgSrc = allCustomMsgInfos{i}.srcPath;
                    if ~isempty(customMsgMap) && customMsgMap.isKey(pkgName)
                        curVal = customMsgMap(pkgName);
                        customMsgMap(pkgName) = unique([curVal, msgSrc]);
                    else
                        customMsgMap(pkgName) = { msgSrc };
                    end
                end
            end

            %for each entry in map, add a package with src
            pkgNames = customMsgMap.keys();
            for i = 1:numel(pkgNames)
                %for each package we should really look at the message and
                %recalculate the package dependencies.
                msgDependsMapForThisCustomPkg = containers.Map();
                msgsInThisCustomPkg = customMsgMap(pkgNames{i});
                % separate message and service files
                %from this get all dependent packages and messages
                for j = 1:numel(msgsInThisCustomPkg)
                    [~,msgName,ext] = fileparts(msgsInThisCustomPkg{j});
                    if strcmp(ext,'.srv')
                        reqMsg = [pkgNames{i},'/',msgName,'Request'];
                        respMsg = [pkgNames{i},'/',msgName,'Response'];
                        msgDependsMapForThisCustomPkg = ...
                            ros.internal.utilities.findMsgDepends(reqMsg,...
                                                                  msgDependsMapForThisCustomPkg, h.ROSVersion);
                        msgDependsMapForThisCustomPkg = ...
                            ros.internal.utilities.findMsgDepends(respMsg,...
                                                                  msgDependsMapForThisCustomPkg, h.ROSVersion);
                    else
                        fullMsgName = [pkgNames{i},'/',msgName];
                        msgDependsMapForThisCustomPkg = ...
                            ros.internal.utilities.findMsgDepends(fullMsgName,...
                                                                  msgDependsMapForThisCustomPkg, h.ROSVersion);
                    end

                end
                if isKey(msgDependsMapForThisCustomPkg,'ros')
                    % ros/Time and ros/Duration are pseudo messages
                    msgDependsMapForThisCustomPkg.remove('ros');
                end
                dependencies = setdiff(msgDependsMapForThisCustomPkg.keys,pkgNames{i});
                idx = context.projectBuilder.findPackage(pkgNames{i});
                if ~isempty(idx)
                    pkgInfoMsg = context.projectBuilder.getPackage(pkgNames{i});
                    pkgInfoMsg = h.addMsgSources(pkgInfoMsg, msgsInThisCustomPkg);
                    if ~isempty(dependencies)
                        pkgInfoMsg.Dependencies = unique([pkgInfoMsg.Dependencies, dependencies]);
                        pkgInfoMsg.MsgDependencies = unique([pkgInfoMsg.MsgDependencies, dependencies]);
                    end
                    context.projectBuilder.updatePackage(pkgInfoMsg);
                else
                    pkgInfoMsg = ros.internal.PackageInfo(pkgNames{i});
                    pkgInfoMsg = h.addMsgSources(pkgInfoMsg, msgsInThisCustomPkg);
                    if ~isempty(dependencies)
                        pkgInfoMsg.Dependencies = unique([pkgInfoMsg.Dependencies, dependencies]);
                        pkgInfoMsg.MsgDependencies = unique([pkgInfoMsg.MsgDependencies, dependencies]);
                    end
                    context.projectBuilder.addPackage(pkgInfoMsg);
                end
            end

            %ensure we are not dependent on ourselves, msg could be in the
            %same package and ensure we are not adding pkgs that are
            %already in dep list
            pkgNamesFlt = pkgNames(~matches(pkgNames,pkgInfo.PackageName));
            if ~isempty(pkgNamesFlt)
                pkgInfo.Dependencies = unique([pkgInfo.Dependencies, pkgNamesFlt]);
                pkgInfo.MsgDependencies = unique([pkgInfo.MsgDependencies, pkgNamesFlt]);
            end
            context.pkgsToBuild = [pkgNamesFlt, pkgInfo.PackageName];

            context.projectBuilder.createPackage([],true); %force create package
            ret = pkgName;
            
            % create archive for remote build
            if context.isROSControlBuild
                bDir = getSourcePaths(buildInfo,true,{'BuildDir'});
                if isempty(bDir)
                    bDir = {pwd};
                end
                ros.codertarget.internal.Util.copyControllerPluginFiles(...
                    context.anchorDir,...
                    h.getValidPackageName(mdlName),...
                    bDir{1});
            end
            

            % create archive for remote build
            if h.isRemoteBuild(context)
                % The zip file will be put in the start dir, the same as the final
                % executable or dll.
                disp(message('ros:slros:deploy:CreateArchiveFile', mdlName).getString);
                disp(ros.slros.internal.diag.DeviceDiagnostics.StepSeparator);
                tar(archive,fullfile(context.anchorDir,'src',{h.getValidPackageName(mdlName)}));
                disp(message('ros:slros:deploy:ArchiveTargetFile', archive).getString);
                % Copy build_model.sh to the same directory as the archive file
                scriptName = ['build_',lower(h.ROSVersion),'_model.sh'];
                targetScript = fullfile(sDir{1}, scriptName);
                scriptLoc = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
                copyfile(fullfile(scriptLoc,scriptName), targetScript, 'f');
                disp(message('ros:slros:deploy:ShellScriptTargetFile', targetScript).getString);
                disp(ros.slros.internal.diag.DeviceDiagnostics.StepSeparator);
                disp(message('ros:slros:deploy:CopyFilesToTarget').getString);
            end
        end

        function [ret, context] = buildProject(h, buildInfo, context, varargin)
            if h.isRemoteBuild(context)
                % Use LoadCommand and LoadCommandArgs from TargetSDK for
                % RemoteBuild
                % Skip buildProject
                ret = 'Success';
                return;
            end
            mdlName = buildInfo.getBuildName;
            if isequal(mdlName,'rtwshared')
                sharedutilsdir = ros.codertarget.internal.Util.sharedUtilsDir(buildInfo, true);
                % Create a dummy rtwshared.a file to avoid build errors
                sharedLibFile = fopen(fullfile(sharedutilsdir, 'rtwshared.a'),'w');
                fclose(sharedLibFile);
                ret = 'Success';
                return;
            end

            %the following should always be true
            %verifying that they are and no accidents happen
            assert(isequal(context.modelName, mdlName));
            pkgName = h.getValidPackageName(mdlName);
            pkgInfo = context.projectBuilder.getPackageInfo(pkgName); %#ok<NASGU>

            % Execute build command
            [res, installDir] = runBuildCommand(h, context);%#ok<ASGLU> %TO Consider: cat res into ret to show
            if context.isLib
                %we need to copy back to the current directory
                srcFileMap = containers.Map({'win64','maci64','glnxa64'}, ...
                    {fullfile(installDir,'lib',[mdlName,'.lib']), ... for windows
                    fullfile(installDir,'lib',['lib',mdlName,'.a'])', ... for mac
                    fullfile(installDir,'lib',['lib',mdlName,'.a'])}); ... for linux
                    if context.isMDL
                        destPath = fullfile(pwd,[mdlName,'_rtwlib.a']);
                    else
                        destPath = fullfile(pwd,[mdlName,'.a']);
                    end
                    [status, msg, msgId] = copyfile(srcFileMap(computer('arch')), ...
                        destPath);
                    if ~status
                        error(msgId,msg);
                    end
            end
            ret = 'Success';
        end

        function [ret, context] = downloadProject(h, buildInfo, context, varargin) %#ok<INUSL>
            ret = true;
            %TODO
        end

        function [ret, context] = runProject(h, buildInfo, context, varargin)%#ok<INUSL>
            ret = true;
            %TODO
        end

        function [ret, context] = onError(h, buildInfo, context, varargin)%#ok<INUSL>
            ret = true;
            try
                if ~context.isSharedUtil
                end
            catch %ignore all errors
            end
        end

        function [ret, context] = terminate(h, buildInfo, context, varargin) %#ok<INUSD>
            ret = true;
            context = [];
        end
    end

    methods (Static, Hidden)
        function skipGen =  skipProjectGeneration(isRemoteBuild, archive, context)
        % SKIPPROJECTGENERATION Skip generation of project and archive
        % if there was no structural change made to the Simulink model.
        % This method returns 'false' for MATLAB Coder ROS and ROS 2
        % projects.

            if isfield(context, 'ProjectData') && ...
                    isfield(context.ProjectData, 'HasCodeChanged')
                skipGen = ~context.ProjectData.HasCodeChanged && ...
                          isRemoteBuild && isfile(archive);
            else
                skipGen = false;
            end
        end

        function pkgInfoMsg = addMsgSources(pkgInfoMsg, msgSources)
            msgFiles = msgSources(endsWith(msgSources,'.msg'));
            srvFiles = msgSources(endsWith(msgSources,'.srv'));
            actFiles = msgSources(endsWith(msgSources,'.action'));
            if ~isempty(msgFiles)
                pkgInfoMsg.MessageFiles = [pkgInfoMsg.MessageFiles,msgFiles];
            end
            if ~isempty(srvFiles)
                pkgInfoMsg.ServiceFiles = [pkgInfoMsg.ServiceFiles, srvFiles];
            end
            if ~isempty(actFiles)
                pkgInfoMsg.ActionFiles = [pkgInfoMsg.ActionFiles, actFiles];
            end
        end

        function pkgInfo = setPackageInfoFromToolchain(pkgInfo, tcFlags)
        % SETPACKAGEINFOFROMTOOLCHAIN Append the Custom toolchain
        % options set by user through the "Specify" option

            pkgInfo.BuildType = tcFlags.getValue('Build Type').custom.value;
            appendFlagsAsStrings('Defines','CppFlags');
            appendFlagsAsCellArrays('Include Directories','IncludeDirectories');
            appendFlagsAsCellArrays('Link Libraries','Libraries');
            appendFlagsAsCellArrays('Library Paths','LibraryDirectories');
            appendFlagsAsCellArrays('Required Packages','Dependencies');

            function appendFlagsAsStrings(itemName, pkgInfoProp)
                if ~isempty(tcFlags.getValue(itemName).custom.value)
                    orig = pkgInfo.(pkgInfoProp);
                    pkgInfo.(pkgInfoProp) = sprintf("%s %s",[orig, ...
                                                             tcFlags.getValue(itemName).custom.value]);
                end
            end
            function appendFlagsAsCellArrays(itemName, pkgInfoProp)
                if ~isempty(tcFlags.getValue(itemName).custom.value)
                    orig = pkgInfo.(pkgInfoProp);
                    pkgInfo.(pkgInfoProp) = unique([orig, ...
                                                    strip(split(tcFlags.getValue(itemName).custom.value)')]);
                end
            end
        end

        function ret = isRemoteBuild(context)
        % ISREMOTEBUILD Returns the value of RemoteBuild field in the
        % context ROS Project data structure. Returns FALSE if the
        % field is not present or value of the field is not a boolean
            if isfield(context, 'ProjectData') && ...
                    isfield(context.ProjectData, 'RemoteBuild')
                ret = context.ProjectData.RemoteBuild;
            else
                ret = false;
            end
        end


        function allMsgInfos = getMsgInfoFromProjectData(projectData)
            allValues = {};
            if isfield(projectData,'Publishers') && (projectData.Publishers.length ~= 0)
                allValues = [allValues projectData.Publishers.values];
            end
            if isfield(projectData,'Subscribers') && (projectData.Subscribers.length ~= 0)
                allValues = [allValues projectData.Subscribers.values];
            end
            allMsgInfos = cellfun(@(x)x.msgInfo,allValues,'UniformOutput',false);
            if isfield(projectData, 'ServiceCallers') && (projectData.ServiceCallers.length ~= 0)
                reqMsgInfo = cellfun(@(x)[x.Request.msgInfo], ...
                                     projectData.ServiceCallers.values,'UniformOutput',false);
                respMsgInfo = cellfun(@(x)[x.Request.msgInfo], ...
                                      projectData.ServiceCallers.values,'UniformOutput',false);
                allMsgInfos = [allMsgInfos reqMsgInfo respMsgInfo];
            end
            if isfield(projectData, 'MessageInfoArray')
                allMsgInfos = [allMsgInfos, projectData.MessageInfoArray];
            end
        end
        function [sharedSrcFiles, sharedIncFiles] = getSharedUtilsSources(buildInfo)
        %GETSHAREDUTILSSOURCES Gather shared-utility source files for a
        % input 'buildInfo' argument of a referenced Simulink model
            sharedutilsdir = ros.codertarget.internal.Util.sharedUtilsDir(buildInfo, true);
            if ~isempty(sharedutilsdir)
                % Gather source files
                sharedSrcInfo = dir(fullfile(sharedutilsdir, '*.c*'));
                sharedSrcFiles = fullfile(sharedutilsdir, {sharedSrcInfo.name});
                % Gather header files
                sharedHeaderInfo = dir(fullfile(sharedutilsdir, '*.h'));
                sharedHppHeaderInfo = dir(fullfile(sharedutilsdir, '*.hpp'));
                sharedHeaders = {sharedHeaderInfo.name sharedHppHeaderInfo.name};
                sharedIncFiles = fullfile(sharedutilsdir, sharedHeaders);
            else
                sharedSrcFiles = {};
                sharedIncFiles = {};
            end
        end

        function pkgInfo = setPackageInfoFromContext(pkgInfo, context)
        %SETPACKAGEINFOFROMCONTEXT Set the package information for generation of colcon build
        % project from the Simulink model or MATLAB code-generation project
            if context.isMDL
                % Update package information from the model
                data = codertarget.data.getData(getActiveConfigSet(context.modelName));
                if ~isempty(data) && isfield(data, 'Packaging')
                    % read the packaging details from codertarget data if the hardware board
                    % is set to "Robot Operating System 2 (ROS 2)", else
                    pkgInfo.MaintainerName = data.Packaging.MaintainerName;
                    pkgInfo.MaintainerEmail = data.Packaging.MaintainerEmail;
                    pkgInfo.Version = data.Packaging.Version;
                    pkgInfo.License = data.Packaging.License;
                end
            else
                if isfield(context,'ProjectData')
                    % Ensures build works with Colcon builder with no
                    % MATLAB target selected
                    data = context.ProjectData.PackageInformation;
                    pkgInfo.MaintainerName = data.MaintainerName;
                    pkgInfo.MaintainerEmail = data.MaintainerEmail;
                    pkgInfo.Version = data.Version;
                    pkgInfo.License = data.License;
                end
            end
        end

        function ret = getValidPackageName(val)
        %GETVALIDPACKAGENAME Get a valid package name for a given character
        %vector
            val = convertStringsToChars(val);
            ret = lower(val);
        end
    end

end

% LocalWords:  sharedutils SDK Xgenmsg pkgs dep maci ISREMOTEBUILD GETSHAREDUTILSSOURCES
% LocalWords:  SETPACKAGEINFOFROMCONTEXT GETVALIDPACKAGENAME
