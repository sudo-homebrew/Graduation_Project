classdef Util
    %This class is for internal use only. It may be removed in the future.

    %codertarget.Util - Utility functions related to generating ROS node
    %                   using Coder Target infrastructure

    %   Copyright 2014-2022 The MathWorks, Inc.

    methods(Static)

        function pkgName = modelNameToValidPackageName(modelName)
            %modelNameToValidPackageName Convert model name to ROS package name
            %   ROS package names should start with a lower case letter and
            %   only contain lower case letters, digits and underscores.

            validateattributes(modelName, {'char'}, {'nonempty'});

            % Make model name lowercase. Then remove all non-word
            % characters (\W regexp), i.e. keep only alphabet characters,
            % underscores & numbers.
            pkgName = regexprep(lower(modelName), '\W', '');
            if isempty(pkgName)
                error(message('ros:slros:cgen:UnableToCreateROSPkgName', modelName));
            end
        end

        function isValid = isValidPackageVersion(versionStr)
            %isValidPackageVersion Verify that ROS package version is valid
            %   Required to be 3 dot-separated integers. See
            %   http://wiki.ros.org/catkin/package.xml.

            % Explicitly make sure that input string is non-empty, since
            % regular expression would match an empty string and falsely
            % return TRUE.
            validateattributes(versionStr, {'char'}, {'nonempty'});
            isValid = strcmpi(versionStr, regexp(versionStr, '^\d+\.\d+\.\d+$', 'match', 'once'));
        end

        function isTopLevel = isTopLevelModel(buildInfo)
            %isTopLevelModel Determine if a given model is a top-level or referenced model
            %   ISTOPLEVELMODEL(BUILDINFO) returns TRUE if the model
            %   represented by BUILDINFO is a top-level model.
            %
            %   ISTOPLEVELMODEL(BUILDINFO) returns FALSE if the model is
            %   used as a referenced model

            validateattributes(buildInfo, {'RTW.BuildInfo'}, {'scalar'}, 'isTopLevelModel', 'buildInfo');

            [~, buildArgValue] = findBuildArg(buildInfo, 'MODELREF_TARGET_TYPE');
            isTopLevel = strcmp(buildArgValue, 'NONE');
        end

        function isTopLevel = isExternalModeBuild(buildInfo)
            %isExternalModeBuild Determine if a the build process is generating
            %code for External mode
            %
            %   ISEXTERNALMODEBUILD(BUILDINFO) returns TRUE if the model
            %   represented by BUILDINFO is generating External mode code
            %

            validateattributes(buildInfo, {'RTW.BuildInfo'}, {'scalar'}, 'isTopLevelModel', 'buildInfo');

            [~, buildArgValue] = findBuildArg(buildInfo, 'EXT_MODE');
            isTopLevel = strcmp(buildArgValue, '1');
        end


        function [isSingleTasking, sampleTimes, hasExplicitPartitions] = getSchedulerData(mdlName, buildDir)
            % GETSCHEDULERDATA Determine the scheduling and tasking policy
            % of the Simulink model based on the Solver parameters.
            %
            % isSingleTasking = GETSCHEDULERDATA(mdlName) returns TRUE if
            % the EnableMultiTasking and ConcurrentTasks parameters are
            % false.
            %
            % [isSingleTasking,sampleTimes] = GETSCHEDULERDATA(mdlName)
            % also returns an array sampleTimes containing all the discrete
            % sample times in the Simulink model.
            arguments
                mdlName {mustBeNonzeroLengthText}
                buildDir {mustBeFolder}
            end
            hasExplicitPartitions = isequal(get_param(mdlName,'ExplicitPartitioning'),'on') ...
                && isequal(get_param(mdlName,'ConcurrentTasks'),'on');
            if hasExplicitPartitions
                % Create sample time structure for periodic tasks
                % specified explicit partitions
                cInfo = load(fullfile(buildDir,"codeInfo.mat"));
                numExplicitTasks = numel(cInfo.codeInfo.OutputFunctions);
                sampleTimes = repmat(struct,numExplicitTasks,1);
                for k=1:numExplicitTasks
                    fcnTiming = cInfo.codeInfo.OutputFunctions(k).Timing;
                    if isequal(fcnTiming.TimingMode,'PERIODIC')
                        sampleTimes(k).Value =[fcnTiming.SamplePeriod fcnTiming.SampleOffset];
                        sampleTimes(k).TID = k;
                        sampleTimes(k).Description = strrep(fcnTiming.NonFcnCallPartitionName, 'D','Discrete ');
                    else
                        sampleTimes(k) = [];
                    end
                end
            else
                allSampleTimes = get_param(mdlName,'SampleTimes');
                % Get all discrete sample times, filter out based on the
                % annotation property as it starts with D*
                discrIndx = arrayfun(@(x)startsWith(x.Annotation,'D'),allSampleTimes);
                sampleTimes = allSampleTimes(discrIndx);
            end
            % the model is single tasking if one of the following is true:
            % 1. The 'EnableMultiTasking' setting is 'off' AND
            % 'ConcurrentTasks' is 'off' OR
            % 2. There is only one discrete sample time AND explicit
            % partitions is not enabled
            isSingleTasking = (isequal(get_param(mdlName,'EnableMultiTasking'),'off') || ...
                (numel(sampleTimes) == 1)) && ~hasExplicitPartitions;
        end
        

        
        function [modelRefNames, refNodeInfo] = uniqueModelRefNames(buildInfo)
            %uniqueModelRefNames Get names of all model references in model
            %   MODELREFNAMES = uniqueModelRefNames(BUILDINFO) returns the
            %   names of all model references listed in the BUILDINFO for
            %   the current model. The list of MODELREFNAMES will only
            %   contain unique entries.

            validateattributes(buildInfo, {'RTW.BuildInfo'}, {'scalar'}, 'uniqueModelRefNames', 'buildInfo');
            modelRefNames = {};
            refNodeInfo = {};
            if ~isempty(buildInfo.ModelRefs)
                modelRefPaths = arrayfun(@(ref)formatPaths(ref, ref.Path), buildInfo.ModelRefs, 'UniformOutput', false);
                modelRefNames = cell(1,numel(modelRefPaths));
                refNodeInfo = cell(1,numel(modelRefPaths));
                for i = 1:numel(modelRefPaths)
                    thisFullPath = modelRefPaths{i};
                    [~, modelRefNames{i}, ~] = fileparts(thisFullPath);
                    modelInfoFile = fullfile(thisFullPath, 'rosModelInfo.mat');
                    if isfile(modelInfoFile)
                        info = load(modelInfoFile);
                        if isfield(info.rosProjectInfo, 'RefModelNodeInfo')
                            refNodeInfo{i} = info.rosProjectInfo.RefModelNodeInfo;
                        else
                            refNodeInfo{i} = struct('nodeDependencies',{});
                        end
                    end
                end

                % Only find the unique model reference names
                modelRefNames = unique(modelRefNames, 'stable');
            end
        end

        function sharedDir = sharedUtilsDir(buildInfo, isAbsolute)
            %sharedUtilsDir Retrieve relative or absolute path to shared utility sources
            %    SHAREDDIR = sharedUtilsDir(BUILDINFO, false) returns the
            %    path to the shared utility folder relative the current
            %    folder (pwd). If no shared utility folder exists, return
            %    ''.
            %
            %    SHAREDDIR = sharedUtilsDir(BUILDINFO, true) returns the
            %    absolute path to the utility folder.
            sharedDir = '';
            for i = 1:length(buildInfo.BuildArgs)
                if strcmpi(buildInfo.BuildArgs(i).Key, 'SHARED_SRC_DIR')
                    sharedDir = strtrim(buildInfo.BuildArgs(i).Value);
                    break;
                end
            end
            if ~isempty(sharedDir) && isAbsolute
                % Convert relative path to absolute path if needed
                sharedDir = ros.internal.FileSystem.relativeToAbsolute(sharedDir);
            end
        end

        function pkgInfo = addPackageConfigModules(buildInfo,pkgInfo)
            % ADDPKGCONFIGMODULES Parse build-info and add pkg-config modules to
            % PackageInfo
            % The buildInfo link flags that contain
            %
            % Example:
            %   import ros.codertarget.internal.Util
            %   load('packageInfo.mat') % load a pre-generated package info
            %   pkgInfo = Util.addPackageConfigModules(buildInfo,pkgInfo)
            %
            % See also ROS.INTERNAL.PACKAGEINFO


            % Copyright 2021 The MathWorks, Inc.

            import ros.codertarget.internal.Util

            validateattributes(buildInfo,{'RTW.BuildInfo'},{'nonempty'});
            validateattributes(pkgInfo,{'ros.internal.PackageInfo'},{'nonempty'});

            pkgCfgModules = Util.getPkgConfigModules(buildInfo);
            if ~isempty(pkgCfgModules)
                cmkPkgCfg = Util.getPkgConfigCMakeOptions(pkgCfgModules);
                pkgInfo.CppFlags = [pkgInfo.CppFlags sprintf('%s ',cmkPkgCfg.cflags)];
                pkgInfo.PkgConfigModules = pkgCfgModules;
                pkgInfo.IncludeDirectories = unique([pkgInfo.IncludeDirectories {cmkPkgCfg.includedirs}]);
                pkgInfo.Libraries = unique([pkgInfo.Libraries {cmkPkgCfg.libraries}]);
                pkgInfo.LibraryDirectories = unique([pkgInfo.LibraryDirectories {cmkPkgCfg.librarydirs}]);
            end
        end

        function cmakeOpts = getPkgConfigCMakeOptions(moduleNames)
            % L_GETPKGCONFIGCMAKEOPTIONS Get CMake options for a cell-array
            % of modules
            %
            % Example:
            %    import ros.codertarget.internal.Util
            %    cmakeOpts = Util.getPkgConfigCMakeOptions({'opencv'});
            %
            %    cmakeOpts =
            %
            %    struct with fields:
            %
            %       pkgsearch: 'pkg_check_modules(OPENCV REQUIRED opencv)'
            %       libraries: '${OPENCV_LIBRARIES}'
            %     includedirs: '${OPENCV_INCLUDE_DIRS}'
            %     librarydirs: '${OPENCV_LIBRARY_DIRS}'
            %          cflags: '${OPENCV_CFLAGS}'

            moduleNames = convertCharsToStrings(moduleNames);
            validateattributes(moduleNames,{'string'},{'row','nonempty'},...
                'getPkgConfigCMakeOptions','moduleNames');
            cmakeOptions = {'pkgsearch','libraries','includedirs','librarydirs','cflags'};
            cmakeOpts = repmat(cell2struct(cell(1,numel(cmakeOptions)),...
                cmakeOptions,2),1,length(moduleNames));

            for k=1:length(moduleNames)
                cmkPkgVar = upper(matlab.lang.makeValidName(moduleNames{k}));
                cmakeOpts(k).pkgsearch = sprintf('pkg_check_modules(%s REQUIRED %s)',...
                    cmkPkgVar,moduleNames{k});
                cmakeOpts(k).cflags  = sprintf('${%s_CFLAGS}',cmkPkgVar);
                cmakeOpts(k).includedirs = sprintf('${%s_INCLUDE_DIRS}',cmkPkgVar);
                cmakeOpts(k).libraries = sprintf('${%s_LIBRARIES}',cmkPkgVar);
                cmakeOpts(k).librarydirs  = sprintf('${%s_LIBRARY_DIRS}',cmkPkgVar);
            end
        end

        function ret = getPkgConfigModules(buildInfo)
            % GETPKGCONFIGMODULES Extract the module names from link and
            % compile flags of buildInfo
            %
            % Example:
            %    import ros.codertarget.internal.Util;
            %    pkgs = Util.getPkgConfigModules(buildInfo);

            validateattributes(buildInfo,{'RTW.BuildInfo'},{'nonempty'},...
                'getPkgConfigModules','buildInfo');
            allLinkFlags = getLinkFlags(buildInfo);
            allCompileOpts = getCompileFlags(buildInfo);
            pkgCfgFlags = [allLinkFlags(contains(allLinkFlags,'pkg-config')), ...
                allCompileOpts(contains(allCompileOpts,'pkg-config'))];
            ret = cell(1,numel(pkgCfgFlags));
            for k=1:numel(pkgCfgFlags)
                % Split "$(XCOMPILERFLAG) `pkg-config --libs --cflags opencv4`" with spaces
                val = strsplit(pkgCfgFlags{k});
                % Trim the last back-tick character to obtain package name
                ret{k} = extractBefore(val{end},'`');
            end
            ret = unique(ret);
        end

        function [context, linkOnlyObjs, preCompiledObjs] =  addLinkObjects(buildInfo,context)
            % ADDLINKOBJECTS Add link objects to the context structure for use with ROS
            % package information and CMakeLists.txt and package.xml generation
            %
            % Example:
            %   context = struct;
            %   context = ros.codertarget.internal.addLinkObjects(buildInfo,context)

            % Copyright 2021 The MathWorks, Inc.

            import ros.codertarget.internal.ProjectTool
            isRemoteBuild = ProjectTool.isRemoteBuild(context);

            validateattributes(buildInfo,{'RTW.BuildInfo'},{'nonempty'});
            validateattributes(context,{'struct'},{'nonempty'});
            allLinkObjs = getLinkObjects(buildInfo);
            isPrecompiled = arrayfun(@(x)x.Precompiled,allLinkObjs);
            isLinkOnly = arrayfun(@(x)x.LinkOnly,allLinkObjs);
            isPrecompiled(isLinkOnly) = false;
            linkOnlyObjs = allLinkObjs(isLinkOnly);
            preCompiledObjs = allLinkObjs(isPrecompiled);

            context = l_addLinkOnlyObjects(linkOnlyObjs,buildInfo,context,isRemoteBuild);

            context = l_addPreCompiledObjects(preCompiledObjs,buildInfo,context);

            context = l_addLinkerFlags(buildInfo,context);

            context = l_addOptionsFromSysLibInfo(buildInfo,context,isRemoteBuild);
        end

        function ret = getROSControlInterfaces()
            jsonFile = fullfile(toolboxdir('ros'),'codertarget','templates','ros_control_interfaces.json');
            ret = jsondecode(fileread(jsonFile));
        end

        function ret = isROSControlEnabled(modelName)
            % ISROSCONTROLENABLED Returns true if the GenerateROSControl
            % flag of the model is set to true
            %
            % Example:
            %  import ros.codertarget.internal.Util
            %  Util.isROSControlEnabled(modelName)

            try
                ctdata = codertarget.data.getData(getActiveConfigSet(modelName));
                ret = isfield(ctdata,'ROS') && ...
                    isfield(ctdata.ROS,'GenerateROSControl') && ...
                    ctdata.ROS.GenerateROSControl;
            catch ME %#ok<NASGU> 
                % Ignore errors in case of no Simulink or Simulink Coder
                ret = false;
            end
        end

        function ret = getROSControlSettings(modelName)
            import ros.slros.internal.dlg.ROSControlSpecifier
            w = get_param(modelName,'ModelWorkspace');
            ret.InportTable = getDataFromWorkspace(w,ROSControlSpecifier.InportTableVarName);
            ret.OutportTable = getDataFromWorkspace(w,ROSControlSpecifier.OutportTableVarName);
            ret.ClassName = getDataFromWorkspace(w,ROSControlSpecifier.ClassNameVarName);
            function outval = getDataFromWorkspace(wkspc, varName)
                if hasVariable(wkspc,varName)
                    outval = evalin(wkspc,varName);
                else
                    outval = [];
                end
            end
        end

        function generateROSControlFiles(rosProjectInfo, bDir, buildInfo)
            import ros.codertarget.internal.Util
            modelInfo = Util.getROSControlProjectInfo(rosProjectInfo,bDir);
            templFolder = fullfile(toolboxdir('ros'),'codertarget','templates');
            templateFile = fullfile(templFolder,'controller_host.h.tmpl');
            
            % Generate <modelname>_ctrlr_host.cpp and <modelname>_ctrlr_host.h
            sourceFileName = [rosProjectInfo.ModelName,'_ctrlr_host'];
            loc_createOutput(modelInfo,templateFile,...
                fullfile(bDir,[sourceFileName,'.h']));
            templateFile = fullfile(templFolder,'controller_host.cpp.tmpl');
            loc_createOutput(modelInfo,templateFile,...
                fullfile(bDir,[sourceFileName,'.cpp']));
                
            % Generate controllers.xml for ros_control package
            templateFile = fullfile(templFolder,'controllers.xml.tmpl');
            loc_createOutput(modelInfo,templateFile,...
                fullfile(bDir,'controllers.xml'));
            % Generate controllers.yaml for ros_control package
            templateFile = fullfile(templFolder,'controllers.yaml.tmpl');
            loc_createOutput(modelInfo,templateFile,...
                fullfile(bDir,'controllers.yaml'));
            buildInfo.addIncludeFiles([sourceFileName,'.h']);
            buildInfo.addSourceFiles([sourceFileName,'.cpp']); 
            buildInfo.addDefines('_SL_ROS_CONTROL_PLUGIN_')
        end        
    
        function rosControlPrjInfo = getROSControlProjectInfo(rosProjectInfo,bDir)
            import ros.codertarget.internal.Util
            rosControlPrjInfo.ModelName = rosProjectInfo.ModelName;
            cinfo = load(fullfile(bDir,'codeInfo.mat'));
            rosControlPrjInfo.ModelClass = rosProjectInfo.ModelClassName;
            interfaces = Util.getROSControlInterfaces();
            rosctrlSettings = Util.getROSControlSettings(rosProjectInfo.ModelName);
            rosControlPrjInfo.ClassName = rosctrlSettings.ClassName;   
            rosControlPrjInfo.ProjectName = ...
                ros.codertarget.internal.ProjectTool.getValidPackageName(rosProjectInfo.ModelName);    
            interfaceTable = [rosctrlSettings.InportTable;rosctrlSettings.OutportTable];
            hwInterfaceTypes = unique(interfaceTable(:,4));
            allResources = [interfaces.InputResources; interfaces.OutputResources];

            rosControlPrjInfo.InitFcn = getFunctionCall(cinfo.codeInfo.InitializeFunctions);
            rosControlPrjInfo.TermFcn = getFunctionCall(cinfo.codeInfo.TerminateFunctions);  
            rosControlPrjInfo.StepFcn = getFunctionCall(cinfo.codeInfo.OutputFunctions);
            rosControlPrjInfo.RootIOAccessFormat = Util.getRootIOAccessFormat(cinfo);
            rootIOAccessorMap = containers.Map();
            rootIOAccessorMap('Inport') = @getInportStructExpr;
            rootIOAccessorMap('Outport') = @getOutportStructExpr;
            % Create a unique list of all hardware interfaces
            for k = 1:numel(hwInterfaceTypes)
                rosControlPrjInfo.HardwareInterfaces(k).Type = hwInterfaceTypes{k};
                interfaceIndex = contains({allResources.Name},hwInterfaceTypes{k});
                rosControlPrjInfo.HardwareInterfaces(k).VarName = ...
                    allResources(interfaceIndex).CppVarname;
                rosControlPrjInfo.HardwareInterfaces(k).Classname = ...
                    allResources(interfaceIndex).Classname;
                rosControlPrjInfo.HardwareInterfaces(k).Header = ...
                    allResources(interfaceIndex).Header;
            end
            rosControlPrjInfo.JointNamesCSList = char(strjoin([rosctrlSettings.InportTable(:,3); rosctrlSettings.OutportTable(:,3)],', '));
            % Map each inport/outport to a given interface
            for k = 1:height(interfaceTable)
                thisRow = interfaceTable(k,:);
                rosControlPrjInfo.JointInterfaces(k).BlockPath = [rosProjectInfo.ModelName,'/',char(thisRow(2))];
                rosControlPrjInfo.JointInterfaces(k).JointName = ['"',char(thisRow(3)),'"'];
                rosControlPrjInfo.JointInterfaces(k).InterfaceType = char(thisRow(4));
                rosControlPrjInfo.JointInterfaces(k).Port = char(thisRow(1));
                blockType = get_param(rosControlPrjInfo.JointInterfaces(k).BlockPath,'BlockType');
                rosControlPrjInfo.JointInterfaces(k).PortType = blockType;
                
                idx = contains({rosControlPrjInfo.HardwareInterfaces.Type},char(thisRow(4)));
                rosControlPrjInfo.JointInterfaces(k).HwInterfaceVarName = rosControlPrjInfo.HardwareInterfaces(idx).VarName;
                rosControlPrjInfo.JointInterfaces(k).HwInterfaceClass = rosControlPrjInfo.HardwareInterfaces(idx).Classname;
                portNum = str2double(rosControlPrjInfo.JointInterfaces(k).Port);
                accessorFcn = rootIOAccessorMap(blockType);
                rosControlPrjInfo.JointInterfaces(k).FieldName = ...
                    accessorFcn(portNum);
            end

            function ret = getInportStructExpr(portNum)
                impl = cinfo.codeInfo.Inports(portNum).Implementation;
                rosControlPrjInfo.InputType = sprintf('extern %s %s;',...
                    impl.BaseRegion.Type.Identifier,...
                    impl.BaseRegion.Identifier);
                ret = [impl.BaseRegion.Identifier,'.',impl.ElementIdentifier];
            end
            function ret = getOutportStructExpr(portNum)
                impl = cinfo.codeInfo.Outports(portNum).Implementation;
                rosControlPrjInfo.OutputType = sprintf('extern %s %s;',...
                    impl.BaseRegion.Type.Identifier,...
                    impl.BaseRegion.Identifier);
                ret = [impl.BaseRegion.Identifier,'.',impl.ElementIdentifier];
            end
            
        end

        function ret = getRootIOAccessFormat(cinfo)
            if ~isempty(cinfo.codeInfo.Inports)
                ret = class(cinfo.codeInfo.Inports(1).Implementation);
            else
                ret = class(cinfo.codeInfo.Outports(1).Implementation);
            end
            hasBothRootIO = ~isempty(cinfo.codeInfo.Inports) && ~isempty(cinfo.codeInfo.Outports);
            if (hasBothRootIO)
                inputImplType = class(cinfo.codeInfo.Inports(1).Implementation);
                outputImplType = class(cinfo.codeInfo.Outports(1).Implementation);
                assert(isequal(inputImplType,outputImplType),...
                    'Root-level input and output port data interfaces do not match.')
            end
        end

        function pkgInfo = setROSControlPkgInfo(modelName,pkgInfo,srcFiles,incFiles)
            pkgInfo.Dependencies = unique([pkgInfo.Dependencies, ...
                {'controller_interface','hardware_interface',...
                'pluginlib','realtime_tools','tf'}]);
            pkgInfo.LibSourceFiles = srcFiles;
            pkgInfo.LibIncludeFiles = incFiles;
            pkgInfo.CppLibraryName = modelName;
            pkgInfo.LibFormat = 'SHARED';
        end

        function copyControllerPluginFiles(pkgRoot,pkgName,srcFolder)
            pkgFolder = fullfile(pkgRoot,'src',pkgName);
            configFolder = fullfile(pkgFolder,'config');
            copyfile(fullfile(srcFolder,'controllers.xml'),pkgFolder,'f');
            if ~isfolder(configFolder)
                mkdir(configFolder);
            end
            copyfile(fullfile(srcFolder,'controllers.yaml'),...
                fullfile(configFolder,'controllers.yaml'),'f');
        end
    end


end

% -------------------------------------------------------------------------
% Local functions
% -------------------------------------------------------------------------
function loc_createOutput(data,tmplFile,outFile)
%Load the given template and render the data in it.

    tmpl = ros.internal.emitter.MLTemplate;
    tmpl.loadFile(tmplFile);
    tmpl.outFile = outFile;
    tmpl.render(data, 2);
    % smart-indent and beautify the generated header and C++ files
    isaMexFileOnPath = 3;
    if (isaMexFileOnPath == exist('c_beautifier','file')) && ...
            (endsWith(outFile,'.cpp') || endsWith(outFile,'.h'))
        c_beautifier(outFile);
    end
end

function context = l_addLinkOnlyObjects(linkOnlyObjs,buildInfo,context,isRemoteBuild)
cudaSysLibNames = {'cudnn','nvinfer_plugin','nvinfer','cudart','cublas','cufft','cusolver'};
for k = 1:numel(linkOnlyObjs)
    % Ignore rtwshared[.lib,.a] as a custom link object. Compile the
    % shared utility sources as part of the ROS build project.
    isRtwSharedLib = ismember(linkOnlyObjs(k).Name,{'rtwshared','rtwshared.lib','rtwshared.a'});

    % Ignore cuda system libs for remote build. The libraries installed on
    % remote ROS device will be used for linking
    [~,libName,~] = fileparts(linkOnlyObjs(k).Name);
    isCudaSysLib = ismember(libName,cudaSysLibNames);
    if isRtwSharedLib || (isRemoteBuild && isCudaSysLib)
        continue;
    else
        context.ImportedLibs{1,k} = buildInfo.formatPaths(fullfile(linkOnlyObjs(k).Path,linkOnlyObjs(k).Name));
    end
end
end

function context = l_addPreCompiledObjects(preCompObjs,buildInfo,context)
for k = 1:numel(preCompObjs)
    % Ignore rtwshared[.lib,.a] as a custom link object. Compile the
    % shared utility sources as part of the ROS build project.
    if ismember(linkOnlyObjs(k).Name,{'rtwshared','rtwshared.lib','rtwshared.a'})
        continue;
    end
    pathToLib = buildInfo.formatPaths(preCompObjs(k).Path,'replaceStartDirWithRelativePath', true);
    if endsWith(pathToLib,{'.o','.obj','.O','.OBJ'})
        % Include full path for object files
        % Remove special character for space in path
        pathToLib = replace(pathToLib,"\ "," ");
        context.Libraries{end+1} = ['"', replace(pathToLib,'\','/'),'"'];
    else
        context.Libraries{end+1} = preCompObjs(k).Name;
        context.LibraryDirectories{end+1} = fileparts(pathToLib);
    end
end

end


function context = l_addLinkerFlags(buildInfo, context)
    allLinkFlags = strtrim(buildInfo.getLinkFlags);
    % remove linkFlags that have pkg-config
    linkFlags = allLinkFlags(~contains(allLinkFlags,'pkg-config'));
    context.LinkerFlags = {};
    for k = 1:numel(linkFlags)
        tmp = regexp(linkFlags{k}, '-l(\S+)', 'tokens');
        if ~isempty(tmp)
            for j = 1:numel(tmp)
                context.LinkerFlags  = [context.LinkerFlags, tmp{j}{1}];
            end
        end
    end
end

function context = l_addOptionsFromSysLibInfo(buildInfo,context,isRemoteBuild)
% ADDOPTIONSFROMSYSLIBINFO Adds System library files to package info
%
% SysLib property of buildInfo, contains the system
% libraries shipping with MATLAB root which are supported for
% Linux only. Add these libraries and paths to CMakeLists.txt
%
% EXAMPLE:
%    import ros.codertarget.internal.Util
%    pkgInfo = Util.addOptionsFromSysLibInfo(buildInfo,pkgInfo);

if isRemoteBuild || ~isunix
    return;
end
[libs,libPaths]=getSysLibInfo(buildInfo);
if ~isempty(libPaths)
    libPathsOpts = buildInfo.formatPaths(libPaths);
    context.LibraryDirectories = [context.LibraryDirectories libPathsOpts];
end

if ~isempty(libs)
    context.LinkerFlags = [context.LinkerFlags libs];
end
end

