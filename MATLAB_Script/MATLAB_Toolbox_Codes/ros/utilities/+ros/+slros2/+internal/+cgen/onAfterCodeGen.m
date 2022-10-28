function onAfterCodeGen(hCS, buildInfo)
%This function is for internal use only. It may be removed in the future.

%ONAFTERCODEGEN Hook point for after code generation


%   Copyright 2019-2022 The MathWorks, Inc.

    if ros.codertarget.internal.isMATLABConfig(hCS)
        onAfterCodegenML(hCS,buildInfo);
    else
        onAfterCodegenSL(hCS,buildInfo);
    end
end

function onAfterCodegenSL(hCS, buildInfo)

    data = codertarget.data.getData(hCS);
    modelName = buildInfo.ModelName;

    % Get the build directory (that's where we will put the generated files)
    bDir = getSourcePaths(buildInfo, true, {'BuildDir'});
    if isempty(bDir)
        bDir = {pwd};
    end

    isRefModel = ~ros.codertarget.internal.Util.isTopLevelModel(buildInfo);

    % Throw an error if an application with same name as model is already
    % running
    genCodeOnly = strcmp(get_param(modelName,'GenCodeOnly'),'on');
    if ~isRefModel
        loc_errorIfApplicationIsRunning(modelName,genCodeOnly)
    end

    if isRefModel
        % Update buildInfo only for the referenced models
        extList = {'.c' '.C' '.cpp' '.CPP' '.s' '.S'};
        incExtList = {'.h' '.H', '.hpp'};
        updateFilePathsAndExtensions(buildInfo, extList, incExtList);
    end

    % ignoreParseError converts parsing errors from findIncludeFiles into warnings
    findIncludeFiles(buildInfo, ...
                     'extensions', {'*.h' '*.hpp'}, ...
                     'ignoreParseError', true);

    % Temporary fix for C++ code generation
    removeSourceFiles(buildInfo,...
                      {'ert_main.c', ...
                       'ert_main.cpp', ...
                       'ert_main.cu', ...
                       'rt_cppclass_main.cpp', ...
                       'linuxinitialize.cpp', ...
                       'main.c'});    

    slRefLibrary = 'ros2lib';
    modelParseFcn = @ros.slros2.internal.bus.Util.getROS2BlocksInModel;
    ros2ModelInfo = ros.slros.internal.cgen.getProjectInfo(data, ...
        modelName, isRefModel, ...
        slRefLibrary, modelParseFcn, bDir{1});
    % Get C++ class interface Settings
    ros2ModelInfo = ros.codertarget.internal.getCppClassDefinition(...
        hCS,bDir{1},ros2ModelInfo);

    ros2ModelInfo.AddRTWTypesHeader = isfile(fullfile(bDir{1},'rtwtypes.h'));
    

    % add dependency for ROS 2 Read Image block if exists
    ros2ModelInfo.ImageDepends = {''};
    scriptLoc = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
    imageBlockList = ros.slros.internal.bus.Util.listBlocks(modelName, ...
        ros.slros2.internal.block.ReadImageBlockMask.MaskType);
    if ~isempty(imageBlockList)
        ros2ModelInfo.ImageDepends = {'cv_bridge'};
        % copy the predefined image header file to current directory
        cgenConsts = ros.slros.internal.cgen.Constants;
        buildInfo.addIncludeFiles(cgenConsts.PredefinedCode.ImageHeaderFile);
        copyfile(fullfile(scriptLoc, cgenConsts.PredefinedCode.ImageHeaderFile), ...
                 fullfile(bDir{1}, cgenConsts.PredefinedCode.ImageHeaderFile),'f');
    end


    % set remote build property
    if isfield(data.ROS, 'RemoteBuild') && islogical(data.ROS.RemoteBuild)
        ros2ModelInfo.RemoteBuild = data.ROS.RemoteBuild;
    end

    ros2ModelInfo = ros.internal.ros2.augmentModelInfo(ros2ModelInfo);
    % Set <modelname>_types.h
    existingIncludeFiles = buildInfo.getIncludeFiles(true, true);
    modelTypesHdrIdx = find(contains(existingIncludeFiles, [modelName '_types.']));
    if ~isempty(modelTypesHdrIdx)
        [~, fname, ext] = fileparts( existingIncludeFiles{modelTypesHdrIdx} );
        ros2ModelInfo.ModelTypesHeader = [fname ext];
    else
        ros2ModelInfo.ModelTypesHeader = '';
    end
    ros2ModelInfo.ModelRefHeaders = loc_getModelRefHeaders(buildInfo);
    % generate node interface functions only if this is a top-level model
    if ~isRefModel
        % Add External mode information
        extModeInfo.ExtmodeSim = ros.codertarget.internal.Util.isExternalModeBuild(buildInfo);
        if extModeInfo.ExtmodeSim
            % get values from the ExtMode configset
            extModeInfo.Port = codertarget.attributes.getExtModeData('Port', hCS);
            extModeInfo.RunInBackground = codertarget.attributes.getExtModeData('RunInBackground', hCS);
            extModeInfo.Verbose = codertarget.attributes.getExtModeData('Verbose', hCS);
        else
            % set default values
            extModeInfo.Port = '17725';
            extModeInfo.RunInBackground = true;
            extModeInfo.Verbose = '0';
        end
        ros2ModelInfo.ExtmodeInfo = extModeInfo;

        % Template generation for ros2matlabnodeinterface.h/cpp
        loc_generateNodeInterface(ros2ModelInfo, buildInfo, bDir{1});
    end
    % Generate MSG-to-BUS and BUS-to-MSG conversion functions
    loc_generateConversionFcns(ros2ModelInfo, modelName, buildInfo, bDir{1});

    % Generate the extern definitions for pub/subs for each model in
    % ref-hierarchy
    loc_generatePubSubHeader(ros2ModelInfo, buildInfo, bDir);
	
	% Link against the cublas, cufft and cusolver libs when CUDA code
    % generation enabled.
    ros.internal.gpucoder.updateBuildInfo(hCS, buildInfo, ros2ModelInfo.RemoteBuild);
	ros2ModelInfo.GPUFlags = ros.internal.gpucoder.getFlags(hCS);
	
	
    %save ros2modelinfo
    save(fullfile(bDir{1}, ros.slros2.internal.cgen.Constants.ROS2ModelInfoFile), ...
         'ros2ModelInfo');

    % Replace the define '-DRT' with '-DRT=RT'. This define clashes with a
    % definition in BOOST math library
    defToReplace.Name = 'RT';
    defToReplace.ReplaceWith = 'RT=RT';
    loc_replaceDefines(buildInfo, defToReplace);

    sDir = getSourcePaths(buildInfo, true, {'StartDir'});
    if isempty(sDir)
        sDir = {pwd};
    end
    % Copy build_ros2_model.sh to the same directory as the archive file
    scriptName = 'build_ros2_model.sh';
    targetScript = fullfile(sDir{1}, scriptName);
    copyfile(fullfile(scriptLoc, scriptName), targetScript, 'f');
end

function onAfterCodegenML(hCS, buildInfo)
% Convert MATLAB target parameters to the format used for Simulink
    bDir = ros.codertarget.internal.getBuildDir(buildInfo);
    [~,modelName] = findBuildArg(buildInfo,'MLC_TARGET_NAME');
    % For stand-alone executable build, entry point function cannot take
    % input arguments or return outputs
    numInputs = nargin(modelName);
    numOutputs = nargout(modelName);
    buildInfo.manageTargetInfo('settargetwordsizesforml',hCS);
    if strcmpi(hCS.OutputType,'exe') && ((numInputs > 0) || (numOutputs > 0))
        error(message('ros:utilities:util:InvalidFunctionPrototype',...
                      modelName,numInputs,numOutputs));
    end
    ros2ModelInfo = loc_mlGetProjectInfo(hCS, modelName);
    isRemoteBuild = ~isequal(hCS.Hardware.DeployTo,'Localhost');
    ros2ModelInfo.RemoteBuild = isRemoteBuild;
    genCodeOnly = hCS.GenCodeOnly;

    if ~isRemoteBuild && ~genCodeOnly
        loc_errorIfApplicationIsRunning(modelName, genCodeOnly);
    end

    cgenInfo = ros.codertarget.internal.ROSMATLABCgenInfo.getInstance;
    if numel(getNodes(cgenInfo)) > 1
        assert(false, message('ros:mlros2:codegen:MultipleNodesNotAllowed',modelName));
    end
    if ~isempty(getMessageTypesWithInt64(cgenInfo)) && ~hCS.HardwareImplementation.ProdLongLongMode
        warning(message('ros:mlroscpp:codegen:SetProdLongLongMode',modelName));
    end

    if ~isempty(getNodes(cgenInfo))
        [baseName, namespace] = getNodeNameParts(cgenInfo, cgenInfo.NodeList{1});
        ros2ModelInfo.Namespace = namespace;
        ros2ModelInfo.NodeName = baseName;
    else
        ros2ModelInfo.Namespace = '';
        ros2ModelInfo.NodeName = modelName;
    end

    setNodeName(cgenInfo,ros2ModelInfo.NodeName);
    nodeinfo = getNodeDependencies(cgenInfo);

    uniqueMsgList = unique(nodeinfo.messageList);
    % Write message information to the project info for use with
    % ProjectTool and local deployment
    ros2ModelInfo.MessageInfoArray = cell(1,numel(uniqueMsgList));
    for k=1:numel(uniqueMsgList)
        [~,~,msgInfo] = ros.internal.getEmptyMessage(uniqueMsgList{k},'ros2');
        ros2ModelInfo.MessageInfoArray{k} = msgInfo;
    end

    % Generate main.cpp
    mainFile = fullfile(bDir,'main.cpp');
    mainTempl = fullfile(toolboxdir('ros'),'codertarget','templates','ros2_ml_main.cpp.tmpl');
    % generate node interface header
    loc_createOutput(ros2ModelInfo, mainTempl, mainFile);
    addSourceFiles(buildInfo,mainFile);

    % Generate message conversion functions, struct type definitions, etc.
    serviceTypes = {};
    if ~isempty(cgenInfo.getMessageTypes)
        typesHeader = [modelName,'_types.h'];
        conversionFiles = ros.slros.internal.cgen.generateAllConversionFcns(...
            cgenInfo.getMessageTypes, serviceTypes, hCS, typesHeader, bDir, ...
            'BusUtilityObject',ros.slros2.internal.bus.Util, ...
            'CodeGenUtilityObject',ros.slros2.internal.cgen.Util,...
            'ROSHeader','"rclcpp/rclcpp.hpp"');
        addIncludeFiles(buildInfo, fullfile(bDir,conversionFiles.HeaderFiles));
        addSourceFiles(buildInfo,  fullfile(bDir,conversionFiles.SourceFiles));
        msgConvertData.HasCoderArray = contains(fileread(fullfile(bDir,typesHeader)),'coder_array.h');
        msgConvertData.NodeName = modelName;
        msgConvertData.ROSVer = 'ros2';
        tmpl = ros.internal.emitter.MLTemplate;
        tmpl.loadFile(fullfile(toolboxdir('ros'),'codertarget','templates','mlroscpp_msgconvert_utils.h.tmpl'));
        tmpl.outFile = fullfile(bDir, 'mlros2_msgconvert_utils.h');
        tmpl.render(msgConvertData,2);
        addIncludeFiles(buildInfo, tmpl.outFile);
    end
    ros2ModelInfo.AddRTWTypesHeader = isfile(fullfile(bDir,'rtwtypes.h'));
    % Set image block dependency field to empty here to avoid conflict
    ros2ModelInfo.ImageDepends = {''};
    %save ros2modelinfo
    save(fullfile(bDir, 'ros2ModelInfo.mat'), 'ros2ModelInfo');

    % Replace the define '-DRT' with '-DRT=RT'. This define clashes with a
    % definition in BOOST math library
    defToReplace.Name = 'RT';
    defToReplace.ReplaceWith = 'RT=RT';
    loc_replaceDefines(buildInfo, defToReplace);
    sDir = getSourcePaths(buildInfo, true, {'StartDir'});
    if isempty(sDir)
        sDir = {pwd};
    end
    % Copy build_model.sh to the same directory as the archive file
    scriptName = 'build_ros2_model.sh';
    targetScript = fullfile(sDir{1}, scriptName);
    scriptLoc = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
    copyfile(fullfile(scriptLoc, scriptName), targetScript, 'f');

end



%--------------------------------------------------------------------------
% Internal functions
%--------------------------------------------------------------------------

function loc_generatePubSubHeader(ros2ModelInfo, buildInfo, bDir)
    ros2NodeConsts = ros.slros2.internal.cgen.Constants.NodeInterface;
    commonHeaderTempl = ros2NodeConsts.PubSubCommonTemplate;
    headerName = [ros2ModelInfo.ModelName, '_',ros2NodeConsts.PubSubCommonHeader];
    % generate pub-sub common header
    loc_createOutput(ros2ModelInfo, commonHeaderTempl, ...
                     fullfile(bDir{1}, headerName));
    buildInfo.addIncludeFiles(headerName );
end

%--------------------------------------------------------------------------
function loc_generateNodeInterface(ros2ModelInfo, buildInfo, bDir)
    ros2NodeConsts = ros.slros2.internal.cgen.Constants.NodeInterface;
    if isequal(get_param(ros2ModelInfo.ModelName,'CodeInterfacePackaging'),'C++ class')
        % when C++ class is selected for packaging - template is different
        headerTempl = ros2NodeConsts.HeaderTemplate;
        sourceTempl = ros2NodeConsts.SourceFileTemplate;
    else
        headerTempl = ros2NodeConsts.NonresuableFcnHeaderTemplate;
        sourceTempl = ros2NodeConsts.NonresuableFcnSourceFileTemplate;
    end
    % generate node interface header
    loc_createOutput(ros2ModelInfo, headerTempl, ...
                     fullfile(bDir, ros2NodeConsts.HeaderFile));
    % generate node interface CPP
    loc_createOutput(ros2ModelInfo, sourceTempl, ...
                     fullfile(bDir, ros2NodeConsts.SourceFile));
    % generate ROS2 Main CPP
    loc_createOutput(ros2ModelInfo, ros2NodeConsts.MainTemplate, ...
                     fullfile(bDir, ros2NodeConsts.MainFile));
    % Add node related build artifacts to buildInfo. Add only the
    % names of the header and CPP file as the buildInfo already has the
    % SourcePaths/IncludePaths for code-generation folder and will look for
    % those headers and includes
    buildInfo.addIncludeFiles(fullfile(bDir,ros2NodeConsts.HeaderFile));
    buildInfo.addSourceFiles(ros2NodeConsts.SourceFile);
    buildInfo.addSourceFiles(ros2NodeConsts.MainFile);
end

%--------------------------------------------------------------------------
function loc_generateConversionFcns(ros2ModelInfo, modelName, buildInfo, bDir)

% Find a header file that looks like <modelname>_types.h. This contains
% definitions of the bus structs, and needs to be included by the bus
% conversion header
    modelTypesHeader = [{ros2ModelInfo.ModelTypesHeader}, ros2ModelInfo.ModelRefHeaders];
    predefinedCode = ros.slros2.internal.cgen.Constants.ROS2PredefinedCode;
    % Generate all the conversion functions
    conversionFiles = ros.slros.internal.cgen.generateAllConversionFcns(...
        ros2ModelInfo.MessageTypes, {}, modelName, modelTypesHeader, bDir, ...
        'BusUtilityObject', ros.slros2.internal.bus.Util, ...
        'CodeGenUtilityObject', ros.slros2.internal.cgen.Util, ...
        'ROSHeader', '"rclcpp/rclcpp.hpp"');
    addIncludeFiles(buildInfo,fullfile(bDir,conversionFiles.HeaderFiles));
    addSourceFiles(buildInfo,  fullfile(bDir,conversionFiles.SourceFiles));
    % Add slros_msgconvert_utils.h
    buildInfo.addIncludeFiles(fullfile(predefinedCode.Location,predefinedCode.ConversionUtilsHeaderFile));
end

function loc_errorIfApplicationIsRunning(exeName, isGenCodeOnly)
% LOC_ERRORIFAPPLICATIONISRUNNING Throw an error if the application with same name
% as model is running already. If the model is set to GenerateCode only, no need to
% error out since compilation will not happen and only the C++/Header files and
% makefiles will be generated.

    if ~isGenCodeOnly
        % Create a map of name of applications per platform
        appNameMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                    {[exeName, '.exe'], exeName, exeName});
        % Create a map of system commands that will query for running application
        isAppRunningCmdMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                            {sprintf('wmic process where "name=''%s''" get ProcessID,ExecutablePath', appNameMap('win64')), ... use wmic query
                                             sprintf('ps ax | grep "%s" | grep -v "grep"', appNameMap('maci64')), ...  use pidof
                                             sprintf('ps ax | grep "%s" | grep -v "grep"', appNameMap('glnxa64')) ...  use pidof
                                            });
        % Get the correct command
        cmd = isAppRunningCmdMap(computer('arch'));
        [status, result] = system(cmd);
        isRunning = false;
        % if status is non-zero, assume application was not running
        if isequal(status, 0)
            isRunning = contains(strtrim(result), appNameMap(computer('arch')));
        end
        if isRunning
            disp(result); % diagnostic
            throwAsCaller(MSLException([], message('ros:slros2:codegen:NodeAlreadyRunningError', exeName, appNameMap(computer('arch')))));
        end
    end
end

%--------------------------------------------------------------------------
function loc_replaceDefines(buildInfo, defToRemove)
    def = buildInfo.getDefines;
    for j = 1:numel(defToRemove)
        for k = 1:numel(def)
            if isequal(def{k}, ['-D', defToRemove(j).Name])
                buildInfo.deleteDefines(defToRemove(j).Name);
                buildInfo.addDefines(defToRemove(j).ReplaceWith);
                break;
            end
        end
    end
end

% -------------------------------------------------------------------------
function ret = loc_getModelRefHeaders(buildInfo)
if ~isempty(buildInfo.ModelRefs)
    modelRefPaths = arrayfun(@(ref)formatPaths(ref, ref.Path), buildInfo.ModelRefs, 'UniformOutput', false);
    infoFileName = ros.slros2.internal.cgen.Constants.ROS2ModelInfoFile;
    ret = cell(1,numel(modelRefPaths));
    for k = 1:numel(modelRefPaths)
        if isfile(fullfile(modelRefPaths{k},infoFileName))
            info = load(fullfile(modelRefPaths{k},infoFileName));
            ret{k} = info.ros2ModelInfo.ModelTypesHeader;
        end
    end
else
    ret = {};
end
end
%--------------------------------------------------------------------------
function loc_createOutput(data,tmplFile,outFile)
%Load the given template and render the data in it.

    tmpl = ros.internal.emitter.MLTemplate;
    tmpl.loadFile(tmplFile);
    tmpl.outFile = outFile;
    tmpl.render(data, 2);
end

%--------------------------------------------------------------------------
function rosProjectInfo = loc_mlGetProjectInfo(hCS, modelName)
% MATLAB ROS 2 project generation meta data

% Store Package information
    rosProjectInfo = struct;
    rosProjectInfo.ModelName = modelName;
    % Generate a structure that ROS project builder expects to see. The
    % structure contains package information
    rosProjectInfo.PackageInformation = struct(...
        'MaintainerName',hCS.Hardware.PackageMaintainerName,...
        'MaintainerEmail',hCS.Hardware.PackageMaintainerEmail,...
        'License',hCS.Hardware.PackageLicense,...
        'Version',hCS.Hardware.PackageVersion);
    rosProjectInfo.ROS = struct(...
        'ROS2Folder',hCS.Hardware.ROS2Folder,...
        'ROS2Workspace',hCS.Hardware.ROS2Workspace,...
        'RemoteBuild',~isequal(hCS.Hardware.DeployTo,'Localhost'));

    % Set build arguments needed for remote build
    rosProjectInfo.RemoteBuild = rosProjectInfo.ROS.RemoteBuild;
    rosProjectInfo.BuildArguments = [];
    rosProjectInfo.StepMethodName = [];
    rosProjectInfo.ModelClassName = [];
end
