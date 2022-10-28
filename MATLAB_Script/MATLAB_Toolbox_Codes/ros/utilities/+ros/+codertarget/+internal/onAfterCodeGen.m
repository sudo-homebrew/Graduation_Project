function onAfterCodeGen(hCS, buildInfo)
%This function is for internal use only. It may be removed in the future.

%ONAFTERCODEGEN Hook point for after code generation

%   Copyright 2014-2021 The MathWorks, Inc.

    if ros.codertarget.internal.isMATLABConfig(hCS)
        [~,modelName] = findBuildArg(buildInfo,'MLC_TARGET_NAME');
        % Since there is no model reference in MATLAB functions, the
        % node generation should always be top level.
        isTopLevelModel = true;
        remoteBuild = ~isequal(hCS.Hardware.DeployTo,'Localhost');
        genCodeOnly = hCS.GenCodeOnly;
        % For stand-alone executable build, entry point function cannot take
        % input arguments or return outputs
        numInputs = nargin(modelName);
        numOutputs = nargout(modelName);
        buildInfo.manageTargetInfo('settargetwordsizesforml',hCS);
        if strcmpi(hCS.OutputType,'exe') && ((numInputs > 0) || (numOutputs > 0))
            error(message('ros:utilities:util:InvalidFunctionPrototype',...
                          modelName,numInputs,numOutputs));
        end
    else
        data = codertarget.data.getData(hCS);
        modelName = buildInfo.ModelName;
        isTopLevelModel = ros.codertarget.internal.Util.isTopLevelModel(buildInfo);
        if isfield(data,'ROS') && isfield(data.ROS,'RemoteBuild') && islogical(data.ROS.RemoteBuild)
            remoteBuild = data.ROS.RemoteBuild;
        else
            % for older ROS models, default is DeployTo = Remote
            remoteBuild = true;
        end
        genCodeOnly = strcmpi(get_param(hCS,'GenCodeOnly'),'on');
        % Open new stage in diagnostic viewer and close it when function exits or
        % if an error occurs
        archiveStage = sldiagviewer.createStage(message('ros:slros:deploy:BuildArchiveStage').getString, 'ModelName', modelName);
        stageCleanup = onCleanup(@() delete(archiveStage));
    end

    % Throw an error if a local application with same name as model is already
    % running
    if isTopLevelModel && ~remoteBuild && ~genCodeOnly
        loc_errorIfApplicationIsRunning(modelName);
    end

    if ros.codertarget.internal.isMATLABConfig(hCS) || ~ros.codertarget.internal.Util.isTopLevelModel(buildInfo)
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
    % Remove unused header
    idx = find(contains(buildInfo.getIncludeFiles('',''),'linuxinitialize.h'));
    buildInfo.Inc.Files(idx) = [];


    % Replace the define '-DRT' with '-DRT=RT'. This define clashes with a
    % definition in BOOST math library
    defToReplace.Name = 'RT';
    defToReplace.ReplaceWith = 'RT=RT';
    loc_replaceDefines(buildInfo, defToReplace);
    if remoteBuild
        %% Replace host-specific UDP block files with target-specific ones
        fileToFind = fullfile('$(MATLAB_ROOT)','toolbox','shared','spc','src_ml','extern','src','DAHostLib_Network.c');
        found = loc_findInBuildInfoSrc(buildInfo,fileToFind);
        if ~isempty(found)
            sourceFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
            loc_addUDPBlocksToBuildInfo(buildInfo, sourceFolder);
        end
    end

    % Save the ROS project meta-data
    ros.slros.internal.cgen.postCodeGenHook(hCS, buildInfo);
end

%--------------------------------------------------------------------------
% Internal functions
%--------------------------------------------------------------------------
function loc_errorIfApplicationIsRunning(modelName)
% LOC_ERRORIFAPPLICATIONISRUNNING Throw an error if the application with same name
% as model is running already. If the model is set to GenerateCode only, no need to
% error out since compilation will not happen and only the C++/Header files and
% makefiles will be generated.
    if ros.codertarget.internal.isLocalNodeRunning(modelName)
        if ispc
            appName = [modelName '.exe'];
        else
            appName = modelName;
        end
        throwAsCaller(MSLException([],message('ros:utilities:util:NodeAlreadyRunningError',...
                                              modelName,appName)));
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

%--------------------------------------------------------------------------
function found = loc_findInBuildInfoSrc(buildInfo,filename)
    filename = strrep(filename,'$(MATLAB_ROOT)',matlabroot);
    found = [];
    for j=1:length(buildInfo.Src.Files)
        iFile = fullfile(buildInfo.Src.Files(j).Path, buildInfo.Src.Files(j).FileName);
        iFile = strrep(iFile,'$(MATLAB_ROOT)',matlabroot);
        if contains(iFile, filename)
            found = iFile;
            break;
        end
    end
end

%--------------------------------------------------------------------------
function loc_addUDPBlocksToBuildInfo(buildInfo, sourceFolder)
    filePathToAdd = sourceFolder;
    fileNameToAdd = 'linuxUDP.c';

    addSourceFiles(buildInfo,fileNameToAdd,filePathToAdd);
    addDefines(buildInfo,'_USE_TARGET_UDP_');
end
