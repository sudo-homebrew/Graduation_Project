function onBuildEntryHook(hCS)
%This function is for internal use only. It may be removed in the future.

%ONBUILDENTRYHOOK Entry hook point for code generation

%   Copyright 2014-2022 The MathWorks, Inc.

    modelName = ros.codertarget.internal.getModelName(hCS);

% Verify that 'Hardware board' setting is compatible with the ROS / ROS 2
% blocks
    ros.slros.internal.block.errorIfBlocksConflict(modelName, 'InCodeGen', true);
    
% Check that target language is C++ and error out if it is not
    targetLang = getProp(hCS, 'TargetLang');
    if ~strcmpi(targetLang, 'C++')
        error(message('ros:slros:cgen:CppLanguageRequired', targetLang));
    end

    % Check that PackageGeneratedCodeAndArtifacts and ERTFilePackagingFormat
    % are set appropriately. These parameters are disabled in onHardwareSelect
    % hook, but this only applies to new models; a user can open a model from a
    % older release that is configured for ROS target but with these parameters
    % editable

    val = getProp(hCS, 'PackageGeneratedCodeAndArtifacts');
    if ~strcmpi(val, 'off')
        error(message('ros:slros:cgen:PackNGoNotSupported'));
    end

    pkgFormat = getProp(hCS, 'ERTFilePackagingFormat');
    if ~strcmpi(pkgFormat, 'modular')
        error(message('ros:slros:cgen:ModularPackagingRequired', pkgFormat));
    end

    if (ros.codertarget.internal.Util.isROSControlEnabled(modelName)) && ...
            ~contains(getProp(hCS,'CodeInterfacePackaging'),'Nonreusable')
        mdlHandle = get_param(modelName,'Handle');
        diag = MSLException(mdlHandle, message('ros:slros:roscontrol:NeedsNonReusableFcn',modelName));
        throw(diag);
    end

    isRemoteBuild = ros.codertarget.internal.isRemoteBuild(hCS);
    isExplicitPartition = isequal(get_param(modelName,'ExplicitPartitioning'), 'on') ...
        && isequal(get_param(modelName,'ConcurrentTasks'), 'on');
    if isExplicitPartition && (ispc && ~isRemoteBuild)
        mdlHandle = get_param(modelName,'Handle');
        diag = MSLException(mdlHandle, message('ros:slros:cgen:InvalidHostForExplicitPartitions',modelName));
        throw(diag);
    end

    % Create empty lib file, so that build infrastructure for model references
    % behaves correctly (it expects to link against <modelName>_rtwlib, but
    % that is not needed for ROS targets.
    libFile = strcat(modelName, '_rtwlib.lib');
    fid = fopen(libFile, 'w');
    if fid ~= -1    
        fclose(fid);
    end

end
