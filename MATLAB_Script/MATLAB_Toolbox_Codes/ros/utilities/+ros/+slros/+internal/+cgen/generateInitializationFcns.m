function genfiles = generateInitializationFcns(modelinfo, destDir, isTopLevelModel)
%This function is for internal use only. It may be removed in the future.

%generateInitializationFcns - Generate & save C++ code for initializing a ROS node
%
%   GEN = generateInitializationFcns(MODELINFO, DESTDIR) generates and saves
%   C++ code for creating a ROS node, and for creating C++ objects
%   corresponding to ROS blocks in the model.
%
%    Inputs
%      MODELINFO : Information about the Simulink model (cgen.ModelInfo object)
%      DESTDIR   : Directory to save the generated C++ files to.
%      ISTOPLEVELMODEL : TRUE if this a top-level model, FALSE if this is a model reference
%
%    Outputs
%      GEN    : A struct with fields 'HeaderFiles' and 'SourceFiles', which
%               are cell arrays with full pathnames of the generated files.
%

%   Copyright 2014-2022 The MathWorks, Inc.

    validateattributes(modelinfo, ...
                       {'ros.slros.internal.cgen.ROSModelInfo'}, {'scalar'});
    validateattributes(destDir, {'char'}, {});

    % Define info about generated code
    cgenConsts = ros.slros.internal.cgen.Constants;
    initinfo.Includes = {...
        cgenConsts.ConversionCode.HeaderFile, ...
        cgenConsts.PredefinedCode.HeaderFile, ...
                        };

    % Include image header if ROS Read Image blocks exist in model
    if ~isempty(modelinfo.ImageBlocks)
        initinfo.Includes{end+1} = cgenConsts.PredefinedCode.ImageHeaderFile;
    end

    % Include time header if ROS Current Time blocks exist in model
    if ~isempty(modelinfo.TimeBlocks)
        initinfo.Includes{end+1} = cgenConsts.PredefinedCode.TimeHeaderFile;
    end

    % Include service call header if ROS Call Service blocks exist in model
    if ~isempty(modelinfo.ServiceCallBlocks)
        initinfo.Includes{end+1} = cgenConsts.PredefinedCode.ServiceHeaderFile;
    end

    initinfo.HeaderFileName = cgenConsts.InitCode.HeaderFile;
    initinfo.SourceFileName = cgenConsts.InitCode.SourceFile;
    initinfo.NodePtrVarName = cgenConsts.NodePtrVar;
    initinfo.NodeNameVarName = cgenConsts.NodeNameVar;
    initinfo.InitFcnName    = cgenConsts.NodeInitFcn;
    initinfo.predefinedHeader = fullfile(cgenConsts.PredefinedCode.Location, ...
                                         cgenConsts.PredefinedCode.HeaderFile);
    initinfo.IsTopLevelModel = isTopLevelModel;

    % Generate the init files
    [headerFile, sourceFile] = generateInitFiles(modelinfo, initinfo);
    headerFileFullName = fullfile(destDir, initinfo.HeaderFileName);
    sourceFileFullName = fullfile(destDir, initinfo.SourceFileName);
    headerFile.write( headerFileFullName );
    sourceFile.write( sourceFileFullName );

    % Package common header files with MathWorks code
    genfiles.HeaderFiles = { initinfo.HeaderFileName ...
                        initinfo.predefinedHeader, ...
                        fullfile(cgenConsts.PredefinedCode.Location, cgenConsts.PredefinedCode.PubSubHeaderFile), ...
                        fullfile(cgenConsts.PredefinedCode.Location, cgenConsts.PredefinedCode.ParamHeaderFile), ...
                        fullfile(cgenConsts.PredefinedCode.Location, cgenConsts.PredefinedCode.ConversionUtilsHeaderFile), ...
                   };
    % Package MathWorks image header if needed
    if ~isempty(modelinfo.ImageBlocks)
        genfiles.HeaderFiles{end+1} = fullfile( ...
            cgenConsts.PredefinedCode.Location, ...
            cgenConsts.PredefinedCode.ImageHeaderFile);
    end

    % Package time header if needed
    if ~isempty(modelinfo.TimeBlocks)
        genfiles.HeaderFiles{end+1} = fullfile( ...
            cgenConsts.PredefinedCode.Location, ...
            cgenConsts.PredefinedCode.TimeHeaderFile);
    end

    % Package service header if needed
    if ~isempty(modelinfo.ServiceCallBlocks)
        genfiles.HeaderFiles{end+1} = fullfile( ...
            cgenConsts.PredefinedCode.Location, ...
            cgenConsts.PredefinedCode.ServiceHeaderFile);
    end

    % Package all source files with MathWorks code
    genfiles.SourceFiles = { initinfo.SourceFileName, ...
                        fullfile(cgenConsts.PredefinedCode.Location, cgenConsts.PredefinedCode.ParamSourceFile), ...
                   };

end

%%
function [headerFile, sourceFile] = generateInitFiles(modelinfo, cgeninfo)
% validate node name
%   see http://wiki.ros.org/Names
%   "First character is an alpha character ([a-z|A-Z]), tilde (~) or forward slash (/)
%   Subsequent characters can be alphanumeric ([0-9|a-z|A-Z]), underscores(_), or forward slashes (/)"
    assert(~isempty(regexp(modelinfo.NodeName,'^[a-zA-Z~/][0-9a-zA-Z_/]*$', 'once')));

    varInfo = defineVariables(modelinfo, cgeninfo);

    if cgeninfo.IsTopLevelModel && ~ros.codertarget.internal.Util.isROSControlEnabled(modelinfo.NodeName)
        % Only generate node initialization function for top-level model
        fcnInfo = defineFunctions(cgeninfo);
    else
        fcnInfo = struct.empty;
    end

    modelTypes = [modelinfo.NodeName '_types.h'];
    % Set up header file
    headerIncludes = StringWriter;
    for i=1:numel(cgeninfo.Includes)
        headerIncludes.addcr(['#include "' cgeninfo.Includes{i} '"']);
    end
    headerIncludes.addcr(['#include "' modelTypes '"']);
    headerFile = defineHeaderFile(headerIncludes, varInfo, fcnInfo);
    ros.slros.internal.cgen.insertHeaderGuards(headerFile, cgeninfo.HeaderFileName);

    % Set up source file
    srcIncludes = StringWriter;
    srcIncludes.addcr(['#include "' cgeninfo.HeaderFileName '"']);
    sourceFile = defineSourceFile(srcIncludes, varInfo, fcnInfo);
end


%%
function headerFile = defineHeaderFile(headerIncludes, varInfo, fcnInfo)
    headerFile = StringWriter;
    headerFile.addcr(headerIncludes);
    for i=1:numel(varInfo)
        if ~isempty(varInfo(i).Comment)
            commentStr = ros.slros.internal.cgen.Util.cleanupUnicodeAndWhitespace(varInfo(i).Comment);
            headerFile.craddcr(['// ' commentStr]);
        end
        headerFile.addcr(['extern ' varInfo(i).Declaration ';']);
    end
    headerFile.addcr;
    for i=1:numel(fcnInfo)
        headerFile.add(fcnInfo.FcnSignature);
        headerFile.addcr(';');
    end
end


%%
function srcFile = defineSourceFile(srcIncludes, varInfo, fcnInfo)
    srcFile = StringWriter;
    srcFile.addcr(srcIncludes);
    for i=1:numel(varInfo)
        if ~isempty(varInfo(i).Comment)
            commentStr = ros.slros.internal.cgen.Util.cleanupUnicodeAndWhitespace(varInfo(i).Comment);
            srcFile.craddcr(['// ' commentStr]);
        end
        if isempty(varInfo(i).Value)
            srcFile.addcr([varInfo(i).Declaration ';']);
        else
            srcFile.addcr([varInfo(i).Declaration ' = ' varInfo(i).Value ';']);
        end
    end
    srcFile.addcr;
    for i=1:numel(fcnInfo)
        srcFile.addcr(fcnInfo.FcnSignature);
        srcFile.addcr(fcnInfo.FcnBody);
    end
end


%%
function varInfo = defineVariables(modelinfo, cgeninfo)

    if cgeninfo.IsTopLevelModel
        % Only create the node variables in the top-level model ...
        varInfo = struct(...
            'Comment', '', ...
            'Declaration', ['ros::NodeHandle * ' cgeninfo.NodePtrVarName], ...
            'Value',       '');

        varInfo(end+1) = struct(...
            'Comment', '', ...
            'Declaration', ['const std::string ' cgeninfo.NodeNameVarName], ...
            'Value',       ['"' modelinfo.NodeName '"']);
        
        if (codertarget.data.getParameterValue(modelinfo.Model, 'DetectTaskOverruns'))
            varInfo(end+1) = struct(...
                'Comment', '', ...
                'Declaration', 'void reportOverrun(int rateID)', ...
                'Value',       '');
        end
        
    else
        % ... not for model references.
        % Initialize an empty struct array with the right field structure.
        varInfo = struct(...
            'Comment', {}, ...
            'Declaration', {},...
            'Value', {});
    end

    for i=1:numel(modelinfo.SubscriberList)
        sub = modelinfo.SubscriberList(i);
        varInfo(end+1)= struct(...
            'Comment', sub.Comment, ...
            'Declaration', sprintf('SimulinkSubscriber<%s, %s> %s', ...
                                   sub.CppRosType, sub.SlBusName, sub.Label),...
            'Value', ''); %#ok<*AGROW>
    end

    publisherOpts = {'SimulinkPublisher','SimulinkRTPublisher'};
    isRosCtrlEn = ros.codertarget.internal.Util.isROSControlEnabled(modelinfo.Model);
    pubOptsIdx = [~isRosCtrlEn,isRosCtrlEn];

    for i=1:numel(modelinfo.PublisherList)
        pub = modelinfo.PublisherList(i);
        varInfo(end+1)= struct(...
            'Comment', pub.Comment, ...
            'Declaration', sprintf('%s<%s, %s> %s', ...
                publisherOpts{pubOptsIdx},pub.CppRosType,...
                pub.SlBusName, pub.Label),...
            'Value', '');
    end

    for i=1:numel(modelinfo.ParameterGetterList)
        pget = modelinfo.ParameterGetterList(i);

        % Choose a specific C++ class for getting scalar or array parameters
        if pget.IsArray
            classString = 'SimulinkParameterArrayGetter';
        else
            classString = 'SimulinkParameterGetter';
        end

        varInfo(end+1)= struct(...
            'Comment', pget.Comment, ...
            'Declaration', sprintf('%s<%s, %s> %s', ...
                                   classString, pget.CppParamType, pget.ROSCppParamType, pget.Label),...
            'Value', '');
    end

    for i=1:numel(modelinfo.ParameterSetterList)
        pset = modelinfo.ParameterSetterList(i);
        varInfo(end+1)= struct(...
            'Comment', pset.Comment, ...
            'Declaration', sprintf('SimulinkParameterSetter<%s, %s> %s', ...
                                   pset.CppParamType, pset.ROSCppParamType, pset.Label),...
            'Value', '');
    end

    for i=1:numel(modelinfo.ServiceCallerList)
        srvCall = modelinfo.ServiceCallerList(i);
        varInfo(end+1)= struct(...
            'Comment', srvCall.Comment, ...
            'Declaration', sprintf('SimulinkServiceCaller<%s, %s, %s> %s', ...
                                   srvCall.CppRosType, srvCall.SlInputBusName, ...
                                   srvCall.SlOutputBusName, srvCall.Label), ...
            'Value', '');

    end

end


%%
function fcnInfo = defineFunctions(cgeninfo)
    fcnSignature = StringWriter;
    fcnBody = StringWriter;

    fcnSignature.add('void %s(int argc, char** argv)', cgeninfo.InitFcnName);
    fcnBody.addcr('{');
    fcnBody.Indent = 2;
    fcnBody.addcr('ros::init(argc, argv, %s);', cgeninfo.NodeNameVarName);
    fcnBody.addcr('%s = new ros::NodeHandle();', cgeninfo.NodePtrVarName);
    fcnBody.Indent = 0;
    fcnBody.addcr('}');

    fcnInfo.FcnSignature = fcnSignature;
    fcnInfo.FcnBody = fcnBody;
end
