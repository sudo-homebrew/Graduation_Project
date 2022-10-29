function rosProjectInfo = getCppClassDefinition(cfg, buildFolder,rosProjectInfo)
%This function is for internal use only. It may be removed in the future.

% GETCPPCLASSDEFINITION Get C++ class definition and constructor arguments
% for use with ROS (2) node generation in C++ node interface templates

% Copyright 2021 The MathWorks, Inc.


validateattributes(cfg,{'Simulink.ConfigSet'},{'nonempty'});
validateattributes(buildFolder,{'char'},{'nonempty'});
validateattributes(rosProjectInfo,{'struct'},{'nonempty'});

if strcmp(get_param(cfg, 'CodeInterfacePackaging'), 'C++ class')
    rtwCPPClassObj = get_param(get_param(getModel(cfg),'Name'),'RTWCPPFcnClass');
    rosProjectInfo.StepMethodName = rtwCPPClassObj.getStepMethodName;
    rosProjectInfo.ModelClassName = rtwCPPClassObj.ModelClassName;
    [rosProjectInfo.ConstructorArgs, rosProjectInfo.ConstructorArgDefns] = ...
        getConstructorArgsFromCodeInfo(buildFolder);
else
    rosProjectInfo.StepMethodName = [];
    rosProjectInfo.ModelClassName = [];
    rosProjectInfo.ConstructorArgs = [];
    rosProjectInfo.ConstructorArgDefns = [];
end

end

function [args,argDefns] = getConstructorArgsFromCodeInfo(buildFolder)
argDefns = [];
args = [];

if isfile(fullfile(buildFolder,"codeInfo.mat"))
    cinfo = load(fullfile(buildFolder,"codeInfo.mat"));
    constructorFcn = cinfo.codeInfo.ConstructorFunction;
    if ~isempty(constructorFcn.ActualArgs)
        for iter=constructorFcn.ActualArgs
            switch class(iter.Implementation.Type)
                case 'coder.types.Pointer'
                    argDefns = sprintf('static %s %s;\n%s',...
                        iter.Implementation.TargetVariable.Type.Identifier, ...
                        iter.Implementation.TargetVariable.Identifier, ...
                        argDefns);
                    args = sprintf('&%s, %s', ...
                        iter.Implementation.TargetVariable.Identifier,...
                        args);
                otherwise
                    assert(false,"Unsupported constructor argument type");
            end
        end
        if ~isempty(args)
            % remove trailing comma
            args = regexprep(strtrim(args),',$','');
        end
    end
end


end
