function onCodegenEntryHook(hCS)
%This function is for internal use only. It may be removed in the future.

%ONCODEGENENTRYHOOK Entry hook point for code generation

%   Copyright 2020 The MathWorks, Inc.

if ros.codertarget.internal.isMATLABConfig(hCS)
    % Check output type
    if ~strcmpi(hCS.OutputType,'exe')
        error(message('ros:mlroscpp:codegen:InvalidOutputType',hCS.OutputType));
    end
    % Check that target language is C++ and error out if it is not
    targetLang = hCS.TargetLang;
    if ~strcmpi(targetLang,'C++')
        error(message('ros:mlroscpp:codegen:CppLanguageRequired',targetLang));
    end
    generateExampleMain =  hCS.GenerateExampleMain;
    if strcmpi(generateExampleMain,'GenerateCodeAndCompile')
        error(message('ros:mlroscpp:codegen:InvalidGenerateExampleMain',generateExampleMain));
    end
    enableVariableSizing = hCS.EnableVariableSizing;
    if enableVariableSizing ~= true
        error(message('ros:mlroscpp:codegen:VariableSizingRequired','false'));
    end
    dynamicMemoryAllocation = hCS.DynamicMemoryAllocation;
    if ~strcmpi(dynamicMemoryAllocation,'AllVariableSizeArrays')
        error(message('ros:mlroscpp:codegen:DynamicMemAllocRequired',dynamicMemoryAllocation));
    end
    
    % Reset codegen info. Each publisher / subscriber registers itself to
    % codegen info during MATLAB code generation. Info object is reset here
    % before codegen starts.
    cgenInfo = ros.codertarget.internal.ROSMATLABCgenInfo.getInstance;
    reset(cgenInfo);
    
    % Supress 'Coder:FE:ExeTargetWithoutMain' warning
    hCS.CustomSource = ' ';
end
end
