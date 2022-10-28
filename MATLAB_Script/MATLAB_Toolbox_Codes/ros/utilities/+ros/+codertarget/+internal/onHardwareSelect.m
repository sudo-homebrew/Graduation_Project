function onHardwareSelect(hCS)
%This function is for internal use only. It may be removed in the future.

%ONHARDWARESELECT Executed when ROS hardware is selected
%   See also ros.codertarget.internal.onHardwareDeselect
%
%   Copyright 2014-2020 The MathWorks, Inc.

if ros.codertarget.internal.isMATLABConfig(hCS)
    if ~strcmpi(hCS.OutputType,'exe')
        error(message('ros:mlroscpp:codegen:InvalidOutputType',hCS.OutputType));
    end
    % Set code generation language to C++
    hCS.TargetLang = 'C++';
    % We generate a special main.cpp for ROS nodes. Do not generate an
    % example main as it would not work with ROS framework.
    hCS.GenerateExampleMain = 'DoNotGenerate'; 
    % Enable variable-sizing
    hCS.EnableVariableSizing = true;
    % Set variable-size arrays to be dynamically allocated. This means that
    % variable-sized arrays are mapped to coder::array type
    hCS.DynamicMemoryAllocation = 'AllVariableSizeArrays';
else
    validateattributes(hCS, {'Simulink.ConfigSet'}, {'nonempty'});
    
    % Note: The setProp commands below will work even if the properties are
    % already disabled
    
    % Set code generation language to C++
    val = getProp(hCS, 'TargetLang');
    if ~strcmpi(val, 'C++')
        setProp(hCS, 'TargetLang', 'C++');
    end
    
    % Set solver to Fixed Step
    val = getProp(hCS, 'SolverType');
    if ~strcmpi(val, 'Fixed-step')
        setProp(hCS, 'SolverType', 'Fixed-step');
    end
    
    % Disable Pack N Go
    val = getProp(hCS, 'PackageGeneratedCodeAndArtifacts');
    if ~strcmpi(val, 'off')
        setProp(hCS, 'PackageGeneratedCodeAndArtifacts', 'off');
    end
    
    % Set ERTFilePackagingFormat to Modular (this is in Config Params > Code
    % Generation > Code Placement pane). This setting is required for the
    % generated code to have a separate <model>_types.h file (g1320866) with
    % either Simulink Coder or Embedded Coder
    val = getProp(hCS, 'ERTFilePackagingFormat');
    if ~strcmpi(val, 'modular')
        setProp(hCS, 'ERTFilePackagingFormat', 'Modular');
    end
    
    % Enable Single Output/Update function generation
    val = getProp(hCS, 'CombineOutputUpdateFcns');
    if ~strcmpi(val, 'on')
        setProp(hCS, 'CombineOutputUpdateFcns', 'on');
    end
    
    % Disable GRTInterface (Classic style) code-generation
    val = getProp(hCS, 'GRTInterface');
    if ~strcmpi(val, 'off')
        setProp(hCS, 'GRTInterface', 'off');
    end
    % Lock down properties so that they cannot be accidentally modified by the
    % user.
    %
    % NOTE: When disabling properties here, be sure to enable them
    % in onHardwareDeselect
    
    hCS.setPropEnabled('TargetLang', false);
    hCS.setPropEnabled('PackageGeneratedCodeAndArtifacts', false);
    hCS.setPropEnabled('ERTFilePackagingFormat', false);
    hCS.setPropEnabled('CombineOutputUpdateFcns', false);
    hCS.setPropEnabled('GRTInterface', false);
    
    
    % Enable Dynamic memory allocation
    data = codertarget.data.getData(hCS);
    if strcmp(data.TargetHardware, 'Raspberry Pi - Robot Operating System (ROS)')
        set_param(hCS, 'MATLABDynamicMemAlloc', true);
    end
end
end
