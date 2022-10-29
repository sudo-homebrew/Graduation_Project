function onHardwareSelect(hCS)
%This function is for internal use only. It may be removed in the future.

%ONHARDWARESELECT Executed when ROS 2 hardware is selected

%   See also ros.slros2.internal.cgen.onHardwareSelect(hCS)

%  Copyright 2019-2021 The MathWorks, Inc.

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
    hwDeviceTypeMap = containers.Map({'win64','glnxa64','maci64'}, ...
        {'Intel->x86-64 (Windows64)', ...
        'Intel->x86-64 (Linux 64)', ...
        'Intel->x86-64 (MAC OS X)'});
    slConfigUISetEnabled(getDialogHandle(hCS), hCS, 'ProdHWDeviceType', true);
    set_param(hCS, 'ProdHWDeviceType', hwDeviceTypeMap(computer('arch')));
    slConfigUISetEnabled(getDialogHandle(hCS), hCS, 'ProdHWDeviceType', false);
    setProp(hCS, 'GRTInterface', 'off');
    ros.codertarget.internal.onHardwareSelect(hCS);
    setProp(hCS, 'CodeInterfacePackaging', 'C++ class');
    % keep model types global
    setProp(hCS, 'IncludeModelTypesInModelClass','off');
end
end
