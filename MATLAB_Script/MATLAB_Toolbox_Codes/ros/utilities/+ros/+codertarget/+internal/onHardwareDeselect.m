function onHardwareDeselect(hCS)
%This function is for internal use only. It may be removed in the future.

%ONHARDWAREDESELECT Executed when ROS hardware is de-selected
%   See also ros.codertarget.internal.onHardwareSelect

%  Copyright 2015-2020 The MathWorks, Inc.

    validateattributes(hCS, {'Simulink.ConfigSet'}, {'nonempty'});

    % Deselect/enable the properties that were disabled during target selection

    hCS.setPropEnabled('TargetLang', true);
    setProp(hCS, 'TargetLang', 'C');
    hCS.setPropEnabled('PackageGeneratedCodeAndArtifacts', true);
    hCS.setPropEnabled('ERTFilePackagingFormat', true);
    hCS.setPropEnabled('CombineOutputUpdateFcns', true);
    hCS.setPropEnabled('GRTInterface', true);
    
end
