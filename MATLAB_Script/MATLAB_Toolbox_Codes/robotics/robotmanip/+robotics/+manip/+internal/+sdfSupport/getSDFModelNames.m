function modelNames = getSDFModelNames(sdfInput)
%This function is for internal use only. It may be removed in the future.

%GETSDFMODELNAMES reads .sdf file and return single or multiple model names
%from input .sdf file.

%   Copyright 2021 The MathWorks, Inc.

% validate SDF input
    in = robotics.internal.validation.validateString(sdfInput, false, 'importrobot', 'sdfInput');

    if contains(in, '</') || contains(in, '/>')
        source = 'string';
    else
        source = 'file';
    end

    % parse xml as per source type
    xmlDoc = matlabshared.multibody.internal.urdf.utils.parseXML(sdfInput, source);

    % get all robot elements
    robotEls = matlabshared.multibody.internal.urdf.Model.getAllSDFRobotElements(xmlDoc);
    % get valid SDF model names
    modelNames = matlabshared.multibody.internal.urdf.Model.getValidSDFModeNames(robotEls);
end
