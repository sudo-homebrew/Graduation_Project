function parser = validateInputs(operation,varargin)
%This function is for internal use only. It may be removed in the future.
%
% This function validates parameter value for each parameter name. It checks
% parameter value dimensions, types and throw error if values are invalid.

%   Copyright 2020 The MathWorks, Inc.

    parser = inputParser;

    validationVector3 = @(x) validateattributes(x,{'numeric'},{'numel',3, 'nonnan','finite'});
    validationVector4 = @(x) validateattributes(x,{'numeric'},{'numel',4, 'nonnan','finite'});
    validationScalar = @(x) validateattributes(x,{'numeric'},{'scalar', 'nonnan','finite'});
    validationLogical = @(x) any(validatestring(x,{'on','off'}));
    validationIndex = @(x) any(validatestring(x,{'0','1'}));

    switch(operation)
      case 'SetModel'
        addParameter(parser,'Position', {}, validationVector3);
        addParameter(parser,'Orientation', {}, validationVector4);
        addParameter(parser,'SelfCollide', {}, validationLogical);
        addParameter(parser,'EnableWind', {}, validationLogical);
        addParameter(parser,'IsStatic', {}, validationLogical);
      case 'SetLink'
        addParameter(parser,'Position', {}, validationVector3);
        addParameter(parser,'Orientation', {}, validationVector4);
        addParameter(parser,'Mass', {}, validationScalar);
        addParameter(parser,'ProductOfInertia', {}, validationVector3);
        addParameter(parser,'PrincipalMoments', {}, validationVector3);
        addParameter(parser,'SelfCollide', {}, validationLogical);
        addParameter(parser,'Gravity', {}, validationLogical);
        addParameter(parser,'Kinematic', {}, validationLogical);
        addParameter(parser,'EnableWind', {}, validationLogical);
        addParameter(parser,'Canonical', {}, validationLogical);
        addParameter(parser,'IsStatic', {}, validationLogical);
      case 'SetJoint'
        addParameter(parser,'Position', {}, validationVector3);
        addParameter(parser,'Orientation', {}, validationVector4);
        addParameter(parser,'FudgeFactor', {}, validationScalar);
        addParameter(parser,'CFM', {}, validationScalar);
        addParameter(parser,'SuspensionCFM', {}, validationScalar);
        addParameter(parser,'SuspensionERP', {}, validationScalar);
        addParameter(parser,'Axis', {}, validationIndex);
        addParameter(parser,'Angle', {}, validationScalar);
        addParameter(parser,'XYZ', {}, validationVector3);
        addParameter(parser,'Damping', {}, validationScalar);
        addParameter(parser,'Friction', {}, validationScalar);
    end

    parse(parser,varargin{:});

end
