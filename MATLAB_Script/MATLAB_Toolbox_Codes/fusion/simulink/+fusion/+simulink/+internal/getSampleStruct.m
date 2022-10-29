% GETSAMPLESTRUCT Get sample structure based on the type
%
%  This function is for internal use only and may be removed in a future
%  release.
%
% Copyright 2021 The MathWorks, Inc.

function out = getSampleStruct(type,varargin)
switch(type)
    case 'Poses'
        out = fusion.internal.interfaces.DataStructures.platformPoseStruct(false);

    case 'Detections'
        measParams = fusion.internal.interfaces.DataStructures.measurementParametersStruct;
        if nargin>1
            measSize = varargin{1};
        else
            measSize = 3; %If size is not provided, use 3 as default.
        end            
        out = struct('Time',0, ...
            'Measurement',zeros(measSize,1),... 
            'MeasurementNoise',zeros(measSize),...
            'SensorIndex',0 ,...
            'ObjectClassID',0 ,...
            'MeasurementParameters',measParams);

    case 'CoverageConfig'
        out = fusion.internal.interfaces.DataStructures.coverageConfigStruct;
        %Orientation is always a 3x3 rotaion matrix
        out.Orientation = eye(3);

    case 'SensorConfig'
        out = fusion.internal.interfaces.DataStructures.sensorConfigStruct;
        
    case 'Emission'
        %common struct for radarEmission and sonarEmission
        out = fusion.internal.interfaces.DataStructures.radarEmissionStruct(false);
        out.SourceLevel = 0;       %Specific to sonarEmission
        out.TargetStrength = 0;    %Specific to sonarEmission

    case 'EmitterConfig'
        out = fusion.internal.interfaces.DataStructures.emitterConfigStruct;
        
    case 'PointClouds'
        if nargin>1
            numPoints = varargin{1};
        else
            numPoints = 1; % If numPoints is not provided, use 1 as default.
        end 
        out = struct('Points',NaN(numPoints,3), ... 
            'Clusters',NaN(numPoints,2), ...
            'NumPoints',0);
        
    case 'MeasurementParameters'
        out = fusion.internal.interfaces.DataStructures.measurementParametersStruct;
        out.Frame = fusionCoordinateFrameType.Rectangular;
        
end
end