%oneRecordingStep    Returns a single recording step 
%   rNext = oneRecordingStep(scene, fmt, wEmitters, wSensors, wOcclusion)
%   returns one recording step of the trackingScenario, scene. See
%   trackingScenario/record for the use of the additional inputs.

%   Copyright 2019-2020 The MathWorks, Inc.

function rNext = oneRecordingStep(scene, fmt, coord, wEmitters, wSensors, wOcclusion)
    % Collect time and poses
    rNext.SimulationTime = scene.SimulationTime;
    if ~scene.IsEarthCentered && strcmp(coord,'Cartesian')
        rNext.Poses = platformPoses(scene,fmt);
    else
        rNext.Poses = platformPoses(scene,fmt,'CoordinateSystem',coord);
    end
    
    if scene.UpdateRate == 0 || wEmitters || wSensors
        % Collect emissions
        [emissions, emitterConfigurations, emitterPIDs] = emit(scene);
        
        % Propagate signals
        propEmis = propagate(scene, emissions, 'HasOcclusion', wOcclusion);
        
        if wEmitters
            rNext.Emissions = propEmis;
            rNext.EmitterConfigurations = emitterConfigurations;
            rNext.EmitterPlatformIDs = emitterPIDs;
        end
    end
    
    if scene.UpdateRate == 0 || wSensors
        % Collect detections
        [detections, sensorConfigurations, sensorPIDs] = detect(scene, propEmis, emitterConfigurations);
        
        % Collect point clouds
        [ptClouds, lidarConfigurations, lidarPIDs, clusters] = lidarDetect(scene);
        
        if wSensors
            rNext.Detections = detections;
            rNext.PointClouds = ptClouds;
            rNext.PointCloudClusters = clusters;
            rNext.SensorConfigurations = [sensorConfigurations;lidarConfigurations];
            rNext.SensorPlatformIDs = [sensorPIDs;lidarPIDs];
        end
    end
    
    if (wEmitters || wSensors)
        % Record coverage configurations after emitters and sensors moved.
        covcon = coverageConfig(scene);
        
        if ~isempty(covcon)
            indices = [covcon.Index];
            toKeep = ...
                ((indices<0) & wEmitters) ... Keep emitter coverage if wanted
                |((indices>0) & wSensors);   % Keep sensor coverage if wanted
            rNext.CoverageConfig = covcon(toKeep);
        end
    end
end