function [lla, gnssVel, status] = stepImpl(obj, pos, vel)
%STEPIMPL Step method for gnssSensor object

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

% Constant values from WGS 84 Earth model.
[~, ~, mu, OmegaEDot] = fusion.internal.frames.wgs84ModelParams;

refFrame = obj.pRefFrame;
proto = obj.pInputPrototype;

% Preallocate outputs.
lla = NaN(size(pos), 'like', proto);
gnssVel = NaN(size(vel), 'like', proto);
maxNumSats = 27;
data = NaN(maxNumSats, 1, 'like', proto);
coder.varsize('data', [maxNumSats, 1], [1, 0]);
statusTemplate = struct('SatelliteAzimuth', data, ...
    'SatelliteElevation', data, ...
    'HDOP', data, 'VDOP', data);
status = repmat(statusTemplate, size(pos,1), 1);
   
receiverRangeError = obj.RangeAccuracy;
receiverRangeRateError = obj.RangeRateAccuracy;
    
currTime = obj.pCurrTime;
dt = 1 ./ obj.SampleRate;
gpsWeek = obj.pGPSWeek;

for idx = 1:size(pos, 1)
    % Orbital parameters.
    [e0, i0, OmegaRefDot, ARef, Omega0, omega0, M0] ...
        = matlabshared.internal.gnss.OrbitalParameters.nominal;

    % Get ground truth receiver position and velocity using input.
    receiverLLA = refFrame.frame2lla(pos(idx,:), obj.ReferenceLocation);
    receiverPosECEF = fusion.internal.frames.lla2ecef(receiverLLA);
    receiverVelECEF = refFrame.frame2ecefv(vel(idx,:), ...
        receiverLLA(1), receiverLLA(2));

    % Calculate satellite positions, velocities, and visibilities using
    % orbital parameters and receiver position.
    [satPos, satVel] = matlabshared.internal.gnss.orbitParametersToECEF(...
        gpsWeek, currTime, 0, 0, ...
        ARef, 0, 0, ...
        mu, 0, 0, ...
        M0, e0, omega0, ...
        i0, 0, ...
        0, 0, 0, 0, 0, 0, ...
        OmegaEDot, OmegaRefDot, Omega0, 0);
    [isSatVisible, satAz, satEl] = nav.internal.gnss.satelliteStatus( ...
        receiverLLA, obj.MaskAngle, satPos);

    % Calculate pseudoranges and pseudorange rates using satellite and
    % receiver positions and velocities.
    [p, pdot] = nav.internal.gnss.calculatePseudoranges(satPos, satVel, ...
        receiverPosECEF, receiverVelECEF);
    
    % Add measurement noises.
    p = p + receiverRangeError .* stepRandomStream(obj, ...
        size(p, 1), size(p, 2));
    pdot = pdot + receiverRangeRateError .* stepRandomStream(obj, ...
        size(pdot, 1), size(pdot, 2));
    
    % Estimate receiver position and velocity using pseudoranges,
    % pseudorange rates, and satellite positions and velocities.
    initPosECEF = obj.pInitPosECEF;
    initVelECEF = obj.pInitVelECEF;
    [posECEF, gnssVelECEF, dopMatrix] ...
        = nav.internal.gnss.computeLocation( ...
        p(isSatVisible), pdot(isSatVisible), ...
        satPos(isSatVisible,:), satVel(isSatVisible,:), ...
        initPosECEF, initVelECEF);
    
    [hdop, vdop] = nav.internal.gnss.calculateDOP(dopMatrix, refFrame, ...
        obj.ReferenceLocation);

    % Package output.
    lla(idx,:) = fusion.internal.frames.ecef2lla(posECEF);
    gnssVel(idx,:) = refFrame.ecef2framev(gnssVelECEF, ...
        lla(idx,1), lla(idx,2));
    status(idx,:) = struct('SatelliteAzimuth', satAz(isSatVisible), ...
        'SatelliteElevation', satEl(isSatVisible), ...
        'HDOP', hdop, 'VDOP', vdop);
    
    currTime = currTime + dt;
end

obj.pCurrTime = currTime;
end

function noise = stepRandomStream(obj, numSamples, numChans)
% Noise (random number) generation.
if strcmp(obj.RandomStream, 'Global stream')
    noise = randn(numSamples, numChans, 'like', obj.pInputPrototype);
else
    noise = randn(obj.pStream, numSamples, numChans, ...
        class(obj.pInputPrototype));
end
end
