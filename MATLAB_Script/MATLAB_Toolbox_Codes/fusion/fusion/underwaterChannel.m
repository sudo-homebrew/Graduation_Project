function emissOut = underwaterChannel(emissIn, platsIn, varargin)
%underwaterChannel  Underwater propagation and reflection of sonar signals
%   EMPROP = underwaterChannel(EMTX, PLATS) returns sonar emissions,
%   EMPROP, which are the combination of the input sonar emissions, EMTX,
%   as well as the reflections of each of the input emissions off each of
%   the platforms, PLATS and the ocean's surface (defined at z = 0). EMTX
%   and EMPROP are arrays of sonarEmission objects.
%   
%   If EMTX is a cell array of sonarEmission objects, then EMPROP is a
%   cell array of sonarEmission objects. If EMTX is an array of
%   sonarEmission objects, then EMPROP is an array of sonarEmission
%   objects.
%
%   PLATS is cell array of platform objects provided by the Platforms
%   property on trackingScenario or an array of platform structures with
%   fields as listed <a href="matlab:help('fusion.internal.interfaces.DataStructures/platformStruct')">here</a>.
%
%   If an array of platform structures is used, the PlatformID and Position
%   fields are required. All of the remaining fields will be assigned
%   default values if not specified.
%
%   EXAMPLE 1: Reflect an sonar emission off a platform.
%
%   % Define a sonar emission.
%   emTx = sonarEmission('PlatformID', 1, 'EmitterIndex', 1, ...
%                   'OriginPosition', [0 0 10]);
%   
%   % Define a platform to reflect the emission.
%   plat = struct('PlatformID', 2, 'Position', [10 0 10], ...
%                   'Signatures', {sonarEmission});
%
%   % Reflect the signal off the platform.
%   emProp = underwaterChannel(emTx, plat)
%
%   EXAMPLE 2: Reflect a sonar emission using trackingScenario and sonarEmitter.
%
%   % Define a tracking scenario.
%   scenario = trackingScenario;
%
%   % Create a sonarEmitter to mount on a platform.
%   emitter = sonarEmitter(1);
%
%   % Mount the emitter on a platform in the scenario.
%   plat = platform(scenario, 'Emitters', emitter);
%
%   % Add a platform to reflect the emitted signal.
%   tgt = platform(scenario);
%   tgt.Trajectory.Position = [30 0 10];
%
%   % Emit the signal.
%   emTx = emit(plat, scenario.SimulationTime)
%
%   % Reflect the emission off the platforms in the scenario.
%   emProp = underwaterChannel(emTx, scenario.Platforms)
%
%   See also: sonarEmitter, sonarSensor, sonarEmission, trackingScenario.

%#codegen
%   Copyright 2018-2021 The MathWorks, Inc.


% Parse name-value pairs
defaultPairs = struct('DefaultReferenceFrame','NED');
mcosParser = fusion.internal.mixin.MCOSConstructorParser;
value = parseConstructorInputs(mcosParser, defaultPairs, varargin{:});
reference = value{1};

% Parse emissions and platforms
emiss = fusion.internal.remotesensors.EmissionUtilities.parseEmissions(emissIn,reference, mfilename);
plats = fusion.internal.remotesensors.EmissionUtilities.parsePlatforms(platsIn,reference, mfilename);

% Are any emissions or platforms in scenario?
if isempty(plats) || fusion.internal.remotesensors.CommonUtilities.isIndexedEmpty(emiss)
    emissOut = emiss;
    return
end

% Specify the number of path from source to destination
numPaths = 2;

% The receiver's location is unknown, so all possible combinations are
% considered. This requires calculating the signal propagation on both the
% receive and transmit path:
%   1. Receive path:  transmitter -> platform's signature
%   2. Transmit path: platform's reflected signal -> a different platform
% Where, here, the "reflecting" platform is viewed as a repeater whose
% receive and transmit gain is determined by its signature.

% Receive path: signal received at platform from a transmitter
isIlluminated = isPlatformIlluminated(emiss,plats,numPaths);

% Are any platforms illuminated?
numRxEmiss = sum(isIlluminated(:));
if numRxEmiss<1
    emissOut = emiss;
    return % no
end

% Allocate output
numPlats = numel(plats);
numRefl = numRxEmiss*numPlats;

% Output format follows input format
if iscell(emiss)
    thisEmiss = {emiss{1}};
else
    thisEmiss = emiss(1);
end
reflections = repmat(thisEmiss,numRefl,1);
topbounces = repmat(thisEmiss,numRefl,1);

iRefl = 1;
iTop = 1;
for iEmiss = 1:numel(emiss)
    % Loop over each signal paths
    for iPaths = 1:numPaths
        isIllumPlats = isIlluminated(:,iEmiss,iPaths);
        
        % Did this emission illuminate any platforms?
        if any(isIllumPlats)
            thisEmiss = fusion.internal.remotesensors.CommonUtilities.getIndexed(emiss,iEmiss);
        
            % Flag to indicate topBouncePath path
            isTopBouncePath = iPaths==2;
            
            if isTopBouncePath
                % For topBouncePath we need to set position as the
                % image of position w.r.t surface at z=0
                thisEmiss.OriginPosition(3) = -thisEmiss.OriginPosition(3);
                thisEmiss.OriginVelocity(3) = -thisEmiss.OriginVelocity(3);
            end
            
            [reflections,iRefl,topbounces,iTop] = reflectEmission(thisEmiss,isTopBouncePath,isIllumPlats,plats,reflections,iRefl,topbounces,iTop);
        end
    end
end

numRefl = iRefl-1;
numTop = iTop-1;
emissOut = assembleOutput(emiss,reflections,numRefl,topbounces,numTop);
end

function [reflections,iRefl,topbounces,iTop] = reflectEmission(thisEmiss,isTopBouncePath,isIllumPlats,plats,reflections,iRefl,topbounces,iTop)
% Adds reflections and top bounces for this emission from every platform
% that is illuminated by this emission to the allocated reflections array.
% iRefl is the index where the next reflection should be added to the
% reflections array. iTop is the index where the next reflection should be
% added to the topbounces array.

iIllumPlats = find(isIllumPlats);
for iPlat = 1:numel(iIllumPlats)
    % Platform receiving this emission
    iReflPlat = iIllumPlats(iPlat);
    reflPlat = plats(iReflPlat);
    
    % Is transmitter on receiving platform?
    if reflPlat.PlatformID==thisEmiss.PlatformID
        continue % Yes, skip to next platform
    end
    
    [reflections,iRefl,topbounces,iTop] = reflectToNextPlat(thisEmiss,isTopBouncePath,reflPlat,plats,reflections,iRefl,topbounces,iTop);
end
end

function [reflections,iRefl,topbounces,iTop] = reflectToNextPlat(thisEmiss,isTopBouncePath,reflPlat,plats,reflections,iRefl,topbounces,iTop)
% Adds reflections from this emission, thisEmiss, that are reflected from
% the illuminated platform, reflPlat, to the next platform. Also adds top
% bounces from this emission.

reflPos = reflPlat.Position(:);
reflVel = reflPlat.Velocity(:);

% Receive emission at this platform
[rgIn,rrIn,urIn] = receiveEmission(thisEmiss,reflPos,reflVel);

% Salt water propagation loss from transmitter to receiver
% propLoss = 10*log(r)+ thorpeLossCoefficient * r;
propLossdB = thorpeloss(thisEmiss.CenterFrequency)*rgIn + 10*log10(rgIn);

% Use same format for orientation in output signals as was used
% in input signals
usesQuat = isa(thisEmiss.Orientation,'quaternion');
for iNextPlat = 1:numel(plats)
    % Next platform to which the emission is reflected
    nextPlat = plats(iNextPlat);
    nextPos = nextPlat.Position(:);
    
    % Is platform towards which the reflection is transmitted the
    % same as the platform that received the signal from the
    % transmitter?
    if nextPlat.PlatformID==reflPlat.PlatformID
        if isTopBouncePath
            topBounchPathSig = thisEmiss;
            vecOut = nextPlat.Position(:) - thisEmiss.OriginPosition(:);
            topBounchPathSigOrient = calculateOrientation(vecOut,usesQuat);
            topBounchPathSig.Orientation = topBounchPathSigOrient;
            topbounces = fusion.internal.remotesensors.CommonUtilities.setIndexed(topbounces,iTop,topBounchPathSig);
            iTop = iTop+1;
        end
        continue % Yes, skip to next platform
    end
    
    % Re-emit received energy towards next platform
    posIn = reflPos;
    if isTopBouncePath
        posIn(3) = -reflPos(3);
    end
    [targetStrength,orient] = transmitEmission(thisEmiss,urIn,reflPlat,posIn,nextPos);
    
    % Update reflected signal fields
    thisRefl = thisEmiss;
    thisRefl.PlatformID = reflPlat.PlatformID;
    thisRefl.OriginPosition(:) = reflPos;
    thisRefl.OriginVelocity(:) = reflVel;
    thisRefl.Orientation(:) = orient;
    
    if isTopBouncePath
        % For topBouncePath we need to set position as the
        % image of position w.r.t surface at z=0
        thisRefl.OriginPosition(3) = -thisRefl.OriginPosition(3);
        thisRefl.OriginVelocity(3) = -thisRefl.OriginVelocity(3);
    end
    
    thisRefl.FieldOfView(:) = [180, 180];
    thisRefl.SourceLevel = thisRefl.SourceLevel-propLossdB;
    thisRefl.TargetStrength = thisRefl.TargetStrength+targetStrength;
    thisRefl.PropagationRange = thisRefl.PropagationRange+rgIn;
    thisRefl.PropagationRangeRate = thisRefl.PropagationRangeRate+rrIn;
    
    reflections = fusion.internal.remotesensors.CommonUtilities.setIndexed(reflections,iRefl,thisRefl);
    iRefl = iRefl+1;
end
end

function emissOut = assembleOutput(emiss,reflections,numRefl,topbounces,numTop)
% Concatenate input, reflected, and top bounce emissions
% - If input was a cell array then output is also a cell array, otherwise
%   it is an array of emission objects.

numIn = numel(emiss);
usesCell = iscell(emiss);
if usesCell
    % Allocate space
    emissOut = repmat({emiss{1}},numIn+numTop+numRefl,1);
    
    % Add inputs to the output
    for m = 1:numIn
        emissOut{m} = emiss{m};
    end
    
    % Add top bounce reflections to the output
    for m = 1:numTop
        emissOut{m+numIn} = topbounces{m};
    end
    
    % Add reflections to the output
    for m = 1:numRefl
        emissOut{m+numIn+numTop} = reflections{m};
    end
else
    % Allocate space
    emissOut = repmat(emiss(1),numIn+numTop+numRefl,1);
    
    % Add inputs to the output
    for m = 1:numIn
        emissOut(m) = emiss(m);
    end
    
    % Add top bounce reflections to the output
    for m = 1:numTop
        emissOut(m+numIn) = topbounces(m);
    end
    
    % Add reflections to the output
    for m = 1:numRefl
        emissOut(m+numIn+numTop) = reflections(m);
    end
end
end

function [TSdB, orientOut] = transmitEmission(emiss,urIn,reflPlat,reflPos,nextPos)
% Compute the bistatic RCS to account for the receive gain and the transmit
% gain of the platform reflecting the emission and the direction of the
% reflected emission's propagation to the next platform.

vecOut = nextPos-reflPos;

% Convert to spherical coordinates (az,el)
[azOut,elOut] = cart2sph(vecOut(1),vecOut(2),vecOut(3));
azOut = rad2deg(azOut);
elOut = rad2deg(elOut);
azOut = fusion.internal.UnitConversions.interval(azOut,[-180 180]);
elOut = fusion.internal.UnitConversions.interval(elOut,[-180 180]);

[x,y,z] = sph2cart(deg2rad(azOut),deg2rad(elOut),1);
urOut = [x(:) y(:) z(:)]';

% Compute RCS including refraction effects
TSdB = getBistaticTS(reflPlat,urIn,urOut,emiss.CenterFrequency);

% Direction of emission propagation to the next platform as an orientation
% matrix.
orient = lookOrientation(urOut);

% Use same format for orientation in output signals as was used
% in input signals
if isa(emiss.Orientation,'quaternion')
    orientOut = quaternion(orient,'rotmat','frame');
else
    orientOut = orient;
end
end

function orient = lookOrientation(vLook)
u1 = vLook/norm(vLook);
if abs(u1(1))>eps || abs(u1(2))>eps
    v2 = [-u1(2);u1(1);0];
    u2 = v2/norm(v2);
    u3 = cross(u1,u2);
else
    v3 = [-u1(3);0;u1(1)];
    u3 = v3/norm(v3);
    u2 = cross(u3,u1);
end
orient = [u1 u2 u3]';
end

function orientOut = calculateOrientation(vLook,isQuat)
orient = lookOrientation(vLook);
if isQuat
    orientOut = quaternion(orient,'rotmat','frame');
else
    orientOut = orient;
end
end

function [rgIn, rrIn, urIn] = receiveEmission(emiss,posIn,velIn)
% Compute the propagation range, range-rate, and direction at which the
% emission is received at the platform from which it will be reflected.

emissPos = emiss.OriginPosition(:);
emissVel = emiss.OriginVelocity(:);
emissOrient = fusion.internal.remotesensors. ...
    CommonUtilities.getRotmat(emiss.Orientation);

vecIn = posIn-emissPos;

if emiss.PropagationRange==0
    % Rotate receiving platform into the transmitter's frame
    vecTx = emissOrient*vecIn;
    
    % Convert to spherical coordinates (az,el)
    [azTx,elTx,rgTx] = cart2sph(vecTx(1),vecTx(2),vecTx(3));
    azTx = rad2deg(azTx);
    elTx = rad2deg(elTx);
    azTx = fusion.internal.UnitConversions.interval(azTx,[-180 180]);
    elTx = fusion.internal.UnitConversions.interval(elTx,[-180 180]);
    
    % Add in atmospheric refraction
    rgIn = rgTx;
    
    % Compute range-rate and RCS including refraction effects
    [x,y,z] = sph2cart(deg2rad(azTx),deg2rad(elTx),1);
    urIn = [x(:) y(:) z(:)]';
else
    rgIn = norm(vecIn);
    urIn = vecIn/rgIn;
end

rrIn = urIn'*(velIn-emissVel);
end

function tsdB = getBistaticTS(platIn,vecIn,vecOut,freq)
% Returns the TS of the platform in decibels.
%
% plat:     Pose for platform whose TS is being calculated
% vecIn:    Vector defining direction of input signal to
%           platform in scenario coordinates
% vecOut:   Vector defining direction of output signal from
%           platform in scenario coordinates
% freq:     Frequency in hertz of input signal

if iscell(platIn)
    plat = platIn{1};
else
    plat = platIn;
end

% Rotate vectors into platform's body frame
orient = fusion.internal.remotesensors.CommonUtilities.getRotmat(plat.Orientation);
vecIn = orient*vecIn;
vecOut = orient*vecOut;

% Make both vecIn and vecOut point away from platform
vecIn = -vecIn;

% Convert vectors into spherical coordinates (az,el) in platform's body frame
[azIn,elIn] = cart2sph(vecIn(1),vecIn(2),vecIn(3));
azIn = rad2deg(azIn);
elIn = rad2deg(elIn);

[azOut,elOut] = cart2sph(vecOut(1),vecOut(2),vecOut(3));
azOut = rad2deg(azOut);
elOut = rad2deg(elOut);

% Compute lookup angle
% - MBET says that equivalent monostatic angle is 1/2 of the
%   bistatic angle when bistatic angle is small (e.g. < 90 deg)
% - For monostatic case, bsAng == 0
% - Note: MBET will be used to estimate the bistatic target strength for
%   angles > 90 degrees as well
bsAng = azOut-azIn;
bsAng = fusion.internal.UnitConversions.interval(bsAng,[-180 180]);
azAng = azIn+bsAng/2;
azAng = fusion.internal.UnitConversions.interval(azAng,[-180 180]);

bsAng = elOut-elIn;
bsAng = fusion.internal.UnitConversions.interval(bsAng,[-180 180]);
elAng = elIn+bsAng/2;
elAng = fusion.internal.UnitConversions.interval(elAng,[-180 180]);

% Lookup TargetStrength at computed angle
signatures = plat.Signatures;

isFound = false;
thisSig = tsSignature();
for m = 1:numel(signatures)
    if iscell(signatures) 
        if isa(signatures{m},'tsSignature')
            thisSig = signatures{m};
            isFound = true;
        end
    else
        if isa(signatures(m),'tsSignature')
            thisSig = signatures(m);
            isFound = true;
        end
    end
    
    if isFound
        break
    end
end

coder.internal.errorIf(~isFound, 'shared_radarfusion:RemoteSensors:SignatureNotFound','TargetStrength',plat.PlatformID);
tsdB = value(thisSig, azAng, elAng, freq);
end

function isIlluminated = isPlatformIlluminated(emiss,plats,numPaths)
%isPlatformInFieldOfView  True when platform lies inside the emission's field of view
%   INFOV = isPlatformInFieldOfView(EMISS,PLAT,NUMPATHS) returns true when
%   any portion of the platform lies within the emission's field of view.
%
%   EMISS is a scalar sonarEmission object or the corresponding emission
%   structure.
%
%   PLAT is a scalar platform structure.
%
%   NUMPATHS is 2, corresponding to the direct-path (path 1) and the top
%   bounce path (path 2).

numEmiss = numel(emiss);
numPlats = numel(plats);
isIlluminated = false(numPlats,numEmiss,numPaths);

for iEmiss = 1:numEmiss
    thisEmiss = fusion.internal.remotesensors.CommonUtilities.getIndexed(emiss,iEmiss);
    
    for iThisPlat = 1:numPlats
        thisPlat = plats(iThisPlat);
        
        % Skip if signal originated from this platform
        if thisPlat.PlatformID==thisEmiss.PlatformID
            continue
        end
        
        % Is the platform illuminated by the transmitted signal?
        inFoV = fusion.internal.remotesensors.EmissionUtilities.isPlatformInFieldOfView(thisEmiss,thisPlat,true);
        isIlluminated(iThisPlat,iEmiss,1) = inFoV(1);

        if numPaths>1
            % Calculate illumination from top bounce path
            mirrorPlat = thisPlat;
            mirrorPlat.Position(3) = -thisPlat.Position(3);
            inFoV = fusion.internal.remotesensors.EmissionUtilities.isPlatformInFieldOfView(thisEmiss,mirrorPlat,true);
            isIlluminated(iThisPlat,iEmiss,2) = inFoV(1);
        end
    end
end
end

function y = thorpeloss(f)
%Thorpeloss  Frequency-dependent attenuation 
    % Compute frequency-dependent attenuation in dB/m using Thorpe's
    % equation (pg. 108 in [1]).
    freqnkHz=f/1e3;         
    y = ((3.3*10^-3)+(.11*freqnkHz.^2)./(1+freqnkHz.^2)+(44*freqnkHz.^2)./(4100+freqnkHz.^2)+(3*10^-4)*freqnkHz.^2);
    y = y/1000; % dB/m
end
