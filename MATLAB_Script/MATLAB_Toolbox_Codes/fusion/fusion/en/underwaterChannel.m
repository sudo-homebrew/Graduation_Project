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

 %   Copyright 2018-2021 The MathWorks, Inc.

