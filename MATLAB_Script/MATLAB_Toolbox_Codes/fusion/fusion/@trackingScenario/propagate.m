function propEmis = propagate(obj,varargin)
%PROPAGATE Propagate emissions in the scenario
%   EMPROP = PROPAGATE(SCENE,EMTX) returns emissions, EMPROP, which are
%   the combination of the input emissions, EMTX, as well as the
%   reflections of each of the input emissions off each of the platforms
%   in the scenario, SCENE. EMTX and EMPROP are arrays of radarEmission or
%   sonarEmission objects. Emissions are always returned as a cell array
%   of emissions from radar or sonar.
%
%   EMPROP = PROPAGATE(SCENE,EMTX,'HasOcclusion',WITHOCCLUSION) allows you
%   to specify whether the radar channel models occlusion. By default,
%   WITHOCCLUSION is true.
%
%   % Example: Propagate the emissions from two platforms in the scenario
%   ts = trackingScenario('UpdateRate', 1);
%   plat1 = platform(ts);
%   plat1.Trajectory.Position = [0,0,0];
%   emitter1 = radarEmitter(1, 'UpdateRate', 1);
%   plat1.Emitters = emitter1;
%   plat2 = platform(ts);
%   plat2.Trajectory.Position = [100,0,0];
%   emitter2 = radarEmitter(2, 'UpdateRate', 1);
%   plat2.Emitters = emitter2;
%   
%   advance(ts);
%   emtx = emit(ts); % Transmitted emissions
%   emprop = PROPAGATE(ts, emtx, 'HasOcclusion', true)
% 
%   % The last emission was emitted by emitter 1 and propagated off of
%   % platform 2
%   disp(emprop{end})
%
%   See also trackingScenario, emit, detect, radarChannel,
%   underwaterChannel.

%   Copyright 2019-2020 The MathWorks, Inc.

propEmis = propagate@radarfusion.internal.scenario.Scenario(obj,varargin{:});
