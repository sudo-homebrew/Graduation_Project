function restart(obj)
%restart Restart simulation.
%   restart(scene) restarts the scenario, scene, at the beginning.
%
%   Example
%   -------
%   % Create a new scenario
%   scene = trackingScenario
% 
%   % add a platform
%   plat = platform(scene)
% 
%   % tell it to follow a 25 m trajectory along the x-axis at 20 m/s.
%   plat.Trajectory = waypointTrajectory('Waypoints',[0 0 0; 25 0 0], ...
%                      'TimeOfArrival', [0 25/20]);
% 
%   % Run the simulation once
%   while advance(scene)
%      p = pose(plat);
%      fprintf('The platform is located at %.2f m at t=%.0f ms\n', p.Position(1), scene.SimulationTime*1000)
%      pause(0.1)
%   end
%
%   % Restart the simulation and run again.
%   restart(scene);
%   while advance(scene)
%      p = pose(plat);
%      fprintf('The platform is located at %.2f m at t=%.0f ms\n', p.Position(1), scene.SimulationTime*1000)
%      pause(0.1)
%   end
%
%   See also advance, record.

%   Copyright 2018-2020 The MathWorks, Inc.

restart@radarfusion.internal.scenario.Scenario(obj);
