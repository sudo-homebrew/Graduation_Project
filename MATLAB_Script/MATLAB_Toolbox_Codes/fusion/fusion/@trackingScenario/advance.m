function running = advance(obj)
%advance Advance the tracking scenario simulation by one time step.
%   isRunning = advance(scene) moves the platforms forward in time,
%   returning true when simulation is still in progress and false when
%   simulation is complete.
%   
%   The first call to advance() changes the SimulationStatus from
%   NotStarted to InProgress.  The time to which each platform is moved is
%   governed by the UpdateRate and InitialAdvance properties:
%
%   - When UpdateRate is positive, the simulation time of the first call
%     to advance() is governed by the InitialAdvance property:  the
%     simulation time is set to zero if InitialAdvance is set to Zero,
%     or set to the reciprocal of the UpdateRate when InitialAdvance is set
%     to UpdateInterval.  In either case, subsequent calls to advance()
%     increment the SimulationTime by the reciprocal of the update rate.
%     
%   - When UpdateRate is zero, advance() ignores the setting of the
%     InitialAdvance property.  The scenario adjusts the time to the value
%     required to update any sensor or emitter on any platform in the
%     scenario.  For example, if a scenario has two sensors where the first
%     has an update rate of 2 Hz and a second has an update rate of 5 Hz,
%     then the first seven calls to advance would adjust the simulation
%     times to 0, 0.2, 0.4, 0.5, 0.6, 0.8 and 1.0 seconds, respectively.
%
%     It is required to call the scenario's detect() or emit() methods
%     between successive calls to advance(); alternatively, they can be
%     rescheduled by calling the detect() or emit() individually for each
%     platform in the simulation.  If the program fails to call either the
%     scenario's emit() or detect() methods, simulation time will not move
%     forward, and the next call to advance() will generate an error. 
%     
%     Simulation ends when all platforms equipped with a sensor or emitter
%     have completed moving, all platforms have completed moving, or the
%     StopTime is reached, whichever is sooner.  You must have at least one
%     sensor or emitter defined on any platform in order to use this mode.
%
%   A simulation may be restarted by calling the scenario's restart method.
%
%   Example
%   -------
%   % Create a new scenario
%   scene = trackingScenario;
%
%   % Create a platform
%   plat = platform(scene);
% 
%   % Follow a circular trajectory 1 m in radius completing in one second.
%   plat.Trajectory = waypointTrajectory('Waypoints', [0 1 0; 1 0 0; 0 -1 0; -1 0 0; 0 1 0], ...
%                      'TimeOfArrival', [0; 0.25; .5; .75; 1.0]);
% 
%   % Perform the simulation
%   while advance(scene)
%       p = pose(plat);
%       fprintf('Time = %f ', scene.SimulationTime);
%       fprintf('Position = [');
%       fprintf('%f ', p.Position);
%       fprintf('] Velocity = [');
%       fprintf('%f ', p.Velocity);
%       fprintf(']\n');
%   end
%
%   See also restart, record, detect, kinematicTrajectory, waypointTrajectory.

%   Copyright 2018-2021 The MathWorks, Inc.

running = advance@radarfusion.internal.scenario.Scenario(obj);
