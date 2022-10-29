%monteCarloRun  Perform Monte Carlo realizations of a tracking scenario
% RECORDINGS = monteCarloRun(SCENE, NUMRUNS) runs a tracking scenario
% multiple times, each time with different random seed. These runs are
% called realizations. SCENE is an M-element array of trackingScenario
% objects or an M-element cell array of trackingScenario objects. NUMRUNS
% is the number of realizations required to run each tracking scenario. The
% output, RECORDINGS is an M-by-NUMRUNS array of trackingScenarioRecording
% objects.
%
% RECORDINGS = monteCarloRun(..., 'UseParallel', TF) allows you to set the
% flag, TF, to run scenarios in parallel. This option requires a Parallel
% Computing Toolbox license and an open parallel pool. By default, the
% value of the TF flag is false.
%
% RECORDINGS = monteCarloRun(..., 'InitialSeeds', SEEDS) allows you to
% specify initial random seeds for obtaining repeatable results. SEEDS must
% be a nonnegative integer in the range 0 to 2^32-1 or an array of NUMRUNS
% elements with nonnegative integers defined in that range. If specified as
% a scalar, an array of seed values will be randomly generated using the
% scalar as an initial seed.
%
% [RECORDINGS, RNGS] = monteCarloRun(...) additionally, returns the random
% number generator values at the beginning of each realization run. RNGS is
% an M-by-NUMRUNS array of struct with the same fields as the rng function
% returns.
%
% % Example: Run a scenario 2 times with automatic random seeds
% load ATCScenario.mat scenario % Load a scenario
% tic
% recordings = monteCarloRun(scenario, 2);
% disp("Time to run the scenarios: " + toc)
%
% % To use Parallel Computing Toolbox, you can set the UseParallel
% % flag to true
% tic
% recordings = monteCarloRun(scenario, 2, 'UseParallel', true);
% disp("Time to run the scenarios in parallel: " + toc)
% 
% See also: trackingScenario, trackingScenarioRecording, rng

 
%   Copyright 2019 The MathWorks, Inc.

