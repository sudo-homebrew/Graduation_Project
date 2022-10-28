function [recordings, RNGS] = monteCarloRun(varargin)
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

narginchk(2,6)

prevRNG = rng; % Keep the previous RNG state
[scene,numRuns,useParallel,inseed] = localParseInputs(varargin{:});

useParallel = useParallel && coder.const(fusion.internal.canUsePCT);

if isempty(inseed)
    seeds = randperm(2^32-1,numRuns);
elseif isscalar(inseed)
    validateattributes(inseed,{'numeric'},{'real','nonnegative','integer','<',intmax('uint32')},mfilename,'SEEDS')
    rng(inseed, 'Twister')
    seeds = randperm(2^32-1,numRuns);
else
    validateattributes(inseed,{'numeric'},{'real','nonnegative','integer','vector','numel',numRuns,'<',intmax('uint32')},mfilename,'SEEDS')
    seeds = inseed;
end

scenes = repmat(scene(:),1,numRuns);
seeds = repmat(seeds(:)',size(scenes,1),1);

if useParallel
    [recordings, RNGS] = localRunScenariosParallel(numRuns,scenes,seeds);
else
    [recordings, RNGS] = localRunScenarios(numRuns,scenes,seeds);
end

% Return the RNG to previous state
rng(prevRNG)
end

function [recordings, RNGS] = localRunScenarios(numRuns,scenes,seeds)
numScenes = size(scenes,1);
recordings = cell(numScenes,numRuns);
RNGS = repmat(rng, numScenes,numRuns);
for i = 1:numScenes
    for j = 1:numRuns
        thisSeed = seeds(i,j);
        [recordings{i,j}, RNGS(i,j)] = scenes(i,j).record('IncludeEmitters',true,...
            'IncludeSensors',true,'InitialSeed',thisSeed,...
            'HasOcclusion',true,'RecordingFormat','Recording');
    end
end
recordings = reshape([recordings{:}],numScenes,numRuns);
end

function [recordings, RNGS] = localRunScenariosParallel(numRuns,scenes,seeds)
numScenes = size(scenes,1);
recordings = cell(numScenes,numRuns);
RNGS = repmat(rng, numScenes,numRuns);
parfor i = 1:numScenes*numRuns
    thisScene = scenes(i);
    thisSeed = seeds(i);
    [recordings{i}, RNGS(i)] = thisScene.record('IncludeEmitters',true,...
        'IncludeSensors',true,'InitialSeed',thisSeed,'HasOcclusion',true,...
        'RecordingFormat','Recording');
end
recordings = reshape([recordings{:}],numScenes,numRuns);
end

function [scene,numRuns,useParallel,inseed] = localParseInputs(varargin)
p = inputParser;
p.addRequired('scene', @(x) validateattributes(x,{'cell','trackingScenario'},{'nonempty'},mfilename,'scene'));
p.addRequired('numRuns', @(x) validateattributes(x,{'numeric'},{'real','scalar','positive','integer'},mfilename,'numRuns'));
p.addParameter('UseParallel', false, @(x) validateattributes(x, {'logical','numeric'}, {'binary','scalar'},mfilename,'UseParallel'));
p.addParameter('InitialSeeds', [], @(x) validateattributes(x, {'numeric'}, {}, mfilename, 'seeds'));
parse(p, varargin{:});

s = p.Results.scene;
if iscell(s)
    validateattributes(s{1}, {'trackingScenario'}, {}, mfilename,'scene{:}')
    scene = [s{:}];
else
    scene = s;
end
    
numRuns = p.Results.numRuns;
useParallel = p.Results.UseParallel;
inseed = p.Results.InitialSeeds;
end