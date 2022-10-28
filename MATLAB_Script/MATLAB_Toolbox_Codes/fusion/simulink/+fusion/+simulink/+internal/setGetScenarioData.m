function scenarioData = setGetScenarioData(scenarioName,varargin)
%SETGETSCENARIODATA Set or get the scenario data for Simulink
%
%  This function is for internal use only and may be removed in a future
%  release.
%
% Copyright 2021 The MathWorks, Inc.

persistent trackingScenarioDataMap;

mlock;

if isempty(trackingScenarioDataMap)
    trackingScenarioDataMap = containers.Map('KeyType','char','ValueType','any');
end

% Data is stored in a map with key as modelName/ScenarioName . This way
% scenario data is generated only once per model per scneario.
modelName = get_param(bdroot, 'Name');
scenarioName = [modelName,'/',scenarioName];

if nargin > 1
    if ischar(varargin{1}) && strcmpi(varargin{1},'clear')
        if trackingScenarioDataMap.isKey(scenarioName)
            % clear previously stored data.
            trackingScenarioDataMap.remove(scenarioName);
        end
    else
        % Setting the data
        trackingScenarioDataMap(scenarioName) = varargin{1};
        scenarioData = trackingScenarioDataMap(scenarioName);
    end
else
    % Getting the data
    if trackingScenarioDataMap.isKey(scenarioName)
        scenarioData = trackingScenarioDataMap(scenarioName);
    else
        scenarioData = [];
    end
end

end

