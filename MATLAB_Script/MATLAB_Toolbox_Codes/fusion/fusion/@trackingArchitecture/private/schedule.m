function toRun = schedule(obj,time)
%SCHDULE  Schedule which nodes to run
% toRun = SCHEDULE(OBJ,TIME) checks which nodes need to run at this time,
% TIME

% Copyright 2020 The MathWorks, Inc.

numNodes = numel(obj.pNodes);
toRun = false(1,numNodes);
for i = 1:numNodes
    thisNodeNextUpdate = nextUpdateTime(obj.pNodes{i});
    toRun(i) = thisNodeNextUpdate <= time;
end
end