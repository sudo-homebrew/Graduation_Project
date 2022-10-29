function isReached = checkIfGoalIsReached(planner, goalState, newState)
%CHECKIFGOALISREACHED Default GoalReachedFcn for plannerRRT

%   Copyright 2019 The MathWorks, Inc.

%#codegen

isReached = false;
threshold = 0.5;
if planner.StateSpace.distance(newState, goalState) < threshold
    isReached = true;
end

end

