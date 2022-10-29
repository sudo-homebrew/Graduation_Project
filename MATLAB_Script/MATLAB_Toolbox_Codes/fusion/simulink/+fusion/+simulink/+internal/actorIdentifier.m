function actorIDs = actorIdentifier(x)
%ACTORIDENTIFIER function to extract the ActorID from truth input.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

if isempty(x)
    actorIDs = nan(0,1);
else
    if coder.target('MATLAB')
        actorIDs = vertcat(x.ActorID);
    else
        actorIDs = zeros(numel(x),1); 
        for i = 1:numel(x)
            actorIDs(i) = x(i).ActorID;
        end
    end
end
