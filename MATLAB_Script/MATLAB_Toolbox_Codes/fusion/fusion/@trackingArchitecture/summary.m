function val = summary(obj)
%SUMMARY Summary of the tracking architecture
% SUMMARY(TA) provides a summary of the trackingArchitecture, TA, in a
% tabular form. The table includes the following columns:
%   System             - A description of the system organized as
%                        'T' or 'F' for tracker or fuser, repectively,
%                        followed by tracker or fuser index and their name. 
%                        For example, 'T1: MyTracker' is a tracker with
%                        index=1 and name 'MyTracker'.
%   ArchitectureInputs - A list of architecture input indices for this system
%   FuserInputs        - A list of indices that provide tracks to the fuser
%   ArchitectureOutput - The output index associated with this system

% Copyright 2020 The MathWorks, Inc.

numNodes = numel(obj.pNodes);
if numNodes > 0
    v = repmat(summary(obj.pNodes{1}),numNodes,1);
    for i = 1:numNodes
        v(i) = summary(obj.pNodes{i});
        v(i) = archOrFuse(obj,v(i),obj.pIsTracker(i));
        ind = find(obj.pNodes{i}.Index == obj.OutputSelection);
        if isempty(ind)
            v(i).ArchitectureOutput = [];
        else
            v(i).ArchitectureOutput = ind;
        end
    end
    val = struct2table(v,'AsArray',true);
else
    val = fusion.trackingArchitecture.internal.TrackingNode.sampleSummary;
end
end

function v = archOrFuse(obj,v,isTracker)
%Checks each input and categorises it as archOrFuse. Returns the corrected
%struct, v.

% Copyright 2020 The MathWorks, Inc.

if isTracker
    v.FuserInputs = 'Not applicable';
    v.ArchitectureInputs = num2str(v.ArchitectureInputs(:)');
else
    numIns = numel(v.ArchitectureInputs);
    isArch = false(1,numIns);
    for i = 1:numIns
        isArch(i) = ~any(v.ArchitectureInputs(i) == obj.pNodeIndices);
    end
    v.ArchitectureInputs = num2str(v.ArchitectureInputs(isArch));
    v.FuserInputs = num2str(v.FuserInputs(~isArch));
end
end