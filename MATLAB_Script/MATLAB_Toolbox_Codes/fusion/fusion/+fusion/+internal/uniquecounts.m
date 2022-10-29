function [count, C, ia, ic] = uniquecounts(A, varargin)
    % This is an internal function and may be removed or modified in a
    % future release.

    % Copyright 2021 The MathWorks, Inc.

    % This function calculates the count of unique elements in A. 
    % count = fusion.internal.uniquecounts(A) returns the number of times
    % unique elements appear in the array A. 
    %
    % [count, C, ia, ic] = fusion.internal.uniquecounts(A) also outputs
    % unique elements, C and indices ia, and ic. These are the same outputs
    % that you could from unique function. 
    % 
    % [count, C, ia, ic] = fusion.internal.uniquecounts(A,varargin) allows
    % sending extra input arguments to unique function. 

    %#codegen

    [C, ia, ic] = unique(A,varargin{:});
    ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();
    count = repmat(ONE,numel(C),1);
    for i = 1:numel(C)
        count(i) = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntLogicalSum(A == C(i));
    end
end
