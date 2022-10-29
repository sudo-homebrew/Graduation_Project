function INFmissing = findMissingIntervals(UBcert)
%

%   Copyright 1986-2020 The MathWorks, Inc.
idx = isinf([UBcert.gUB]);
INFmissing = reshape([UBcert(idx).Interval],[2 numel(find(idx))])';
% idx = [UBcert.ptUB]==0;
% PTUBmissing = reshape([UBcert(idx).Interval],[2 numel(find(idx))])';
