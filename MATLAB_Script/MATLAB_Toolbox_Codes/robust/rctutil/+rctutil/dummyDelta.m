function Delta = dummyDelta(blk)
% Use Nominal value of 0 (ie., nominal for normalized pert)

%   Copyright 1986-2020 The MathWorks, Inc.
idx = find(blk(:,1)<0 | blk(:,2)==0);
blk(idx,:) = abs(blk(idx,[1 1]));
Delta = zeros(sum(blk,1));
