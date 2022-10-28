function isPCTAvailable = canUsePCT
% isPCTAvailable = fusion.internal.canUsePCT returns true if:
% 1. PCT is installed
% 2. License is available
% 3. A parallel pool is running. It starts a parallel pool if PCT is
% installed and a license is available.

% This is an internal function and may be removed in a future release.

% Copyright 2018-2020 The MathWorks, Inc.

isPCTAvailable = canUseParallelPool();

% If a pool can be started, actually start it. This may throw.
if isPCTAvailable
    pool = gcp();
    isPCTAvailable = ~isempty(pool) && pool.Connected;
end
end