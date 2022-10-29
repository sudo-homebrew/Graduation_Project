function GPASS = hinfRepairThreshold(GMAX,RelTol,AbsTol)
% Threshold for triggering repair.

%   Copyright 2021 The MathWorks, Inc.
RepairTol = max(sqrt(RelTol),2*RelTol); % relaxed relative tolerance
GPASS = (1+RepairTol) * GMAX + AbsTol;