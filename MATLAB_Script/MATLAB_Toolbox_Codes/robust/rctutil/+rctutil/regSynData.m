function [D11r,C1r,D12r,B1r,D21r] = regSynData(D11,C1,D12,B1,D21,dC1,dD12,dB1,dD21)
% Builds regularized plant matrices using calculated perturbations.

%   Copyright 2018 The MathWorks, Inc.
B1r = [B1 dB1];
C1r = [C1 ; dC1];
D12r = [D12 ; dD12];
D21r = [D21 dD21];
D11r = blkdiag(D11,zeros(size(dC1,1),size(dB1,2)));