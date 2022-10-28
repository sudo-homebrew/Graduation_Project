function [G1,G2,m] = stabproj(G)
%STABPROJ State-space stable/anti-stable decomposition.
%
% [SS_1,SS_2,M] = STABPROJ(SS_) produces
%     a decomposition of G(s) as the sum of its m-state stable part G1(s) and
%     its antistable part G2(s) 
%
% Stabproj uses "stabsep.m" from Control Toolbox for better numerical
% property.

% R. Y. Chiang & M. G. Safonov 7/85
% Copyright 1988-2004 The MathWorks, Inc. 
[G1,G2] = stabsep(G);
m = order(G1);
