function [nacp,nacv, gva] = uncertainty2nac(epu, evu, altsigma)
%uncertainty2nac Converts 95% uncertainty bounds to ADSB Categories
% Inputs:    
%         epu      - 95% circular bound on horizontal position  m
%         evu      - 95 % circular bound on horizontal velocity m/s
%         altsigma - one standard-deviation in vertical position m
% Outputs:
%         nacp     - Navigation Accuracy Category Position
%         nacv     - Navigation Accuracy Category Velocity
%         gva      - Geometric Vertical Accuracy Category
%
% This is an internal function and may be removed or modified in a future
% release.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

% NACp see Table C-13 of ICAO Document

NACpTable = [inf 18.52e3 7.408e3 3.704e3 1852 926 555.6 185.2 92.6 30 10 3];
NACvTable = [inf 10 3 1 0.3];
GVATable = [inf 150 45];

nacp = find(NACpTable > epu, 1,'last') - 1;
nacv = find(NACvTable > evu, 1, 'last') - 1;
gva = find(GVATable > 2*altsigma, 1, 'last') - 1;
end
