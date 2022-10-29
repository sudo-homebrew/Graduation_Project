function solverOption = poseGraphSolverOptions(optionStr)
%POSEGRAPHSOLVEROPTIONS Create solver options for pose graph optimization
%   SOLVEROPTION = poseGraphSolverOptions() returns the default pose graph
%   solver option (which is the builtin trust region solver) with default
%   settings.
%
%   SOLVEROPTION = poseGraphSolverOptions(OPTIONSTR) returns pose graph
%   solver as specified by OPTIONSTR. OPTIONSTR can be one of the
%   following:
%   - 'builtin-trust-region'
%   - 'g2o-levenberg-marquardt'
%
% See also poseGraph, poseGraph3D, optimizePoseGraph, trimLoopClosures

%   Copyright 2020-2021 The MathWorks, Inc.
 
%#codegen
    narginchk(0,1);
    if nargin == 0
        solverOption = nav.solveroptions.TrustRegion();
        return;
    end
    validOption = validatestring(optionStr, {'builtin-trust-region', 'g2o-levenberg-marquardt'}, 'poseGraphSolverOptions', 'solver option name' );
    switch (validOption)
        case 'builtin-trust-region'
            solverOption = nav.solveroptions.TrustRegion();
        otherwise
            solverOption = nav.solveroptions.G2oLevenbergMarquardt();
    end
end

