function [posECEF, gnssVelECEF, dopMatrix] = computeLocation(p, pdot, ...
    satPos, satVel, initPosECEF, initVelECEF)
%COMPUTELOCATION Compute receiver position from GNSS measurements
%
%   Inputs
%       p              - Pseudoranges. S-element column vector. S is the 
%                        number of satellites.
%       pdot           - Pseudorange rates. S-element column vector. S is 
%                        the number of satellites.
%       satPos         - Satellite positions in ECEF (m). S-by-3 matrix. S 
%                        is the number of satellites.
%       satVel         - Satellite velocities in ECEF (m/s). S-by-3 matrix.
%                        S is the number of satellites.
%       initPosECEF    - Initial position estimate in ECEF (m). 3-element 
%                        row vector.
%       intVelECEF     - Initial velocity estimate in ECEF (m/s). 3-element
%                        row vector.
%   Outputs
%       posECEF        - Position estimate in ECEF (m). 3-element row 
%                        vector.
%       gnssVelECEF    - Velocity estimate in ECEF (m/s). 3-element row 
%                        vector.
%       dopMatrix      - 4x4 dilution of precision in ECEF. Diagonals are
%                        (x, y, z, c*tau) uncertainties (m^2).
%
%   References:
%   [1] Groves, Paul. (2013). Principles of GNSS, Inertial, and Multisensor
%       Integrated Navigation Systems, Second Edition. 
%
%   This function is for internal use only. It may be removed in the
%   future.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

if (numel(p) < 4)
    posECEF = NaN(1, 3);
    gnssVelECEF = NaN(1, 3);
    dopMatrix = NaN(4,4);
    return;
end

posPrev = [initPosECEF(:); 0];
velPrev = [initVelECEF(:); 0];

posEst = posPrev;
velEst = velPrev;

Hpos = ones(size(satPos, 1), 4);
Hvel = ones(size(satVel, 1), 4);
HposPrev = Hpos; % Used for DOP calculation.

resPos = Inf;
minResPos = 1e-4;
resVel = Inf;
minResVel = 1e-4;
maxIterations = 200;
iter = 0;
allResPos = NaN(maxIterations, 1, 'like', p);
allResVel = NaN(maxIterations, 1, 'like', p);
% Check if residuals are increasing, if so, save previous estimate that
% corresponds to smaller residual.
checkConverge = @(x) issorted(x, 'descend', 'MissingPlacement', 'last');
while (resPos > minResPos) && (iter < maxIterations)
    % Obtain current true range estimate and line-of-sight vector.
    [pEst, ~, losVector] ...
        = nav.internal.gnss.calculatePseudoranges(satPos, satVel, ...
        posPrev(1:3).', velPrev(1:3).');
    % Add previous clock bias error (m). This is the time offset of the 
    % receiver clock times the speed of light.
    pPred = pEst + posPrev(end);
    
    Hpos(:,1:3) = -losVector;
    
    posEst = posPrev + Hpos.' * Hpos \ Hpos.' * (p - pPred);

    resPos = norm(posEst - posPrev);
    
    iter = iter + 1;
    allResPos(iter) = resPos;
    if ~checkConverge(allResPos)
        posEst = posPrev;
        Hpos = HposPrev;
        break;
    end
    posPrev = posEst;
    HposPrev = Hpos;
end

iter = 0;
while (resVel > minResVel) && (iter < maxIterations) && checkConverge(allResVel)
    % Obtain current true range rate estimate and line-of-sight vector.
    [~, pdotEst, losVector] ...
        = nav.internal.gnss.calculatePseudoranges(satPos, satVel, ...
        posEst(1:3).', velPrev(1:3).');
    % Add previous clock drift error (m/s). This is the time drift of the 
    % receiver clock times the speed of light.
    pdotPred = pdotEst + velPrev(end);
    
    Hvel(:,1:3) = -losVector;
    
    velEst = velPrev + Hvel.' * Hvel \ Hvel.' * (pdot - pdotPred);
    
    resVel = norm(velEst - velPrev);
    
    iter = iter + 1;
    allResVel(iter) = resVel;
    if ~checkConverge(allResVel)
        velEst = velPrev;
        break;
    end
    velPrev = velEst;
end
posECEF = posEst(1:3).';

gnssVelECEF = velEst(1:3).';

dopMatrix = inv(Hpos.' * Hpos);
end
