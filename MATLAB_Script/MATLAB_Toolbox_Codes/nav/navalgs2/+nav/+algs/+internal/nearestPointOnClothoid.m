function [sMin, dMin, pathPoint] = nearestPointOnClothoid(queryXY,x0,y0,th0,k0,dk,L,mustBeProjection)
%This class is for internal use only. It may be removed in the future.

%nearestPointOnClothoid Find nearest point and distance between an XY point and clothoid
%
%   Returns the minimum distance between query point, QUERYXY, and the
%   clothoid, defined using base location, [x0 y0], tangent direction
%   relative to x-axis, th0, curvature, k0, constant change in curvature,
%   dk, and arc-length in segment.
%
%   References: 
%   [1] MARCO FREGO, ENRICO BERTOLAZZI, "Point-Clothoid Distance and 
%       Projection Computation", SIAM J. Sci. Comput., Vol. 41 (2019), No. 5, 
%       pp. A3326-A3353
%
% Algorithm A.3

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

    if nargin == 7
        mustBeProjection = false;
    end

    sSign = k0*dk;
    if sSign >= 0
    % If k0,dk are of the same sign, then segment points away from inflection pt
        [sMin, dMin] = dist2ClothNoInflect(queryXY,x0,y0,th0,k0,dk,L,mustBeProjection);
    else
    % Segment points towards inflection
        if L*dk*dk + sSign <= 0
        % Segment does not cross inflection point
            [x1,y1,th1,k1] = clothoid(x0,y0,th0,k0,dk,L);
            [s,dMin] = dist2ClothNoInflect(queryXY,x1,y1,th1+pi,-k1,dk,L,mustBeProjection);
            sMin = L-s;
        else
        % Inflection point contained between [s0, s0+L]
            
            % Find offset of segment base from inflection point
            sBar = -k0/dk;
            % Calculate clothoid at this location
            kBar = 0;
            [xBar,yBar,thBar,~] = clothoid(x0,y0,th0,k0,dk,sBar);
            
            % Find minimum in positive half
            [s0,d0] = dist2ClothNoInflect(queryXY,xBar,yBar,thBar   ,kBar,dk,L-sBar,mustBeProjection);
            
            % Find minimum in negative half
            [s1,d1] = dist2ClothNoInflect(queryXY,xBar,yBar,thBar+pi,kBar,dk,  sBar,mustBeProjection);
            
            % Return the smallest result
            if ~isnan(s0) && (isnan(s1) || d0 <= d1)
                sMin = sBar+s0;
                dMin = d0;
            else
                if ~isnan(s1)
                    sMin = sBar-s1;
                    dMin = d1;
                else
                    sMin = nan;
                    dMin = inf;
                end
            end
        end
    end
    
    % Evaluate point
    [x,y,th,k] = clothoid(x0,y0,th0,k0,dk,sMin);
    pathPoint = [x y th k dk sMin];
end

function [sMinLocal,dMinLocal] = dist2ClothNoInflect(queryXY,x0,y0,th0,k0,dk,S,mustBeProjection)
%dist2ClothNoInflect Returns the minimum distance for half-clothoid segment
%
%   Partitions a clothoid-segment that lies in one half of the parent
%   clothoid into two separate regions and finds the minimum distance to
%   the query point in each section (where applicable).

% Algorithm A.3

    n = 1;
    if abs(k0) >= sqrt(2*pi*n*abs(dk))
    %Quasi-Circular case: Segment exists entirely within the outermost loop
    %of the attractor, so distance function can be approximated as pt/circle dist
        [sMinLocal,dMinLocal] = dist2ClothoidSpiral(queryXY,x0,y0,th0,k0,dk,S,mustBeProjection);
    else
        if abs(k0) + abs(dk)*S <= sqrt(2*pi*abs(dk))
        %Regular Clothoid: Segment is found in the region leading up to the attractor
            [sMinLocal,dMinLocal] = dist2ClothoidNonSpiral(queryXY,x0,y0,th0,k0,dk,S,mustBeProjection);
        else
        %Overlap: Part of segment lies in attraction loop, part lies in general clothoid region
            
            % Find arclength offset where the boundary occurs
            sBreak = sqrt(2*pi/abs(dk)) - abs(k0)/abs(dk);
            
            % Evaluate clothoid at this partition
            [xB,yB,thB,kB] = clothoid(x0,y0,th0,k0,dk,sBreak);
            
            % Find minimum in standard clothoidal region
            [s0,d0] = dist2ClothoidNonSpiral(queryXY,x0,y0,th0,k0,dk,sBreak,mustBeProjection);
            
            % Find minimum in attractor spiral
            [s1,d1] = dist2ClothoidSpiral(queryXY,xB,yB,thB,kB,dk,S-sBreak,mustBeProjection);
            
            % Return smaller of the two results as global minimum
            if d0 < d1
                sMinLocal = s0;
                dMinLocal = d0;
            else
                sMinLocal = sBreak+s1;
                dMinLocal = d1;
            end
        end
    end
end

function [sLocal,dLocal] = dist2ClothoidSpiral(queryXY,x0,y0,th0,k0,dk,S,mustBeProjection)
%dist2ClothoidSpiral Returns minimum distance and arclength along clothoid
%to a query point, where the clothoid segment is entirely inclosed in the
%clothoid-attractor's monotonically decreasing limit. This method initially
%refines the S-interval so that the difference in tangent angle between 
%[s0,s1] is < 2*pi. This guarantees at most two local minima, the smaller
%of which is eventually returned.

% Algorithm A.4
    
    % Find span of the tangent angle
    dTh = abs(dk*S^2/2+k0*S);
    
    if dTh <= 2*pi
    %Appropriately sized segment, return minimum
        [sLocal,dLocal] = dist2ClothoidArc(queryXY,x0,y0,th0,k0,dk,S,mustBeProjection);
    else
    %Further refinement required
        % Calculate osculating circle center at loop start
        xC = x0 - sin(th0)/k0;
        yC = y0 + cos(th0)/k0;
        
        if 1/(k0^2) <= ((queryXY(1)-xC)^2 + (queryXY(2)-yC)^2)
        %If query point lies outside the circle containing the current interval
            dS = aPlus(2*pi,k0,dk);
            [sLocal,dLocal] = dist2ClothoidArc(queryXY,x0,y0,th0,k0,dk,dS,mustBeProjection);
        else
        %Query point lies inside the current loop
            % Retrieve end-point for current loop
            [x1,y1,th1,k1] = clothoid(x0,y0,th0,k0,dk,S);
            
            % Calculate osculating circle center at loop end
            xC = x1 - sin(th1)/k1;
            yC = y1 + cos(th1)/k1;
            
            if 1/(k1^2) >= ((queryXY(1)-xC)^2 + (queryXY(2)-yC)^2)
            % Point lies outside of end-pt circle -> minimum lies in 2*pi
            % interval prior to s1
                dS = aPlus(2*pi,-k1,dk);
                [s0,dLocal] = dist2ClothoidArc(queryXY,x1,y1,th1+pi,-k1,dk,dS,mustBeProjection);
                sLocal = S-s0;
            else
            % Point lies in between the current arc. Bisect current arc, 
            % find min on each segment and return smaller of the two.
                dS = aPlus(dTh/2,k0,dk);
                [s0,d0] = dist2ClothoidSpiral(queryXY,x0,y0,th0,k0,dk,dS,mustBeProjection);
                [x1,y1,th1,k1] = clothoid(x0,y0,th0,k0,dk,dS);
                [s1,d1] = dist2ClothoidSpiral(queryXY,x1,y1,th1,k1,dk,S-dS,mustBeProjection);
                if d0 < d1
                    sLocal = s0;
                    dLocal = d0;
                else
                    sLocal = dS+s1;
                    dLocal = d1;
                end
            end
        end
    end
end

function sNew = aPlus(S,k0,dk)
%aPlus Compute length of arc corresponding to change in tangent = 2*pi from the current point along curve
%
% Algorithm A.4
    
    t = sign(k0)*dk;
    sNew = 2*S/(abs(k0)+sqrt(2*S*t+k0^2));
end

function [sMin,dMin] = dist2ClothoidArc(queryXY,x0,y0,th0,k0,dk,S,mustBeProjection)
%dist2ClothoidArc Finds the arclength for a circular subsection of the
%clothoid where distance to queryXY is minimum
%
% Algorithm A.5
    
    % Define failure-case outputs
    sMin = nan;
    dMin = inf;

    % Calculate clothoid at end of interval
    [x1,y1,th1,~] = clothoid(x0,y0,th0,k0,dk,S);
    s0 = 0;
    s1 = S;
    % Calculate tangent angle of vector between query point and arc
    phi0 = th0-atan2(y0-queryXY(2),x0-queryXY(1));
    phi1 = th1-atan2(y1-queryXY(2),x1-queryXY(1));
    
    % Check direction to move
    % NOTE: Trig evaluation is required on rhs due to floating point error in cos(pi/2)
    f0 = cos(phi0) <= cos(pi/2); 
    f1 = cos(phi1) >= cos(-pi/2);
    
    if f0
    % Take step towards minimum
        [s0, f0] = dist2ArcIter(queryXY,x0,y0,th0,k0,dk,S,0,false);
    end
    if f1
    % Take step towards minimum
        [s1, f1] = dist2ArcIter(queryXY,x0,y0,th0,k0,dk,S,S,false);
    end

    if ~f0 && ~f1
        % Search from center of segment
        [sMin, fm] = dist2ArcIter(queryXY,x0,y0,th0,k0,dk,S,S/2,false);
        [x,y,th,k] = clothoid(x0,y0,th0,k0,dk,S);
        
        % Calculate distance if search resulted in a minima
        if fm && (~mustBeProjection || isValidProjection(queryXY,x,y,th,k))
            % Calculate distance
            dMin = sqrt((queryXY(1)-x)^2+(queryXY(2)-y)^2);
        else
            dMin = inf;
            sMin = nan;
        end
    end

    %Evaluate the normalized clothoid at minima
    [x,y,~,~] = clothoid(x0,y0,th0,k0,dk,s0);
    d0 = sqrt((x-queryXY(1))^2+(y-queryXY(2))^2);
    [x,y,~,~] = clothoid(x0,y0,th0,k0,dk,s1);
    d1 = sqrt((x-queryXY(1))^2+(y-queryXY(2))^2);

    if mustBeProjection
        % Check whether any of the results are valid projections
        if f0 && d0 < dMin
            dMin = d0;
            sMin = s0;
        end
        if f1 && d1 < dMin
            dMin = d1;
            sMin = s1;
        end
        
        if isinf(dMin)
            % If no valid projection was found from search nearest search,
            % search for maximum.
            [sMin,fm] = dist2ArcIter(queryXY,x0,y0,th0,k0,dk,S,S/2,mustBeProjection);
            [x,y,th,k] = clothoid(x0,y0,th0,k0,dk,sMin);
            if fm && isValidProjection(queryXY,x,y,th,k)
                dMin = sqrt((queryXY(1)-x)^2+(queryXY(2)-y)^2);
            else
                sMin = nan;
            end
        end
    else
        % Select best result
        if d0 < dMin
            dMin = d0;
            sMin = s0;dS = inf; t = inf;
        end
        if d1 < dMin
            dMin = d1;
            sMin = s1;
        end
    end
end

function [sMin,minFound] = dist2ArcIter(queryXY,x0,y0,th0,k0,dk,S,sGuess,mustBeProjection)
%dist2ArcIter Minimizes the current distance each step
%
%   Minimizes the derivative of the squared distance between a point and arc,
%   f(s) = R*sin(k0*s+w), where:
%       R    = sqrt(a0^2+b0^2+k0^(-2)+2*a0*k0^(-1))
%       w    = atan2(b0, a0+k0^(-1);
%      |a0|  = |c(th0) -s(th0)| * |y0-qy|
%      |b0|  = |s(th0)  c(th0)| * |x0-qx|
%
% Algorithm A.5

    tol = 1e-10;
    sMin = sGuess;
    s = sGuess;
    iterLimit = 20;
    i = 0;
    minFound = false;
    stepWeight = 2;
    prevStep = nan;
    dS = inf; t = inf;

    while i < iterLimit
        % Evaluate clothoid at given arclength
        [x,y,th,k] = clothoid(x0,y0,th0,k0,dk,s);
        
        % Calculate distance function parameters
        if mustBeProjection
            % Calculate a center of curvature
            pC = arcCenter(x,y,th,k);

            % Calculate vector between query point and center
            v = pC(:)'-queryXY(:)';
            
            % Find distance
            dCtr = norm(v);
            
            if dCtr < tol
                sMin = s;
                minFound = true;
                return
            end
            
            % Find projection length
            projDist = v*[cos(th);sin(th)];
            
            % Find dot product
            dotP = projDist/dCtr;
            
            % Multiply by radius and scale to pi/2 to get rough approx of dS
            dS = dotP/abs(k);
        else
            a0 = (y-queryXY(2))*cos(th) - (x-queryXY(1))*sin(th); % Projection dist
            b0 = (y-queryXY(2))*sin(th) + (x-queryXY(1))*cos(th); % Projection length
            t  = a0*k;
            
            % Calculate next step
            if 1 + 2*t > 0
            % 1+2*t > 0 is sufficient to satisfy f'(s) > 0, which indicates
            % that we are facing uphill
                % Calculate next step size
                t  = b0/(1+t);
                dS = -t*Atanc(t*k); % Atanc needed for numeric stability if k->0
            else
            % Otherwise we are facing downhill
                % Calculate step based on current curvature
                w = atan2(b0, a0+1/k);
                if k<0
                    if w < 0
                        w = w+pi;
                    else
                        w = w-pi;
                    end
                end
                dS = -w/k;
            end
        end

        if sign(prevStep)~=sign(dS)
            % Reduce step size if we overshot the min. This will
            % artificially reduce the step-size once we are near the target
            % leading to more likely convergence while maintaining
            % accuracy.
            stepWeight = max(stepWeight*.5,0.1);
        else
            stepWeight = min(stepWeight/0.75,1);
        end
        dS = dS*stepWeight;
        prevStep = dS;

        % Check for exit
        if abs(dS) < tol
            if s >= 0 && s <= S
                % Minimum has been found, and is within bounds
                minFound = true;
                sMin = s;
                return;
            end
            s = s+dS;
            if s >= 0 && s <= S
                % Minimum has been found, and is within bounds
                minFound = true;
                sMin = s;
            end
            return;
        else
            s = s+dS;
        end
        i = i+1;
    end

    % Evaluate clothoid at given arclength
    [x,y,~,k] = clothoid(x0,y0,th0,k0,dk,s);

    % Did not return. Check whether the proximity between query-point
    % and curve is leading to a step-size degeneration.
    if abs(t*k) < tol || max(abs([x y] - queryXY(:)')) < 1e-5
        % If step-size is degenerating, check whether the change in
        % angle is sufficiently small.
        if abs(dS*k) < tol
            sMin = s;
            minFound = true;
        end
    end
end

function [sMin,dMin] = dist2ClothoidNonSpiral(queryXY,x0,y0,th0,k0,dk,S,mustBeProjection)
%dist2ClothoidNonSpiral Returns minimum distance and intervals outside of attractor
%
%   Finds nearest point on clothoid for intervals between the attractor 
%   spirals. This function first transforms the clothoid to its standard
%   position (less costly formulae). It then splits the curve into two
%   sections, separated by the y-axis of the transformed reference frame.
%   Lastly, it finds the nearest point in each section (where applicable),
%   and returns the smaller of the two.
%
%     To convert current clothoid to standard position:
%       1) remap features to the inflection point (sBar)
%       2) scale dimensions to unit (scale)
%       3) apply inverse rotation (thBar)
%       4) reflect about x-axis if clothoid is inverted
%
% Algorithm A.6
    
    % Calculate clothoid at the inflection point
    kBar = 0;
    scale = sqrt(abs(dk)/pi);
    if k0 == 0
        % Input is already the inflection point
        sBar  = 0;
        xBar  = x0;
        yBar  = y0;
        thBar = th0;
        a = 0;
        b = S*scale;
    else
        sBar  = -k0/dk;
        a = -sBar*scale;
        b = (S-sBar)*scale;
        [xBar,yBar,thBar,~] = clothoid(x0,y0,th0,k0,dk,sBar);
    end
    
    % Resize, reflect, and rotate
    M = [1 0; 0 sign(dk)]; % Inversion matrix
    Rinv = [cos(thBar) sin(thBar); -sin(thBar) cos(thBar)]; % Rotation matrix
    qBar = scale*M*Rinv*(queryXY(:)-[xBar;yBar]);
    
    % Evaluate normalized clothoid at "a"
    [x,y,~,~] = clothoid(xBar,yBar,thBar,kBar,pi,a);
    
    if b^2-a^2 <= 4
    % Variation of tangent angle over interval is <= 2*pi
        [s,d] = dist2ClothoidStandard(qBar, a, b, mustBeProjection);
    else
        % Compute distance to attractor, +/-(0.5; 0.5)
        dInf = sqrt((qBar'-.5)*(qBar-.5));
        
        % Compute distance between interval base and attractor
        dSegPt = sqrt((x-.5)^2+(y-.5)^2);
        
        if dInf >= dSegPt
            % Minimum will be found on first part of the curve
            [s,d] = dist2ClothoidStandard(qBar, a, a+4/(a+sqrt(4+a^2)),mustBeProjection);
        else
            % Evaluate normalized clothoid at "b"
            [x,y,~,~] = clothoid(xBar,yBar,thBar,kBar,pi,b);
            dSegPt = sqrt((x-.5)^2+(y-.5)^2);
            
            if dInf <= dSegPt
            % Minimum lies in second part of the curve
                [s,d] = dist2ClothoidStandard(qBar, b-4/(b+sqrt(b^2-4)), b,mustBeProjection);
            else
            % The global minimum lies somewhere inside the interval. Since 
            % the squared distance function contains multiple local max/min
            % over the interval, we first find critical points of the
            % squared distance function by finding S that correspond to f'=0.
            % The Halley method is used to find either the nearest max or
            % min. Since critical points of the sqr-dist function alternative
            % between max/min for each pair zeros, higher order derivatives
            % are used to determine which sub-intervals should be searched.
               	
                % Set limit solver tolerance and iteration limit
                iterLimit = 10;
                tol = 1e-10;
                
                % Set initial guess to beginning of first section
                s = a;
                i = 0;
                while i < iterLimit
                    % Evaluate normalized clothoid deriv at s
                    [x,y,~,~] = clothoid(xBar,yBar,thBar,kBar,pi,s);
                    k  = pi*s;
                    th = (pi/2)*s^2;
                    
                    % Evaluate squared dist to attractor
                    px = x-.5;
                    py = y-.5;
                    p = sqrt(px^2+py^2);
                    sci = th-atan2(py,px);
                    
                    % Evaluate 0th,1st,2nd derivatives of pt-to-clothoid squared dist
                    f   = p-dInf;
                    df  = cos(sci);
                    ddf = sin(sci)*(k-sin(sci/p));
                    
                    % Calculate Halley step, then update or exit
                    dS = (f*df)/(df^2-f*ddf/2);
                    s = s - dS;
                    if abs(dS) < tol
                        break;
                    end
                    i = i+1;
                end
                
                % Calculate interval boundaries that will contain a minimum
                % rather than maximum
                LNeg = min(s-a, 4/(s+sqrt(s^2-4)));
                LPos = min(b-s, 4/(s+sqrt(s^2+4)));
                
                % Find minimum in both sub intervals
                [sPos,dPos] = dist2ClothoidStandard(qBar,s,s+LPos,mustBeProjection);
                [sNeg,dNeg] = dist2ClothoidStandard(qBar,s-LNeg,s,mustBeProjection);
                
                % Choose smaller of the two
                if dPos <= dNeg
                    s = sPos;
                    d = dPos;
                else
                    s = sNeg;
                    d = dNeg;
                end
            end
        end
        
    end
    % Return results to global scale
    sMin = sBar+s/scale;
    dMin = d/scale;
end

function [sMin,dMin] = dist2ClothoidStandard(queryXY,a,b,mustBeProjection)
%dist2ClothoidStandard Returns the minimum distance and corresponding
%arclength between a point and clothoid on the arclength interval [a,b]. 
%Both the point and clothoid are assumed to be in "standard position".
%
% Algorithm A.7

    % Preallocate output
    sMin = nan;
    dMin = inf;

    % Get position and tangent angle of interval bounds
    [xA,yA,~,~] = clothoid(0,0,0,0,pi,a);
    psiA = (pi/2)*a^2 - atan2(yA-queryXY(2),xA-queryXY(1));
    [xB,yB,~,~] = clothoid(0,0,0,0,pi,b);
    psiB = (pi/2)*b^2 - atan2(yB-queryXY(2),xB-queryXY(1));
    
    % Get initial guess and search direction
    % NOTE: Trig evaluation is required on rhs due to floating point error in cos(pi/2)
    f0 = cos(psiA) < cos(pi/2);
    f1 = cos(psiB) > cos(-pi/2);
    
    s0 = a;
    s1 = b;

    % If the gradient points inward at boundary, search for minimum inside interval
    if f0
        [s0,f0] = dist2StandardIter(queryXY,a,b,a,false);
    end
    if f1
        [s1,f1] = dist2StandardIter(queryXY,a,b,b,false);
    end

    if ~f0 && ~f1
        % Search from center of segment
        [sMin, fm] = dist2StandardIter(queryXY,a,b,(s0+s1)/2,false);
        [x,y,th,k] = clothoid(0,0,0,0,pi,sMin);
        
        % Calculate distance if search resulted in a minima
        if fm && (~mustBeProjection || isValidProjection(queryXY,x,y,th,k))
            % Calculate distance
            dMin = sqrt((queryXY(1)-x)^2+(queryXY(2)-y)^2);
        else
            dMin = inf;
            sMin = nan;
        end
    end

    %Evaluate the normalized clothoid at minima
    [x,y,~,~] = clothoid(0,0,0,0,pi,s0);
    d0 = sqrt((x-queryXY(1))^2+(y-queryXY(2))^2);
    [x,y,~,~] = clothoid(0,0,0,0,pi,s1);
    d1 = sqrt((x-queryXY(1))^2+(y-queryXY(2))^2);

    if mustBeProjection
        % Check whether any of the results are valid projections
        if f0 && d0 < dMin
            dMin = d0;
            sMin = s0;
        end
        if f1 && d1 < dMin
            dMin = d1;
            sMin = s1;
        end
        
        if isinf(dMin)
            % If no valid projection was found from search nearest search,
            % search for maximum.
            [sMin,fm] = dist2StandardIter(queryXY,a,b,(s0+s1)/2,mustBeProjection);
            [x,y,th,k] = clothoid(0,0,0,0,pi,sMin);
            if fm && isValidProjection(queryXY,x,y,th,k)
                dMin = sqrt((queryXY(1)-x)^2+(queryXY(2)-y)^2);
            else
                sMin = nan;
            end
        end
    else
        % Select best result
        if d0 < dMin
            dMin = d0;
            sMin = s0;
        end
        if d1 < dMin
            dMin = d1;
            sMin = s1;
        end
    end
end

function [sMin, minFound] = dist2StandardIter(queryXY,a,b,sGuess,mustBeProjection)
%dist2StandardIter Iteratively minimizes distance between point and clothoid
%
% Algorithm A.7
    
    sMin = sGuess;
    minFound = false;
    s = sGuess;
    tol = 1e-10;
    iterLimit = 20;
    i = 0;
    dS = inf; t = inf;
    
    boundHit = 0;
    while i < iterLimit
        % Calculate clothoid at current arclength
        [x,y,th,k] = clothoid(0,0,0,0,pi,s);
        
        % Calculate pt-to-clothoid distance parameters
        if mustBeProjection
            pC = arcCenter(x,y,th,k); 
            v = pC(1:2)-queryXY(1:2)';
            v = v./norm(v);
            a0 = -(v(2)*cos(th) - v(1)*sin(th));
            b0 = -(v(2)*sin(th) + v(1)*cos(th));
        else
            a0 = (y-queryXY(2))*cos(th) - (x-queryXY(1))*sin(th); % Projection dist
            b0 = (y-queryXY(2))*sin(th) + (x-queryXY(1))*cos(th); % Projection length
        end
        
        % Normalize the normal vector using radius of curvature
        t = a0*k;
            
        if 1+2*t > 0
            t  = b0/(1+t);
            dS = -t*Atanc(t*k);
        else
            w = atan2(b0,a0+1/k);
            if k < 0
                if w < 0
                    w = w+pi;
                else
                    w = w-pi;
                end
            end
            dS = -w/k;
        end
        
        s = s+dS;
        if dS > 0
            if s > b 
                if boundHit == 1
                    % Attempted to go out of bounds twice
                    return;
                end
                s = min(s,b);
                boundHit = 1; 
            else
                boundHit = 0;
            end
        elseif dS < 0
            if s < a
                if boundHit == -1
                    % Attempted to go out of bounds twice
                    return;
                end
                s = max(s,a);
                boundHit = -1;
            else
                boundHit = 0;
            end
        end
        if abs(dS) < tol && boundHit == 0
            if s >= a && s <= b
                sMin = s;
                minFound = true;
            end
            return;
        end
        i = i+1;
    end
    
    % Evaluate clothoid at given arclength
    [x,y,~,k] = clothoid(0,0,0,0,pi,s);

    % Did not return. Check whether the proximity between query-point
    % and curve is leading to a step-size degeneration.
    if abs(t*k) < tol || max(abs([x y] - queryXY(:)')) < 1e-5
        % If step-size is degenerating, check whether the change in
        % angle is sufficiently small.
        if abs(dS*k) < tol
            sMin = s;
            minFound = true;
        end
    end
end

function [x1,y1,th1,k1] = clothoid(x0,y0,th0,k0,dk,L)
%clothoid Construct terminal points for clothoid
%
% Algorithm A.1

    if L == 0
        x1 = x0; y1 = y0; th1 = th0; k1 = k0;
    else
        if abs(dk) < 1e-10
            [x1,y1,th1,k1] = circle(x0,y0,th0,k0,dk,L);
        else
            k1  = dk*L+k0;
            th1 = dk/2*L^2 + k0*L + th0;
            % Calc integrated xy
            xy = matlabshared.tracking.internal.scenario.fresnelg(L, dk, k0, th0);
            % Add to base location
            x1 = x0 + real(xy);
            y1 = y0 + imag(xy);
        end
    end
end

function [x1,y1,th1,k1] = circle(x0,y0,th0,k0,dk,L)
    % Use circle/line approximation
    dTheta = k0*L;
    cT = cos(th0); sT = sin(th0);
    R = [cT -sT; sT cT];
    th1 = th0+dTheta;
    k1 = k0+dk*L;
    % Curvature is small, use Cosc/Sinc functions to evaluate
    % curve. These methods are capable of accurately evaluating
    % curves that border between arcs and lines.
    p = L*R*[Sinc(dTheta); Cosc(dTheta)];
    x1 = x0+p(1);
    y1 = y0+p(2);
end

function val = Cosc(x)
%Cosc Approximates (1-cos(x))/x for x close to 0
%
% Algorithm A.1
    
    if abs(x) < 0.002
        val = (x/2)*(1+((x^2)/12)*(1-(x^2)/30));
    else
        val = (1-cos(x))/x;
    end
end

function xyCtr = arcCenter(x,y,th,k)
%arcCenter Return point lying at radius of curvature for an SE2 pose and curvature

    %#codegen
    vPerp = [-sin(th) cos(th)];
    xyCtr = [x y]+vPerp./k;
end

function val = Sinc(x)
%Sinc Approximates sin(x)/x for x close to 0
%
% Algorithm A.1
    
    if abs(x) < 0.002
        val = 1-((x^2)/6)*(1-(x^2)/20);
    else
        val = sin(x)/x;
    end
end

function val = Atanc(x)
%Atanc Approximates atan(x)/x for x close to 0
%
% Algorithm A.2
    
    if abs(x) < 0.002
        val = 1-x^2*(1/3-(x^2)/5);
    else
        val = atan(x)/x;
    end
end

function isValid = isValidProjection(queryXY,x,y,th,k)
% Check whether point is a valid projection
    if abs(k) > sqrt(eps)
        % Find center point
        p = arcCenter(x,y,th,k);
    else
        % Use path point when center of curvature becomes excessively large
        p = [x y];
    end
    
    % Check whether query point lies along radius of curvature
    v1 = p-queryXY(:)';
    v2 = [cos(th); sin(th)];
    dotP = v1/norm(v1)*v2;
    isValid = abs(dotP) <= sqrt(eps);
end