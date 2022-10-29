function smoothVals = smoothness(points, distanceFcn)
%This function is for internal use only. It may be removed in the future.

%SMOOTHNESS Evaluate smoothness for given set of wayposes/waypoints.
%
%   SMOOTHVALS = smoothness(POINTS, DISTANCEFCN) evaluates the smoothness
%   of each 3 consecutive points POINTS using distance function handle 
%   DISTANCEFCN. This returns 1x(N-2) vector (double) where N is the number
%   of POINTS (wayposes/waypoints). The closer the value is to 0, the
%   smoother the path. Detailed formula follows. The idea is to look at the
%   triangles formed by consecutive path segments and compute the angle 
%   between those segments using Pythagora's theorem. Then, the outside 
%   angle for the computed angle is normalized by the path segments and 
%   contributes to the path smoothness. For a straight line path, the 
%   smoothness will be 0.

% Copyright 2019-2020 The MathWorks, Inc.
%
%   References:
%
%   [1] http://ompl.kavrakilab.org/classompl_1_1geometric_1_1PathGeometric.html.

%#codegen

%Validate points attributes
validateattributes(points, {'numeric'}, {'nonempty'}, 'nav.algs.internal.smoothness', 'points');

%Validate function handle
validateattributes(distanceFcn, {'function_handle'}, {}, 'nav.algs.internal.smoothness', 'distanceFcn');

% Smoothness
smoothVals = zeros(1, size(points,1)-2);

if size(points,1) > 2
    a = distanceFcn(points(1,:), points(2,:));
    for i = 3:size(points,1)
        % view the path as a sequence of segments, and look at the triangles it forms:
        %
        % use Pythagoras generalized theorem to find the cos of the angle between segments a and b
        b = distanceFcn(points(i-1,:), points(i,:));
        c = distanceFcn(points(i-2,:), points(i,:));
        
        acosValue = (a * a + b * b - c * c) / (2.0 * a * b);
        
        if (acosValue > -1.0 && acosValue < 1.0)
            % the smoothness is actually the outside angle of the one we compute
            angle = pi - acos(acosValue);
            
            % and we normalize by the length of the segments
            k = 2.0 * angle / (a + b);
            smoothVals(i-2) = k * k;
        end
        a = b;
    end
end
end