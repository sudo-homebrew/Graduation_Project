function h = plotArc(initPose,r,dS,s0)
%This function is for internal use only. It may be removed in the future.

%plotArc Displays an arc segment
%
%   h = plotArc(INITPOSE, R, DS) Plots an arc whose shape is determined by
%   an initial SE2 configuration, INITPOSE, and a signed radius, R, which
%   dictates the curvature of the arc. The arc will be plotted between 
%   [0, dS]. If dS is negative, the arc segment will extend along the anti-tangent.
%
%   h = plotArc(INITPOSE, R, DS, S0) Plots an arc segment between [s0, s0+dS]

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    if nargin == 3
        s0 = 0;
    end
    ds = linspace(s0,s0+dS,100);
    pts = nav.algs.internal.evaluateArc(initPose,r,ds);
    figure(1)
    h = plot(pts(1,:),pts(2,:));
    axis equal
end
