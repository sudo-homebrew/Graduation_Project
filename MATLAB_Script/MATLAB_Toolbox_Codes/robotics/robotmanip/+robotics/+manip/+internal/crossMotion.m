function [ motionVecDot ] = crossMotion( v, motionVec )
%This function is for internal use only. It may be removed in the future.

%CROSSMOTION Spatial cross product for motion vectors. 
%   NOTE that the motionVec is moving with spatial velocity v.

%   Copyright 2016 The MathWorks, Inc.

%#codegen

sc1 = robotics.manip.internal.skew(v(1:3));
sc2 = robotics.manip.internal.skew(v(4:6));
motionVecDot = [sc1, zeros(3); sc2, sc1]*motionVec;

end

