function [ forceVecDot ] = crossForce( v, forceVec )
%This function is for internal use only. It may be removed in the future.

%CROSSMOTION Spatial cross product for force vectors. 
%   NOTE that the forceVec is moving with spatial velocity v.

%   Copyright 2016 The MathWorks, Inc.

%#codegen

sc1 = robotics.manip.internal.skew(v(1:3));
sc2 = robotics.manip.internal.skew(v(4:6));
forceVecDot = [sc1, sc2; zeros(3), sc1]*forceVec;

end

