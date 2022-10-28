function sortedSolutions = sortByEuclideanDistance(solutions, referenceConfig, isJointRevolute)
%sortByEuclideanDistance Sort a matrix of solution configurations relative to a reference configuration by Euclidean norm
%   This function sorts a matrix of configurations using a pre-defined
%   distance metric. The computed distance between any state and the
%   reference state, referenceConfig, is a Euclidean norm of difference
%   between a revolute joint's values which is then wrapped to [-pi, pi],
%   and a displacement between a prismatic joint's values. 
 
%   Copyright 2020 The MathWorks, Inc.
 
% Compute the distances between each configuration and the reference
dist = robotics.manip.internal.RigidBodyTreeUtils.distance(...,
    referenceConfig, solutions, isJointRevolute);
 
% Sort the outputs
[~, sortedIdx] = sort(dist);
sortedSolutions = solutions(sortedIdx,:);
   
end
