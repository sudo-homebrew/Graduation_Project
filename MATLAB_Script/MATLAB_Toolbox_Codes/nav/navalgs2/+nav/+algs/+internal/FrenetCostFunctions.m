classdef FrenetCostFunctions < handle
%This class is for internal use only. It may be removed in the future.

%FrenetCostFunctions Cost functions evaluated on frenet trajectories
%
%   FrenetCostFunctions contains methods used for computing total cost
%   of a quintic frenet trajectory based on arc length, lateral and
%   longitudinal smoothness deviation from the reference path and time
%   taken to reach the terminal state.
%
%   FrenetCostFunctions methods:
%       evaluateTotalCost               - Compute the total cost of a
%                                         given pair of frenet
%                                         trajectories
%
%       jerkCost                        - Compute integral of jerk
%                                         squared using trajectory
%                                         defined in Frenet frame
%
%   References:
%
%   [1] M. Werling, J. Ziegler, S. Kammel, and S. Thrun. "Optimal
%       trajectory generation for dynamic street scenarios in a frenet
%       frame." In 2010 IEEE International Conference on Robotics and
%       Automation, 2010, pp. 987-993.
%
%   Copyright 2019 The MathWorks, Inc.

%#codegen
    methods(Static)
        function totalCost = evaluateTotalCost(frenetTrajectory, deviationOffset, weights)
        %evaluateTotalCost Compute total cost of trajectory
        %   Evaluate the cost of the trajectory described in frenet states.
        %   DeviationOffset is the offset added to the deviation computed
        %   from the reference path. Weights is a structure containing scalars
        %   for the cost multipliers of the corresponding trajectory attributes.

        % Validate number of input arguments
            narginchk(3,3)

            [lateralJerkCost, longitudinalJerkCost] = nav.algs.internal.FrenetCostFunctions.jerkCost(frenetTrajectory);

            totalCost = lateralJerkCost * weights.LateralSmoothness + ...
                longitudinalJerkCost * weights.LongitudinalSmoothness + ...
                (frenetTrajectory(end,4) - deviationOffset)^2 * weights.Deviation + ...
                frenetTrajectory(end,7) * weights.Time + ...
                frenetTrajectory(end,1) * weights.ArcLength;
        end

        function [lateralJerkCost, longitudinalJerkCost] = jerkCost(frenetTrajectory)

            time = frenetTrajectory(:,7);

            timeInterval = diff(time(1:2));
            lPrime = frenetTrajectory(:,5);
            lPrimePrime = frenetTrajectory(:,6);


            sDot = frenetTrajectory(:,2);
            sDotDot = frenetTrajectory(:,3);
            longitudinalJerk = gradient(sDotDot,time);

            % Compute lateral acceleration
            lDotDot = lPrimePrime .* (sDot.^2) + lPrime.* sDotDot;
            lateralJerk = gradient(lDotDot,time);

            % Jerk cost is given by integral of squared jerk
            % Compute lateral jerk cost
            lateralJerkCost =  sum(lateralJerk .^2) * timeInterval;

            % Compute longitudinal jerk cost
            longitudinalJerkCost = sum(longitudinalJerk.^2) * timeInterval;
        end
    end
end
