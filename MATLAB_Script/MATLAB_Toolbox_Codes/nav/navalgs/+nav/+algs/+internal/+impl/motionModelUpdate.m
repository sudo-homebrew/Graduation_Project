function Xn = motionModelUpdate(Xn_1, oldOdometry, newOdometry, alpha)
%This function is for internal use only. It may be removed in the future.

%motionModelUpdate Implementation of sampling based odometry motion model
%
%   XN = MOTIONMODELUPDATE(XN_1, OLDODOMETRY, NEWODOMETRY, ALPHA)
%   returns the current poses XN, given the previous sample poses XN_1,
%   previous odometry OLDODOMETRY, current odometry NEWODOMETRY, and
%   then noise parameters ALPHA for the sampling-based odometry motion
%   model.

%   Copyright 2016-2018 The MathWorks, Inc.

%   References:
%
%   [1] S. Thrun, W. Burgard and D. Fox, Probabilistic Robotics.
%   Cambridge, MA: MIT Press, 2005.

%#codegen

    sizeOfXn_1 = size(Xn_1);

    Xn = zeros(sizeOfXn_1);

    % Compute pose difference
    delta = newOdometry - oldOdometry;
    delta(1, 3) = robotics.internal.angdiff(oldOdometry(1, 3), newOdometry(1, 3));

    % Compute motions - rotation1, translation and rotation2
    delta_trans = sqrt(delta(1,1)^2 + delta(1,2)^2);

    % Avoid computing a bearing from two poses that are extremely near each
    % other (happens on in-place rotation).
    if delta_trans < 0.01
        delta_rot1 = 0.0;
    else
        delta_rot1 = robotics.internal.angdiff(oldOdometry(1, 3), atan2(delta(1, 2), delta(1, 1)));
    end

    delta_rot2 = robotics.internal.angdiff(delta_rot1, delta(1, 3));

    % Additional step to ensure backward motion. The original model seems
    % to only take into account forward motion.
    delta_rot1_noise = min(abs(robotics.internal.angdiff([0, pi], delta_rot1)));
    delta_rot2_noise = min(abs(robotics.internal.angdiff([0, pi], delta_rot2)));

    % Compute samples for each motion
    d_rot1_hat = robotics.internal.angdiff(sample(sizeOfXn_1(1), ...
                                                  alpha(1, 1)*(delta_rot1_noise^2) + alpha(1, 2)*(delta_trans^2)), ...
                                           delta_rot1*ones(sizeOfXn_1(1), 1));

    d_trans_hat = delta_trans - ...
        sample(sizeOfXn_1(1), ...
               alpha(1, 3)*(delta_trans^2) + alpha(1, 4)*(delta_rot1_noise^2 + delta_rot2_noise^2));

    d_rot2_hat = robotics.internal.angdiff(sample(sizeOfXn_1(1), ...
                                                  alpha(1, 1)*(delta_rot2_noise^2) + alpha(1, 2)*(delta_trans^2)), ...
                                           delta_rot2*ones(sizeOfXn_1(1), 1));

    % Compute new states from previous states
    Xn(:,1) = Xn_1(:,1) + d_trans_hat.*cos(Xn_1(:,3) + d_rot1_hat);
    Xn(:,2) = Xn_1(:,2) + d_trans_hat.*sin(Xn_1(:,3) + d_rot1_hat);
    Xn(:,3) = robotics.internal.wrapToPi(Xn_1(:,3) + d_rot1_hat + d_rot2_hat);

end

function s = sample(numSamples, input)
%sample Gaussian sampling from uniform distribution

    sigmaNeg = -input;
    sigmaPos = input;

    r = (sigmaPos-sigmaNeg).*rand(numSamples,12) + sigmaNeg;
    s = sum(r, 2)./2;
end
