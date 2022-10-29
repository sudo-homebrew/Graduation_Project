function rateObj = rosrate(desiredRate)
%ROSRATE Execute loop at fixed frequency using ROS time
%   R = ROSRATE(DESIREDRATE) allows you to execute a loop at a fixed
%   frequency equal to DESIREDRATE. The time source is linked to the same
%   time source that is used by the global ROS node.
%   The default setting for OverrunAction is 'slip', which executes the
%   next loop immediately if the LastPeriod is greater than DesiredPeriod.
%
%   If rosinit creates a ROS master in MATLAB, the global node uses
%   wall clock time.
%
%   The accuracy of the rate execution is influenced by the scheduling
%   resolution of your operating system and by the level of other
%   system activity.
%
%
%   Example:
%       % Initialize ROS master and node
%       rosinit;
%
%       % Create a rate object that runs at 20 Hz
%       r = rosrate(20);
%
%       % Start looping
%       for i = 1:10
%            % User Code
%            waitfor(r)
%       end
%
%       % Shutdown ROS
%       rosshutdown;
%
%   See also ros.Rate.

%   Copyright 2015-2020 The MathWorks, Inc.
%#codegen

    narginchk(1,1);

    if coder.target('MATLAB')
        try

            rateObj = ros.Rate([], desiredRate);

        catch ex
            % Save stack traces and exception causes internally, but do not
            % print them to the console
            rosex = ros.internal.ROSException.fromException(ex);
            throwAsCaller(rosex);
        end
    else
        % MATLAB Code generation redirection
        rateObj = ros.Rate([], desiredRate);
    end
end
