function rosshutdown()
%ROSSHUTDOWN Shut down the global node and ROS master
%   ROSSHUTDOWN shuts down the global node and, if it is running in MATLAB,
%   the ROS master.
%
%   When you have finished working with ROS, use ROSSHUTDOWN to shut
%   down the ROS entities that were created by rosinit. If the
%   global node and ROS master are not running, this function has no effect.
%   Any ROS entities that depend on the global node, for example subscribers
%   created with rossubscriber, will be deleted and unusable after calling
%   ROSSHUTDOWN.
%
%   Note that a ROS master that is running outside of MATLAB will not be
%   affected by a call to ROSSHUTDOWN. Only the ROS master started within
%   your MATLAB session will be shut down.
%
%   See also ROSINIT.

%   Copyright 2020 The MathWorks, Inc.

    try
        ros.internal.Global.node('clear');
        ros.internal.Global.core('clear');
    catch ex
        % Save stack traces and exception causes internally, but do not
        % print them to the console
        rosex = ros.internal.ROSException.fromException(ex);
        throwAsCaller(rosex);
    end

end
