function createSimulinkBus(varargin)
%createSimulinkBus - Create Simulink bus corresponding to a ROS message
%
%   ros.createSimulinkBus(MODEL, MSGTYPE) creates the Simulink bus
%   object in the global scope (base workspace or data dictionary) for
%   Simulink model MODEL and ROS message
%   type MSGTYPE.
%      * If MSGTYPE includes other messages inside it, bus objects are
%        created for the nested messages as well.
%      * If the bus objects already exist in the global scope, they are
%        not recreated.
%      * Any created bus objects use the array size limits specified by the
%        "Manage Array sizes" dialog (From any Simulink model, choose
%        Tools > ROS > Manage Array Sizes).
%
%   ros.createSimulinkBus(MODEL) creates Simulink bus objects, if
%   needed, for all ROS message types used in MODEL.
%
%   Note:
%
%    By default, Simulink bus objects for ROS messages are automatically
%    created by the ROS blocks (Subscriber, Publisher, Blank  Message)
%    during model initialization.  Moreover, the buses are also
%    automatically refreshed if the user makes any changes via the "Manage
%    Array sizes for ROS Messages" Dialog. This function is only needed
%    when working with specialized blocks (e.g., MATLAB Function Block,
%    Data Store Write) that emit Simulink buses. In these cases, call this
%    function from the InitFcn for the model, subsystem or block (or from a
%    script that is run manually before working with the model).
%
%   Example:
%
%     ros.createSimulinkBus('mymodel', 'geometry_msgs/Pose')
%     ros.createSimulinkBus(gcs)

%   Copyright 2019 The MathWorks, Inc.

    ros.createSimulinkBus(varargin{:});
end
