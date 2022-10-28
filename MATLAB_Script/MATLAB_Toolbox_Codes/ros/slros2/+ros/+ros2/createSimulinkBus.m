function [busnames] = createSimulinkBus(modelName, rosMsgType)
%createSimulinkBus - Create Simulink bus corresponding to a ROS 2 message
%
%   BUSNAMES  = ros.ros2.createSimulinkBus(MODEL, MSGTYPE) creates the
%   Simulink bus object for Simulink model MODEL and ROS 2 message type
%   MSGTYPE and returns cell-array of Simulink bus names.
%      * If MSGTYPE includes other messages inside it, bus objects are
%        created for the nested messages as well.
%      * If the bus objects already exist in the global scope, they are
%        not recreated.
%      * Any created bus objects use the array size limits specified by the
%        "Manage Array sizes" dialog (From any Simulink model, choose
%        Tools > ROS > Manage Array Sizes).
%
%   ros.ros2.createSimulinkBus(MODEL) creates Simulink bus objects, if
%   needed, for all ROS 2 message types used in MODEL.
%
%   Note:
%
%    By default, Simulink bus objects for ROS 2 messages are automatically
%    created by the ROS blocks (Subscriber, Publisher, Blank  Message)
%    during model initialization.  Moreover, the buses are also
%    automatically refreshed if the user makes any changes via the "Manage
%    Array sizes for ROS Messages" dialog. This function is only needed
%    when working with specialized blocks (e.g., MATLAB Function Block,
%    Data Store Write) that emit Simulink buses. In these cases, call this
%    function from the InitFcn for the model, subsystem or block (or from a
%    script that is run manually before working with the model).
%
%   Example:
%
%     busnames = ros.ros2.createSimulinkBus('mymodel', 'geometry_msgs/Pose');
%     busnames = ros.ros2.createSimulinkBus(gcs);
%
% See also ROS.CREATESIMULINKBUS

%  Copyright 2019 The MathWorks, Inc.

validateattributes(modelName, {'char', 'string'}, {'nonempty'}, 'ros.ros2.createSimulinkBus', 'modelName');

% BDROOT will throw a meaningful error if modelName is not the name of a
% loaded Simulink model (or if the name is syntactically invalid)

modelName = convertStringsToChars(modelName);
modelName = bdroot(modelName);

if exist('rosMsgType', 'var')
    validateattributes(rosMsgType, {'char','string'}, {'nonempty'}, 'ros.ros2.createSimulinkBus', 'rosMsgType');
    msgTypes = cellstr(rosMsgType);
else
    msgTypes = ros.slros2.internal.bus.Util.getBlockLevelMessageTypesInModel(modelName);
end

busnames = cell(numel(msgTypes),1);

for i=1:numel(msgTypes)
    % Creates the bus in the global scope (if the bus doesn't already exist)
    ros.slros2.internal.bus.Util.createBusIfNeeded(msgTypes{i}, modelName);
    busnames{i} = ros.slros2.internal.bus.Util.rosMsgTypeToBusName(msgTypes{i});
end

