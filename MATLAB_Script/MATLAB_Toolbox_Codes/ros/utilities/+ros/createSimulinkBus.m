function createSimulinkBus(modelName, rosMsgType)
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

%   Copyright 2014-2018 The MathWorks, Inc.

validateattributes(modelName, {'char'}, {'nonempty'}, 'ros.createSimulinkBus', 'modelName');

% BDROOT will throw a meaningful error if modelName is not the name of a
% loaded Simulink model (or if the name is syntactically invalid)
modelName = bdroot(modelName);

if isequal(exist('ros.slroscpp.internal.bus.Util','class'),8)
    busUtilObj = ros.slroscpp.internal.bus.Util;
else
    busUtilObj = ros.slros.internal.bus.Util;
end

if exist('rosMsgType', 'var')
    validateattributes(rosMsgType, {'char'}, {'nonempty'}, 'ros.createSimulinkBus', 'rosMsgType');
    msgTypes = cellstr(rosMsgType);
else
    msgTypes = busUtilObj.getBlockLevelMessageTypesInModel(modelName);
end


for i=1:numel(msgTypes)
    % Creates the bus in the global scope (if the bus doesn't already exist)
    busUtilObj.createBusIfNeeded(msgTypes{i}, modelName);
end
