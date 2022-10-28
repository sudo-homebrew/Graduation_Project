function errorIfBlocksConflict(modelName, varargin)
%This function is for internal use only. It may be removed in the future.

%ERRORIFBLOCKSCONFLICT Verifies a Simulink model for invalid usage of ROS
%and ROS 2 blocks.
% The function will throw an error if:
% 1. The model contains both ROS and ROS 2 blocks (on Update diagram)
% 2. If 'Hardware board' setting is not compatible with the blocks (on
% Deploy)
%
% Examples:
%   ROS.SLROS.INTERNAL.BLOCK.ERRORIFBLOCKSCONFLICT(modelName) searches the
%   model specified by modelName argument, and throws an error if it has
%   both ROS and ROS 2 blocks and.
%
%   ROS.SLROS.INTERNAL.BLOCK.ERRORIFBLOCKSCONFLICT(modelName, 'InCodeGen',
%   true) searches the model specified by modelName argument, and throws an
%   error if:
%       a) 'Hardware board' is set to 'Robot Operating System 2 (ROS 2)
%       and the model contains any ROS blocks
%       b) 'Hardware board' is set to 'Robot Operating System (ROS)' and the
%       model contains any ROS 2 blocks


%   Copyright 2019-2021 The MathWorks, Inc.

p = inputParser;
addParameter(p, 'InCodeGen', false, @(x)islogical(x));
parse(p, varargin{:});
cgen = p.Results.InCodeGen;

[~, ~, pubSubMsgBlockList, paramBlockList,  imageBlockList, timeBlockList, svcCallBlockList, roslogfileBlockList, writeBlocksList]...
    = ros.slros.internal.bus.Util.getROSBlocksInModel(modelName);
% Explicitly search for  ROS 1 'Read Point Cloud' blocks since above function does not account for it
pointCloudBlks = ros.slros.internal.bus.Util.listBlocks(modelName, 'ros.slros.internal.block.ReadPointCloud');

pubSubMsgBlockList_ros2 = {};
ros2logfileBlockList = {};
imageBlockList_ros2 = {};

if isequal(exist('ros.slros2.internal.bus.Util','class'), 8)
    % For internal use: If the ROS 2 Simulink support is installed then
    % search for ROS 2 blocks.
    [~, ~, pubSubMsgBlockList_ros2, ~,  imageBlockList_ros2, ~, svcCallBlockList_ros2, ros2logfileBlockList, writeBlocksList_ros2]...
        = ros.slros2.internal.bus.Util.getROS2BlocksInModel(modelName);
end
% Explicitly search for  ROS 2 'Read Point Cloud' blocks since above function does not account for it
pointCloudBlks_ros2 = ros.slros.internal.bus.Util.listBlocks(modelName, 'ros.slros2.internal.block.ReadPointCloud');

roslogfileBlockList = roslogfileBlockList(contains(get_param(roslogfileBlockList,'ReferenceBlock'),'robotlib'));
ros2logfileBlockList = ros2logfileBlockList(contains(get_param(ros2logfileBlockList,'ReferenceBlock'),'ros2lib'));

allROSBlocks = [pubSubMsgBlockList; paramBlockList; ...
                imageBlockList; timeBlockList; ...
                svcCallBlockList; roslogfileBlockList; pointCloudBlks;...
                writeBlocksList];
allROS2Blocks = [pubSubMsgBlockList_ros2;imageBlockList_ros2;...
                 svcCallBlockList_ros2; ros2logfileBlockList;...
                 pointCloudBlks_ros2; writeBlocksList_ros2];
anyROSBlockFound = ~isempty(allROSBlocks);
anyROS2BlockFound = ~isempty(allROS2Blocks);

% Error when both blocks are found in the same model
if cgen
    ROS1HWBoard = message('ros:slros:cgen:ui_hwboard').getString;
    ROS2HWBoard = message('ros:slros2:codegen:ui_hwboard').getString;
    thisHWBoard = get_param(modelName, 'HardwareBoard');
    % cgen is set to 'true' by onBuildEntryHook error for setting incorrect
    % Hardware board happens at CTRL+B
    if anyROSBlockFound && ~contains(thisHWBoard, ROS1HWBoard)
        throwAsCaller(MSLException([], message('ros:slros:cgen:HWBoardIncorrectError',allROSBlocks{1},ROS1HWBoard)));
    end
    if anyROS2BlockFound && ~contains(thisHWBoard, ROS2HWBoard)
        throwAsCaller(MSLException([], message('ros:slros:cgen:HWBoardIncorrectError', allROS2Blocks{1}, ROS2HWBoard)));
    end
else
    % error for mixing ROS and ROS2 blocks happens at CTRL+D
    if anyROS2BlockFound && anyROSBlockFound
        blk1 = getSimulinkBlockHandle(allROSBlocks{1});
        blk2 = getSimulinkBlockHandle(allROS2Blocks{1});
        throwAsCaller(MSLException([blk1, blk2], message('ros:slros:cgen:ROS2ROS1TogetherError')));
    end
end
