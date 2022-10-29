function [varargout] = getBlockHelpMapNameAndPath(block_type)
%getBlockHelpMapNameAndPath  Returns the mapName and the relative path to the maps file for this block_type

% Copyright 2021 The MathWorks, Inc.

varargout = cell(1, nargout);
[varargout{:}] = robotics.slcore.internal.block.getHelpMapNameAndPath(block_type, ...
                                                                      {
                                                                        'ros.slros2.internal.block.ReadImage'       'ros2ReadImBlock';
                                                                        'ros.slros2.internal.block.ReadPointCloud'  'ros2ReadPCBlock';
                                                                        'ros.slros2.internal.block.ReadScan'        'ros2ReadScanBlock';
                                                                      }, ...
                                                                      { % All blocks are in ROS Toolbox
                                                                        'ros';
                                                                      } ...
                                                                     );
