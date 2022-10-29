function [varargout] = getBlockHelpMapNameAndPath(block_type)
%getBlockHelpMapNameAndPath  Returns the mapName and the relative path to the maps file for this block_type

% Copyright 2017-2021 The MathWorks, Inc.

varargout = cell(1, nargout);
[varargout{:}] = robotics.slcore.internal.block.getHelpMapNameAndPath(block_type, ...
                                                                      {
                                                                        'ros.slros.internal.block.ReadImage'       'rosReadImBlock';
                                                                        'ros.slros.internal.block.ReadPointCloud'  'rosReadPCBlock';
                                                                        'ros.slros.internal.block.ReadScan'        'rosReadScanBlock';
                                                                      }, ...
                                                                      { % All blocks are in ROS Toolbox
                                                                        'ros';
                                                                      } ...
                                                                     );
