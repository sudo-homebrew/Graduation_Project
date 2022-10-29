classdef PointCloud2Types
%This class is for internal use only. It may be removed in the future.

%PointCloud2Types Type conversions between ROS and MATLAB point clouds
%   The ROS data types are documented on the ROS wiki page for the
%   sensor_msgs/PointField data type.

%   Copyright 2014-2020 The MathWorks, Inc.

%#codegen

    methods (Static)
        function [mlType, numBytes] = rosToMATLABType(type)
        %rosToMATLABType Convert from ROS to MATLAB data type
        %   The data type for each point field is specified by an
        %   integer number. This function converts to the corresponding
        %   MATLAB data type.
        %
        %   See also matlabToROSType.

        % Set conversions between PointCloud2 and MATLAB types
            switch(type)
              case 1      %INT8
                mlType = 'int8';
                numBytes = 1;
              case 2      %UINT8
                mlType = 'uint8';
                numBytes = 1;
              case 3      %INT16
                mlType = 'int16';
                numBytes = 2;
              case 4      %UINT16
                mlType = 'uint16';
                numBytes = 2;
              case 5      %INT32
                mlType = 'int32';
                numBytes = 4;
              case 6      %UINT32
                mlType = 'uint32';
                numBytes = 4;
              case 7      %FLOAT32
                mlType = 'single';
                numBytes = 4;
              case 8      %FLOAT64
                mlType = 'double';
                numBytes = 8;
              otherwise
                coder.internal.error('ros:mlroscpp:pointcloud:InvalidTypeNum', ...
                                     type, '1 - 8');
            end
        end

        function [rosType, numBytes] = matlabToROSType(type)
        %matlabToROSType Convert from MATLAB to ROS data type
        %   This function converts a given MATLAB data type (string
        %   input) to a ROS data type (numeric).
        %   This is the inverse operation of the rosToMATLABType
        %   function.
        %
        %   See also rosToMATLABType.

        % Set conversions between PointCloud2 and MATLAB types
            switch(type)
              case 'int8'      %INT8
                rosType = 1;
                numBytes = 1;
              case 'uint8'     %UINT8
                rosType = 2;
                numBytes = 1;
              case 'int16'     %INT16
                rosType = 3;
                numBytes = 2;
              case 'uint16'    %UINT16
                rosType = 4;
                numBytes = 2;
              case 'int32'     %INT32
                rosType = 5;
                numBytes = 4;
              case 'uint32'    %UINT32
                rosType = 6;
                numBytes = 4;
              case 'single'    %FLOAT32
                rosType = 7;
                numBytes = 4;
              case 'double'    %FLOAT64
                rosType = 8;
                numBytes = 8;
              otherwise
                coder.internal.error('ros:mlroscpp:pointcloud:InvalidType', ...
                                     type, '(int8, uint8, int16, uint16, int32, uint32, single, double)');
            end
        end
    end
end
