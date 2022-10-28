% INITCAGMPHD Constant acceleration gmphd initialization
%   phd = INITCAGMPHD initializes a constant acceleration gmphd
%   (Gaussian-mixture probability hypothesis density) filter with zero
%   components.
%
%   phd = INITCAGMPHD(detections) initializes a constant acceleration
%   gmphd filter based on the information provided in the input,
%   detections. detections must be a cell array of objectDetection objects.
%   Each element of the cell array must specify the measurement parameters,
%   as a struct with the following fields. Default values are used when a
%   field is missing:
%     Frame           - 'rectangular' or 'spherical'. Default: 'rectangular'
%     OriginPosition  - a 3-by-1 real vector.  Default: [0;0;0]
%     OriginVelocity  - a 3-by-1 real vector.  Default: [0;0;0]
%     Orientation     - a 3-by-3 orthonormal
%                       orientation matrix.    Default: eye(3)
%     HasElevation    - a logical scalar.      Default: true, elevation is measured
%     HasVelocity     - a logical scalar.      Default: false in rectangular, true in spherical
%     IsParentToChild - a logical scalar.      Default: false, orientation defines a rotation from child to parent frame
%
%   The function initializes a constant acceleration state with the
%   convention followed by constacc and cameas, [x;vx;ax;y;vy;ay;z;vz;az].
%
%   Notes:
%   ------
%   1.  You can use INITCAGMPHD as the FilterInitializationFcn property
%       for the trackingSensorConfiguration.
%   2.  When detections are provided as input, the function adds 1
%       component to the density which reflects the mean of the detections.
%   3.  The function uses the spread of measurements to describe the
%       positional covariance.
%   4.  The function configures the process noise of the filter
%       by assuming a unit standard deviation for the acceleration
%       change rate.
%   5.  The function specifies a maximum of 500 components in the filter.
%   6.  The function specifies that the filter has extended objects if
%       number of input detections are greater than 1.
%
%   Example 1: Detection with position measurement from a point object
%   ----------------------------------------------------------------
%   % The constant acceleration measurement function, cameas, provides a
%   % position measurement in 3-D. For example: [1;2;3] where x = 1, y = 2
%   % and z = 3 refers to x,y and z position of the object. Consider a
%   % point object located at [1;2;3]
%   detection = objectDetection(0,[1;2;3]);
%   phd = INITCAGMPHD(detection);
%   
%   % Check that the values of State has the same position estimate as the
%   % measurement
%   phd.States
%   
%   % Check that the filter is specified to track point objects
%   phd.HasExtent
%   
%   Example 2: Detections with position measurements from an extended object
%   ------------------------------------------------------------------------
%   % Consider an object located at position [1;2;3] with detections 
%   % uniformly spread around it's extent of size 1.2, 2.3 and 3.5 in x, 
%   % y and z direction respectively.
%   detections = cell(20,1);
%   location = [1;2;3];
%   dimensions = [1.2;2.3;3.5];
%   measurements = location + dimensions.*(-1 + 2*rand(3,20));
%   for i = 1:20
%       detections{i} = objectDetection(0,measurements(:,i));
%   end
%   phd = INITCAGMPHD(detections);
%   % Check the values of State has the same position
%   % estimates as the mean of measurements.
%   phd.States
%   mean(measurements,2)
%
%   % Check that the filter is specified to track extended objects
%   phd.HasExtent
%
%   See also: constacc, constaccjac, cameas, cameasjac, initcvgmphd,
%   initctgmphd, trackingSensorConfiguration

 
%   Copyright 2019 The MathWorks, Inc.

