% CVMEASMSCJAC  Jacobian of measurement using constant velocity (CV) model in MSC frame
%   The trackingEKF object allows you to specify the MeasurementJacobianFcn
%   property. CVMEASMSCJAC provides the Jacobian of the constant velocity
%   measurement function, cvmeasmsc, with respect to the state.
%
%   jacobian = CVMEASMSCJAC(state) calculates the jacobian with respect to
%   angular measurement (azimuth and elevation) of the state in the sensor
%   frame described by. The motion can be either in 2-D or 3-D.
%   If motion model is in 2D, values corresponding to elevation are assumed
%   to be zero.
%
%   jacobian = CVMEASMSCJAC(state,frame) provides the measurement jacobian
%   in the frame specified. The allowed values for frame are 'rectangular'
%   and 'spherical' (default)
%   If using 'rectangular' frame, the three elements present in the
%   measurement represent the x, y and z position of the target in the
%   observer's Cartesian frame.
%
%   jacobian = CVMEASMSCJAC(state,frame,laxes) specifies the axes, in
%   LAXES, of the local coordinate system. LAXES is a 3-by-3 orthonormal
%   orientation matrix whose columns specify the directions of the local x,
%   y, and z axes in the global Cartesian frame.
%
%   jacobian = CVMEASMSC(state,measurementParameters) specifies the
%   measurement parameters as a struct. The struct must have the following
%   fields (or a subset of them).
%       Frame           - either 'rectangular' or 'spherical' or an enum
%                         with the same values. Default: 'spherical'
%       Orientation     - a 3-by-3 orthonormal orientation matrix. Default:
%                         eye(3).
%       HasElevation    - a logical scalar, true if elevation is measured.
%       IsParentToChild - a logical scalar, true if the orientation is
%                         given as a parent to child frame rotation.
%
%   % Example 1: Obtain azimuth and elevation measurement jacobian
%   % ------------------------------------------------------------
%   mscState = [0.5;0;0.3;0;1e-3;1e-2];
%   cvmeasmscjac(mscState)
%
%   % Example 2: Obtain position measurement jacobian
%   % -----------------------------------------------
%   mscState = [0.5;0;0.3;0;1e-3;1e-2];
%   % Specify the frame as second input.
%   cvmeasmscjac(mscState,'rectangular');
%   % Specify the frame using measurementParameters
%   cvmeasmscjac(mscState,struct('Frame','rectangular'));
%
%   See also trackingMSCEKF, initcvmscekf, constvelmsc, cvmeasmsc

 
%   Copyright 2018 The MathWorks, Inc.

