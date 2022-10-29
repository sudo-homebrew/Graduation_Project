function jacobian = singermeasjac(state, varargin)
% SINGERMEASJAC  Jacobian of the measurement using the Singer acceleration model
%   The trackingEKF object allows you to specify the MeasurementJacobianFcn
%   property. SINGERMEASJAC provides the Jacobian of the Singer
%   acceleration measurement function, singermeas, with respect to the
%   state.
%
%   jacobian = SINGERMEASJAC(STATE) provides the Jacobian of the
%   measurement in rectangular coordinates relative to the state. State can
%   be of a Singer acceleration in 1D, 2D or 3D.
%       In a 1-D case, state = [x; vx; ax]. 
%       In a 2-D case, state = [x; vx; ax; y; vy; ay].
%       In a 3-D case, state = [x; vx; ax; y; vy; ay; z; vz; az].
%   Positions are in meters; velocities are in meters/second; accelerations
%   are in meters/second^2.
%
%   jacobian = SINGERMEASJAC(STATE, FRAME) calculates the Jacobian of the
%   measurement in spherical coordinates if FRAME is 'spherical' and in
%   rectangular coordinates if FRAME is 'rectangular' (default). 
%   In singermeas, the measurement is [x; y; z] in rectangular frame and 
%   [az; el; range; range-rate] in spherical frame. Therefore, the size of
%   the measurement Jacobian is:
%       In the rectangular case: 3-by-n
%       In the spherical case: 4-by-n
%   where n is the number of states.
%
%   jacobian = SINGERMEASJAC(STATE, FRAME, SENSORPOS, SENSORVEL) allows the
%   definition of the sensor position and sensor velocity.
%
%   jacobian = SINGERMEASJAC(STATE, FRAME, SENSORPOS, SENSORVEL, LAXES)
%   specifies the axes, in LAXES, of the local coordinate system. LAXES is
%   a 3-by-3 orthonormal orientation matrix whose columns specify the
%   directions of the local x, y, and z axes in the global Cartesian frame.
%
%   jacobian = SINGERMEASJAC(STATE, MEASUREMENTPARAMETERS) specifies the
%   measurement parameters as a struct. The struct must have the following
%   fields (or a subset of them):
%       Frame - either 'rectangular' or 'spherical' or an enum with the
%               same values.
%       OriginPosition - a 3-by-1 real vector.
%       OriginVelocity - a 3-by-1 real vector.
%       Orientation    - a 3-by-3 orthonormal orientation matrix.
%       HasAzimuth     - a logical scalar, true if azimuth is measured.
%       HasElevation   - a logical scalar, true if elevation is measured.
%       HasVelocity    - a logical scalar, true if range rate is measured.
%		HasRange	   - a logical scalar, true if range is measured.
%		IsParentToChild - a logical scalar, true if the orientation is 
%						  given as a parent to child frame rotation.
%
%   When Frame is 'rectangular', HasVelocity determines if the measurement
%   is [x;y;z;vx;vy;vz] or [x;y;z].
%
%   When Frame is 'spherical', the measurement order is [az,el,r,rr]
%   HasAzimuth   - determines if measurement contains azimuth measurement 
%   HasElevation - determines if measurement contains elevation measurement
%   HasRange     - determines if measurement contains range measurement
%   If HasRange is true:
%   HasVelocity  - determines if measurement contains range-rate.
%   If HasRange is false:
%   HasVelocity  - no effect on measurement. 
%
%   Class support
%   -------------
%   * state must be a finite real vector of the dimensions specified above,
%     either double or single precision.
%   * frame must be specified as either 'spherical' or 'rectangular'.
%   * sensorpos and sensorvel must be a 3 elements finite real column 
%     vector of the same type as state.
%   * laxes must be a 3x3 real rotation matrix of the same type as state.
%
%   Example
%   -------
%   % Define a state for 2-D Singer acceleration motion
%   state = [1;10;0;2;20;1]; 
%
%   % Measurement Jacobian in rectangular frame
%   jacobian = SINGERMEASJAC(state) 
%
%   % Measurement Jacobian in spherical frame
%   jacobian = SINGERMEASJAC(state, 'spherical')
%
%   % Measurement Jacobian in spherical frame relative to a stationary  
%   % sensor located at [1;-2;0]
%   jacobian = SINGERMEASJAC(state, 'spherical', [1;-2;0], [0;0;0]) 
%
%   % Measurement Jacobian in spherical frame relative to a stationary  
%   % sensor located at [1;-2;0] that is rotated by 90 degrees about the z  
%   % axis relative to the global frame
%   laxes = [0 -1 0; 1 0 0; 0 0 1];
%   jacobian = SINGERMEASJAC(state, 'spherical', [1;-2;0], [0;0;0], laxes) 
%
%   See also trackingEKF, initcaekf, constacc, singermeas, ctmeasjac, cvmeasjac

%   Copyright 2020 The MathWorks, Inc.

%#codegen

jacobian = matlabshared.tracking.internal.fusion.accelmeasjac(state, mfilename, varargin{:});