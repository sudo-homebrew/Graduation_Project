% SINGERMEAS   Measurement based on the Singer acceleration motion model
%   Nonlinear tracking filters require a definition of a MeasurementFcn
%   property. SINGERMEAS provides the position measurement of the state
%   used in the Singer acceleration motion function, singer.
%
%   MEASUREMENT = SINGERMEAS(STATE) provides the position measurement of a
%   state used in the singer acceleration motion model in rectangular
%   coordinates relative to a stationary sensor located at the origin. The
%   motion can be either 1D, 2D or 3D, with the corresponding definition:
%       In a 1-D case, state = [x; vx; ax]. 
%       In a 2-D case, state = [x; vx; ax; y; vy; ay].
%       In a 3-D case, state = [x; vx; ax; y; vy; ay; z; vz; az].
%   If the input, STATE, is specified as a matrix, states must be
%   concatenated along columns, where each column represents a state
%   following the convention specified above. The output will be a matrix 
%   with the same number of columns as the input, where each column 
%   respresents the measurement from the corresponding state.
%
%   Positions are in meters; velocities are in meters/second; accelerations
%   are in meters/second^2. 
%   If the motion model is 1D values along the y and z axes are assumed to
%   be zero. If the motion model is 2D, values along the z axis are assumed
%   to be zero.
%
%   MEASUREMENT = SINGERMEAS(STATE, FRAME) measurement is in spherical
%   coordinates if FRAME is 'spherical' and in rectangular coordinates if
%   FRAME is 'rectangular' (default). 
%   If using rectangular coordinates, the three elements in measurement
%   represent (x,y,z) in meters. If the coordinates are in spherical form,
%   the measurement contains four elements (az, el, r, rr), where azimuth
%   (az, in degrees) is measured from x axis toward y axis, elevation (el,
%   in degrees) is measured from x-y plane toward z axis, r (in meters) is
%   the range, and rr (in meters/second) is the range rate. Range rate is
%   positive for a target moving away from the sensor.
%
%   MEASUREMENT = SINGERMEAS(STATE, FRAME, SENSORPOS, SENSORVEL) allows the
%   definition of the sensor position and sensor velocity.
%
%   MEASUREMENT = SINGERMEAS(STATE, FRAME, SENSORPOS, SENSORVEL, LAXES)
%   specifies the axes, in LAXES, of the local coordinate system. LAXES is
%   a 3-by-3 orthonormal orientation matrix whose columns specify the
%   directions of the local x, y, and z axes in the global Cartesian frame.
%
%   MEASUREMENT = SINGERMEAS(STATE, MEASUREMENTPARAMETERS) specifies the
%   measurement parameters as a struct. The struct must have the following
%   fields (or a subset of them):
%       Frame           - either 'rectangular' or 'spherical' or an enum 
%                         with the same values.
%       OriginPosition  - a 3-by-1 real vector.
%       OriginVelocity  - a 3-by-1 real vector.
%       Orientation     - a 3-by-3 orthonormal orientation matrix.
%       HasAzimuth      - a logical scalar, true if azimuth is measured.
%       HasElevation    - a logical scalar, true if elevation is measured.
%       HasVelocity     - a logical scalar, true if velocity is measured.
%       HasRange        - a logical scalar, true if range is measured.
%		IsParentToChild - a logical scalar, true if the orientation is 
%						  given as a parent to child frame rotation.
%
%   [..., BOUNDS] = SINGERMEAS(...), additionally returns the measurement
%   bounds, which are used by the filter in residual calculations.
%   The function returns the following bounds based on the frame.
%       Rectangular: An M-by-2 matrix where M is the number of elements in
%                    the measurement and each row is [-Inf Inf], indicating
%                    no wrapping.
%       Spherical:   An M-by-2 matrix where M is the number of elements in
%                    the measurement. The rows are in the following order:
%                       When HasAzimuth = true,   [-180 180]
%                       When HasElevation = true, [-90 90]
%                       When HasRange = true,     [-Inf Inf]
%                       When HasRangeRate = true, [-Inf Inf]
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
%     or a matrix with states concatenated along columns, either double or 
%     single precision.
%   * frame must be specified as either 'spherical' or 'rectangular'.
%   * sensorpos and sensorvel must be a 3 elements finite real column 
%     vector of the same type as state.
%   * laxes must be a 3x3 real rotation matrix of the same type as state.
%
%   Example
%   -------
%   % Define a state for 2-D singer acceleration motion
%   state = [1;10;3;2;20;5];
%
%   % Measurement in rectangular frame
%   measurement = SINGERMEAS(state)
%
%   % Measurement in spherical frame
%   measurement = SINGERMEAS(state, 'spherical')
%
%   % Measurement in spherical frame relative to a stationary sensor
%   % located at [1;-2;0]
%   measurement = SINGERMEAS(state, 'spherical', [1;-2;0], [0;0;0])
%
%   % Measurement in spherical frame relative to a stationary sensor
%   % located at [1;-2;0] that is rotated by 90 degrees about the z axis
%   % relative to the global frame
%   laxes = [0 -1 0; 1 0 0; 0 0 1];
%   measurement = SINGERMEAS(state, 'spherical', [1;-2;0], [0;0;0], laxes)
%
%   % Measurements from multiple 2D states in rectangular frame
%   states = [1 2 3; 10 20 30; 2 4 5; 20 30 40; 5 6 11; 1 3 1.5];
%   measurements = SINGERMEAS(states)
%
%   See also trackingEKF, trackingUKF, initcaekf, initcaukf, constacc,
%   cvmeas, ctmeas, singermeasjac

 
%   Copyright 2020 The MathWorks, Inc.

