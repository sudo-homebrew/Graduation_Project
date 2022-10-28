%singerProcessNoise  Process noise matrix for Singer accleration model
%   Q = singerProcessNoise(STATE) returns an N-by-N Singer acceleration
%   process noise matrix, where N is the number of elements in the state
%   vector, STATE. It uses the following default values:
%     T      - Time step, default: 1 second.
%     tau    - Target maneuver time constant, default: 20 seconds.
%     sigmam - Target maneuver standard deviation: 1 m/sec^2
%
%   Q = singerProcessNoise(STATE, T), additionally, allows you to specify
%   the time step, T.
%
%   Q = singerProcessNoise(STATE, T, TAU), additionally, allows you to
%   specify the target maneuver time constant, TAU.
%
%   Q = singerProcessNoise(STATE, T, TAU, SIGMAM), additionally, allows you
%   to sepcify the target maneuver standar deviation, SIGMAM.
%
%   Class support
%   -------------
%   STATE must be a finite real double or single precision vector with 3,
%   6, or 9 elements, corresponding to [pos;vel;acc] in 1, 2, or 3
%   dimensions, respectively. 
%   T must be a real finite double or single precision scalar.
%   TAU must be a real finite positive double or single precision vector or
%   scalar. If specified as vector, it must have the same number of
%   elements as the number of dimensions. If specified as a scalar, it will
%   be expanded to the number of dimensions.
%   SIGMAM must be a real finite positive double or single precision vector
%   or scalar. If specified as vector, it must have the same number of
%   elements as the number of dimensions. If specified as a scalar, it will
%   be expanded to the number of dimensions.
%
%   Example
%   -------
%   % Get the Singer process noise for a 3-D Singer state with default time
%   % step, target maneuver time constant, and standard deviation
%   Q1 = singerProcessNoise((1:9)')
%
%   % Get the Singer Process noise for a 3-D Singer state with time step of
%   % 2 seconds, target maneuver time constant of 10 seconds in x-axis and
%   % y-axis and 100 seconds in z-axis, and target maneuver standard
%   % deviation of 1 in x- and y- axes and 0 in z-axis.
%   Q2 = singerProcessNoise((1:9)', 2, [10 10 100], [1 1 0])
%
%   See also: singer, singerjac, initsingerekf

 
%   Copyright 2020 The MathWorks, Inc.

