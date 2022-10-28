% CTRECT A constant turn-rate rectangular target motion model
%   states = CTRECT(states) calculates the state at the next time-step
%   based on the current state, assuming dT = 1 second.
%
%   state = CTRECT(state, dt) calculates the state at the next time-step
%   based on the current state and the time interval, dt.
%
%   state = CTRECT(state, w, dt) calculates the state at the next
%   time-step based on the current state, the noise w, and the time
%   interval, dt.
%
%   state can be defined as a vector or matrix. If specified as a vector,
%   the orientation of the state vector can be either column or row vector,
%   and the output will match the input's orientation. The function uses
%   the speed and heading to describe the state according to the
%   following convention:
%
%   state = [x;y;s;theta;omega;L;W]
%
%   where:
%   x and y are positions in meters
%   s is the speed in meters/second
%   theta is the orientation or heading of the rectangle in degrees
%   omega is the turn-rate in degrees/second
%   L and W are length and width of the rectangle respectively in meters
%
%   If state is specified as a matrix, it must be concatenated along
%   columns, where each column represents a state defined according to the
%   convention above.
%
%   If specified, w must be a scalar or a 2-by-N matrix, where N is the
%   number of state columns. If specified as a scalar, it will be expanded
%   to a 2-by-N matrix. The first row specifies the process noise in
%   acceleration (meters/second^2) and the second row specifies the process
%   noise in yaw acceleration (degrees/second^2)
%
%   Example 1: predict a constant turn rate rectangular state
%   ---------------------------------------------------------
%   % Define a state vector
%   state = [1;2;2;30;1;4.7;1.8];
%
%   % Predict the state assuming dt = 1 second
%   state = CTRECT(state);
%
%   % Predict the state given dt = 0.1 second
%   state = CTRECT(state, 0.1);
%
%   % Predict the state using a process noise inputs and dt = 0.1 second
%   state = CTRECT(state, [0;0], 0.1);
%
%   % Example 2: predict multiple constant turn-rate rectangular states
%   --------------------------------------------------------------------
%   % Define a state matrix
%   states = [1 3 4;-1 2 10;5 3 1.3;1 1.3 2.1;30 0 -30;4.7 3.4 4.5;1.8 2 3];
%
%   % Predict the state assuming dt = 1 second
%   states = ctrect(states);
%
%   % Predict the state given dt = 0.1 second
%   states = ctrect(states,0.1);
%
%   % Predict the state using process noise inputs and dt = 0.1 second
%   states = ctrect(states, 0.1*randn(2,3), 0.1);
%   
%   See also: gmphd, ctrectjac, ctrectmeas, ctrectmeasjac, initctrectgmphd

 
%   Copyright 2019 The MathWorks, Inc.

