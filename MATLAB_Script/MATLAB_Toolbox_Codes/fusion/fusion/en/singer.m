% SINGER      Singer acceleration motion model in state-space.
%   Both trackingEKF and trackingUKF require a definition of a
%   StateTransitionFcn property. SINGER provides a 2nd-order Markov process
%   model, where the acceleration is modeled as exponentially decaying.
%
%   state = SINGER(state) calculates the state at the next time-step
%   based on the current state, assuming dt = 1 second.
%
%   state = SINGER(state, dt) calculates the state at the next time-step
%   based on the current state and the time interval, dt. 
%
%   state = SINGER(state, dt, tau) additionally, allows you to specify
%   the target maneuver time constant. If not specified, it is assumed to
%   be 20 seconds.
%
%   The Singer acceleration model assumes that the acceleration at time
%   step k+1 depends on the acceleration at time step k with the following
%   equation:
%     a(k+1) = a(k)*exp(-dt/tau)
%   where: 
%     a(k)    is the acceleration at time step k
%     tau     is the target maneuver time constant
%     dt      is the time step
%
%   state can be defined as a vector or matrix for 1, 2 or 3 dimensions. 
%   If specified as a vector, the orientation of the state vector can be 
%   either column or row vector, and the output will match the input's 
%   orientation.
%       In a 1-D case, state = [x; vx; ax]. 
%       In a 2-D case, state = [x; vx; ax; y; vy; ay].
%       In a 3-D case, state = [x; vx; ax; y; vy; ay; z; vz; az].
%   If specified as a matrix, states must be concatenated along columns,
%   where each column represents a state following the convention specified
%   above.
%
%   Positions are in meters; velocities are in meters/second; accelerations
%   are in meters/second^2.
%
%   Note: The Singer model only supports additive noise. To specify the
%   additive process noise matrix, use the function singerProcessNoise.
%
%   Class support
%   -------------
%   state must be a real finite double or single precision vector or matrix
%   with the dimensions specified above.
%   dt must be a real finite double or single precision scalar.
%   tau must be a real finite double or single precision vector or scalar.
%   If specified as vector, it must have the same number of elements as the
%   number of dimensions. If specified as a scalar, it will be expanded to
%   the number of dimensions.
%   
%
%   % Example 1: Predict a Singer acceleration state with various time steps
%   % ----------------------------------------------------------------------
%   % Define a state vector for 3-D Singer acceleration motion
%   state = [1;1;0;2;5;-2;0;3;0];
%
%   % Predict the state assuming dt = 1 second
%   state = SINGER(state) 
%
%   % Predict the state given dt = 0.1 second
%   state = SINGER(state, 0.1) 
%
%   % Example 2: Predict multiple Singer acceleration states
%   % ------------------------------------------------------
%   % Define a state matrix for 2-D Singer acceleration motion
%   states = [1 2 2.5;1 2.5 3;0 -1 2;2 3 -1;5 0 3;-2 4 2];
%   
%   % Predict the state assuming dt = 1 second
%   states = SINGER(states)
%   
%   % Predict the state given dt = 0.1 second
%   states = SINGER(states, 0.1)
%
%   % Example 3: Predict and measure the position using a singer model
%   % ----------------------------------------------------------------
%   % Define a state vector for a 2-D Singer acceleration motion
%   state = [10;-10; 3;0;10;-3];
%   dt = 0.2; % time step in seconds
%   tau = 10; % Maneuver time is 10 seconds
%   
%   % Use SINGER to create a trajectory and measure the positions with singermeas
%   position = zeros(2, 100); % Pre-allocate memory
%   measurement = zeros(3, 100); % Pre-allocate memory
%   for i = 1:1:100
%       state = SINGER(state, dt, tau);
%       position(:,i) = [state(1); state(4)];
%       measurement(:,i) = singermeas(state);
%   end
%   plot(position(1,:), position(2,:))
%   hold on
%   plot(measurement(1,:), measurement(2,:), '.')
%   title('Singer Acceleration Model'); xlabel('X[m]'); ylabel('Y[m]')
%   legend('Trajectory', 'Measurements'); hold off;
%
%   See also trackingEKF, initsingerekf, constvel, constacc, constturn,
%   singerjac, singerProcessNoise, singermeas

 
%   Copyright 2020 The MathWorks, Inc.

