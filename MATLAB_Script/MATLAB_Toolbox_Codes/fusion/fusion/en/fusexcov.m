%FUSEXCOV Covariance fusion using cross-covariance
%   FUSEXCOV estimates the fused state and covariance within a Bayesian
%   framework, where the cross-correlation between the tracks is unknown.
%
%   [fusedState,fusedCov] = FUSEXCOV(trackState,trackCov) fuses track
%   states, trackState, an N-by-M matrix, where N is the dimension of the
%   state and M is the number of tracks and their corresponding covariance
%   matrices, trackCov, an N-by-N-by-M matrix, and returns a single fused
%   state, fusedState, an N-by-1 vector, and its covariance, fusedCov, an
%   N-by-N matrix.
%  
%   [fusedState,fusedCov] = FUSEXCOV(trackState,trackCov, crossCovFactor)
%   allows a cross covariance factor (a scalar value) for the effective
%   correlation coefficient when computing the cross covariance. Default
%   value is 0.4.
%
%   Class Support 
%   ------------- 
%   Function supports single and double precision only. Class of output is
%   same as the class of trackState.
%
%   % Example 1: Cross-covariance fusion using default values
%   % -------------------------------------------------------
%   % Define state vector of tracks
%   x(:,1) = [1;2;0]; x(:,2) = [2;2;0]; x(:,3) = [2;3;0];
% 
%   % Define covariance of tracks
%   p(:,:,1) = [10 5 0; 5 10 0;0 0 1]; p(:,:,2) = [10 -5 0; -5 10 0;0 0 1];
%   p(:,:,3) = [12 9 0; 9 12 0;0 0 1];
% 
%   % Estimate fused state and covariance
%   [fusedState,fusedCov] = fusexcov(x,p)
% 
%   % Plot the results using trackPlotter
%   tPlotter = theaterPlot('XLim',[-10, 10],'YLim',[-10, 10],'ZLim',[-10, 10]);
%   tPlotter1 = trackPlotter(tPlotter,'DisplayName','Input Tracks','MarkerEdgeColor','blue')
%   tPlotter2 = trackPlotter(tPlotter,'DisplayName','Fused Tracks','MarkerEdgeColor','green')
%   plotTrack(tPlotter1, x', p)
%   plotTrack(tPlotter2, fusedState', fusedCov)
%   title('Cross-covariance fusion')
%
%   % Example 2: Cross-covariance fusion using cross covariance factor parameter
%   % -------------------------------------------------------------------------- 
%   % Define state vector of tracks
%   x(:,1) = [1;2;0]; x(:,2) = [2;2;0]; x(:,3) = [2;3;0];
% 
%   % Define covariance matrices of tracks
%   p(:,:,1) = [10 5 0; 5 10 0; 0 0 1]; p(:,:,2) = [10 -5 0; -5 10 0; 0 0 1]; 
%   p(:,:,3) = [12 9 0; 9 12 0; 0 0 1];
% 
%   % Estimate fused state and covariance
%   [fusedState,fusedCov] = fusexcov(x,p,0.5)
% 
%   % Plot the results using trackPlotter
%   tPlotter = theaterPlot('XLim',[-10, 10],'YLim',[-10, 10],'ZLim',[-10, 10]);
%   tPlotter1 = trackPlotter(tPlotter,'DisplayName','Input Tracks','MarkerEdgeColor','blue')
%   tPlotter2 = trackPlotter(tPlotter,'DisplayName','Fused Tracks','MarkerEdgeColor','green')
%   plotTrack(tPlotter1, x', p)
%   plotTrack(tPlotter2, fusedState', fusedCov)
%   title('Cross-covariance fusion')
%
%   See also fusecovint, fusecovunion

 
% Copyright 2018 The MathWorks, Inc.

