function [fusedState,fusedCov] = fusecovint(trackState,trackCov,minProp)
%FUSECOVINT Covariance fusion using covariance intersection
%   FUSECOVINT computes the fused state and covariance as an intersection
%   of the individual covariances. This is done by creating a convex
%   combination of the covariances, and finding weights such that the fused
%   covariance is minimized.
%
%   [fused,fusedCov] = FUSECOVINT(trackState,trackCov) fuses track
%   states, trackState, an N-by-M matrix, where N is the dimension of the
%   state and M is the number of tracks and their corresponding covariance
%   matrices, trackCov, an N-by-N-by-M matrix, and returns a single fused
%   state, fusedState, an N-by-1 vector, and its covariance, fusedCov, an
%   N-by-N matrix.
% 
%   [fusedState,fusedCov] = FUSECOVINT(trackState,trackCov, minProp)
%   estimates the fused covariance by minimizing the minProp (determinant
%   or trace ) property. Valid values are 'det' (default) or 'trace'.
%
%   Class Support
%   -------------
%   Function supports single and double precision only. Class of output is
%   same as the class of trackState.
%
%   Example 1: Covariance intersection fusion using default values
%   --------------------------------------------------------------
%   % Define state vector of tracks
%   x(:,1) = [1;2;0]; x(:,2) = [2;2;0]; x(:,3) = [2;3;0];
% 
%   % Define covariance of tracks
%   p(:,:,1) = [10 5 0; 5 10 0; 0 0 1]; p(:,:,2) = [10 -5 0; -5 10 0; 0 0 1]; 
%   p(:,:,3) = [12 9 0; 9 12 0; 0 0 1]; 
% 
%   % Estimate fused state and covariance
%   [fusedState,fusedCov] = fusecovint(x,p);
% 
%   % Plot the results using trackPlotter
%   tPlotter = theaterPlot('XLim',[-10, 10],'YLim',[-10, 10],'ZLim',[-10, 10]);
%   tPlotter1 = trackPlotter(tPlotter,'DisplayName','Input Tracks','MarkerEdgeColor','blue')
%   tPlotter2 = trackPlotter(tPlotter,'DisplayName','Fused Tracks','MarkerEdgeColor','green')
%   plotTrack(tPlotter1, x', p)
%   plotTrack(tPlotter2, fusedState', fusedCov)
%   title('Covariance intersection fusion')
%
%   Example 2: Covariance intersection fusion using trace minimization
%   ------------------------------------------------------------------
%   % Define state vector of tracks
%   x(:,1) = [1;2;0]; x(:,2) = [2;2;0]; x(:,3) = [2;3;0];
% 
%   % Define covariance matrices of tracks
%   p(:,:,1) = [10 5 0; 5 10 0; 0 0 1]; p(:,:,2) = [10 -5 0; -5 10 0; 0 0 1]; 
%   p(:,:,3) = [12 9 0; 9 12 0; 0 0 1]; 
% 
%   % Estimate fused state and covariance
%   [fusedState,fusedCov] = fusecovint(x,p,'trace');
% 
%   % Plot the results using trackPlotter
%   tPlotter = theaterPlot('XLim',[-10, 10],'YLim',[-10, 10],'ZLim',[-10, 10]);
%   tPlotter1 = trackPlotter(tPlotter,'DisplayName','Input Tracks','MarkerEdgeColor','blue')
%   tPlotter2 = trackPlotter(tPlotter,'DisplayName','Fused Tracks','MarkerEdgeColor','green')
%   plotTrack(tPlotter1, x', p)
%   plotTrack(tPlotter2, fusedState', fusedCov)
%   title('Covariance intersection fusion')
%
%   See also fusexcov, fusecovunion

%   References
%   --------- 
%   [1] Matzka, Stephan, and Richard Altendorfer. "A comparison of
%   track-to-track fusion algorithms for automotive sensor fusion."
%   Multisensor Fusion and Integration for Intelligent Systems. Springer
%   Berlin Heidelberg, 2009. 69-81.
%   [2] Julier, Simon, and Jeffrey K. Uhlmann. "General decentralized data
%   fusion with covariance intersection (CI)." Multisensor Data Fusion. CRC
%   Press, 2001.

%   Copyright 2018 The MathWorks, Inc.

%#codegen
 
narginchk(2,3);
% Input validation and error check
fusion.internal.validateCovFusion(trackState,trackCov,'fusecovint');

classToUse = class(trackState);
trackCovMat = cast(trackCov,classToUse);

% finding number of tracks given by different sensors
ntracks =size(trackCovMat,3);

% finding the size of the covariance in matrix
covSize=size(trackCovMat,2);

% Initializing the zero and identity matrix
zeroMat = zeros(ntracks-1,1,classToUse);
identityMat  = ones(1,ntracks, classToUse);
if (nargin==2)
    minProp='det';
end
minProp = validatestring(minProp,{'det','determinant','trace'},...
    'fusecovint', 'minProp', 3); 
if (strcmp(minProp,'determinant'))
  minProp = 'det';
end

CovMatrix = zeros(1,ntracks,classToUse);
switch  minProp
    %Calculating the trace of each covariance matrix given by each sensor
    %for an object.
    case 'trace'
        %trace of covariance matrix
        for i = 1:ntracks
            CovMatrix(i) = trace(trackCovMat(:,:,i));
        end
    case 'det'
        %determinant of covariance matrix
        for i = 1:ntracks
            CovMatrix(i) = det(trackCovMat(:,:,i));            
        end
end

%Initializing 'A' with size 'ntracks-1 by ntracks' 
A = zeros(ntracks-1,ntracks,classToUse);
%Constructing the Matrix 'A' 
for i=1:ntracks-1
    A(i,i)= CovMatrix(i);
    A(i,i+1)= -1*CovMatrix(i+1);
end
%Padding Our 'A' matrix with identity matrix
A=[A;identityMat];

%Constructing 'B' matrix with size 'ntracks by 1'
B=[zeroMat;1];

%Calculating the weights by using A*Weight=B which can be computed as Weight = A\B.
Weight = A\B;

%Initializing the fused covariance matrix
initialFusedCov = zeros(covSize,covSize,classToUse);

%Calculating the fused Covariance matrix
eyeCov = eye(covSize,classToUse);
for i = 1:ntracks
    initialFusedCov = initialFusedCov + Weight(i) * (trackCovMat(:,:,i)\eyeCov);
end

%Calculating the fused covariance 
fusedCov =(initialFusedCov\eyeCov);

%Initializing the fused state
initialFusedState=zeros(covSize,1,classToUse);

for i = 1:ntracks
    initialFusedState = initialFusedState + ( Weight(i) * (trackCovMat(:,:,i)\eyeCov) * trackState(:,i) );
end

%Calculating the fused state 
fusedState=fusedCov*initialFusedState;

end