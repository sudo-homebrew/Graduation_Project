function [fusedState,fusedCov] = fusecovunion(trackState,trackCov)
%FUSECOVUNION Covariance fusion using covariance union
%   FUSECOVUNION estimates the fused state and covariance such that it
%   maintains consistency.
%
%   [fusedState,fusedCov] = FUSECOVUNION(trackState,trackCov) fuses track
%   states, trackState, an N-by-M matrix, where N is the dimension of the
%   state and M is the number of tracks and their corresponding covariance
%   matrices, trackCov, an N-by-N-by-M matrix, and returns a single fused
%   state, fusedState, an N-by-1 vector, and its covariance, fusedCov, an
%   N-by-N matrix.
%
%   Class Support
%   -------------
%   Function supports single and double precision only. Class of output is
%   same as the class of trackState.
%
%   Example 1: Covariance union fusion
%   ----------------------------------
%   % Define state vector of tracks
%   x(:,1) = [1;2;0]; x(:,2) = [2;2;0]; x(:,3) = [2;3;0]; 
% 
%   % Define covariance of tracks
%   p(:,:,1) = [10 5 0; 5 10 0; 0 0 1]; p(:,:,2) = [10 -5 0; -5 10 0; 0 0 1]; 
%   p(:,:,3) = [12 9 0; 9 12 0; 0 0 1];
% 
%   % Estimate fused state and covariance
%   [fusedState,fusedCov] = fusecovunion(x,p);
% 
%   % Plot the results using trackPlotter
%   tPlotter = theaterPlot('XLim',[-10, 10],'YLim',[-10, 10],'ZLim',[-10, 10]);
%   tPlotter1 = trackPlotter(tPlotter,'DisplayName','Input Tracks','MarkerEdgeColor','blue')
%   tPlotter2 = trackPlotter(tPlotter,'DisplayName','Fused Tracks','MarkerEdgeColor','green')
%   plotTrack(tPlotter1, x', p)
%   plotTrack(tPlotter2, fusedState', fusedCov)
%   title('Covariance union fusion')
%
%   See also fusexcov, fusecovint

%   References
%   ---------
%   [1] Matzka, Stephan, and Richard Altendorfer. "A comparison of
%   track-to-track fusion algorithms for automotive sensor fusion."
%   Multisensor Fusion and Integration for Intelligent Systems. Springer
%   Berlin Heidelberg, 2009. 69-81.
%   [2] Bochardt, Ottmar, et al. "Generalized information representation
%   and compression using covariance union." Information Fusion, 2006 9th
%   International Conference on. IEEE, 2006.
%   [3] Khachiyan, Leonid G.  Rounding of Polytopes in the Real Number
%   Model of Computation.  Mathematics of Operations Research, Vol. 21, No.
%   2 (May, 1996) pp. 307--320.

% Copyright 2018 The MathWorks, Inc.

%#codegen

narginchk(2,2);

% Input validation and error check
fusion.internal.validateCovFusion(trackState,trackCov,'fusecovunion');

classToUse = class(trackState);
trackCovMat = cast(trackCov,classToUse);

% finding number of tracks given by different sensors
ntracks =size(trackCovMat,3);
eyeCovSize = eye(size(trackCovMat(:,:,1)),classToUse);
nPoints = 100;
points = zeros(size(trackState(:,1),1),nPoints*ntracks,classToUse);
for i = 1:ntracks
    pts = generatePoints(1,trackCovMat(:,:,i)\eyeCovSize,trackState(:,i),nPoints);
    idx = ((i-1)*nPoints)+1;
    points(:,idx:idx+nPoints-1) = pts;
end
[fusedState, fusedCov] = lowner(points, 0.001);
end


function [points] = generatePoints(gammaThreshold,invTrackCov,trackState,numPoints)
% finding number of tracks given by different sensors
classToUse = class(trackState);

% finding the size of the covariance matrix
covSize = size(trackState,1);

[V,D] = eig(invTrackCov );  % Decomposition of invTrackCov
D = abs(D);    % this is added for codegen support as positive semidefinite matrix have all positive eigenvalues
[Y,I] = sort(diag(D));  % Sorting of eigenvalues by ascending order
diagMat = diag(Y,0);  % Diagonal matrix of sorted eigenvalues
L = V(:,I );    % Permutation of eigenvectors corresponding to eigenvalues
L = real(L);   % this is added for codegen support
points = zeros(covSize,numPoints,classToUse);
x = zeros(size(trackState),classToUse)';

randNum = cast(rand(1,numPoints),classToUse);
for l = 1:numPoints
    x(1) = sqrt(gammaThreshold/diagMat(1,1))*(2*randNum(l)-1);
    for i = 2:covSize
        tau = cast(0,classToUse);
        for j = 1:i-1
            tau = tau + diagMat(j,j)*(x(j)^2);
        end
        tau  = gammaThreshold-tau;
        x(i) = sqrt(tau /diagMat(i,i))*(2*rand-1);
    end
    points(:,l) = (L*x'+trackState);
end
end

function [fState,fCov] = lowner(points,tol) % estimate lowner ellipsoid
[dim,nPoints] = size(points);
ellipsetemp = khachiyan([points;ones(1,nPoints)],tol);
ellipseparam = cast(ellipsetemp,'like',points);
% Intersect with the hyperplane where the input points lie.
A = ellipseparam(1:dim,1:dim); 
b = ellipseparam(1:dim,end);
fState = -A\b; 
fCov = A/(1 - fState'*b-ellipseparam(end));
% Force all the points to really be covered.
ac = points-repmat(fState,1,nPoints);
fCov = inv(fCov/max(dot(ac,fCov*ac,1)));
end

function lownerEllipsoid = khachiyan(points,tol)
% KHACHIYAN Approximate Lowner ellipsoid of a centrally symmetric set.
% (Ref. [3])
pts = cast(points,'double'); % This function works only for double precision
[dim, nPoints] = size(pts);
% Initialize the barycentric coordinate descent.
invA = nPoints*inv(pts*pts'); %#ok<MINV>
w = dot(pts,invA*pts,1); %(2.7)
[wmax, ~] = max(w);
epsilon = (wmax/dim)-1; %(2.18)
while epsilon>=tol
    [wmax, wind] = max(w);
    epsilon = (wmax/dim)-1; %(2.18)
    % (end of Lemma 4)
    factor1 = 1 + ((epsilon*dim)/((dim-1)*wmax));
    factor2 = ((epsilon*(dim^2))/((dim-1)*(wmax^2)));
    b = invA*pts(:,wind);  % variable b of lemma 4
    invA = factor1*invA - (factor2*b)*b';
    w = factor1*w - factor2*((b'*pts).*(b'*pts));
end
lownerEllipsoid = invA/wmax;
end