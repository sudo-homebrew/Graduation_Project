function nisCostMatrix = calculateNISCostMatrix( ...
    knownLandmarks, knownLandmarksCovariance, ...
    observedLandmarks, observedLandmarksCovariance, ...
    varargin)
    %This function is for internal use only. It may be removed in the future.

    %CALCULATENISCOSTMATRIX Calculate cost matrix using Normalized Innovation Square
    %
    %   NISCOSTMATRIX = CALCULATENISCOSTMATRIX(KNOWNLANDMARKS,  ...
    %   KNOWNLANDMARKSCOVARIANCE, OBSERVEDLANDMARKS, ...
    %   OBSERVEDLANDMARKSCOVARIANCE, SWITCHCASE) calculates a cost matrix
    %   by defining the cost of assigning a observedLandmark to a
    %   knownLandmark as the Squared Mahalanobis distance between them.
    %
    %   Input Arguments:
    %
    %   KNOWNLANDMARKS              - Known landmarks in the map, specified
    %                                 as an M-by-K matrix, where K is the
    %                                 dimension of landmarks and M is the
    %                                 number of landmarks.
    %
    %   KNOWNLANDMARKSCOVARIANCE    - Covariance of known landmarks
    %                                 specified as a K-element vector or as
    %                                 an M*K-by-M*K matrix, where K is the
    %                                 dimension of the landmarks and M is
    %                                 the number of landmarks. If specified
    %                                 as a K-element vector, then same
    %                                 covariance is assumed for all the
    %                                 landmarks.
    %
    %   OBSERVEDLANDMARKS           - Observed landmarks in the environment,
    %                                 specified by an N-by-K matrix, where
    %                                 K is the dimension of the landmark
    %                                 and N is the number of landmarks.
    %
    %
    %   OBSERVEDLANDMARKSCOVARIANCE - Covariance of observed landmarks
    %                                 specified as a K-element vector or as
    %                                 an N*K-by-N*K matrix, where K is the
    %                                 dimension of the landmarks and N is
    %                                 the number of observed landmarks.
    %                                 If specified as a K-element vector,
    %                                 then same covariance is assumed for
    %                                 all the observed landmarks.
    %
    %   SWITCHCASE                  - Tells which case is to be triggered
    %                                 for calculating the cost matrix.
    %
    %   Output Arguments:
    %
    %     NISCOSTMATRIX             - Cost matrix returned as an M-by-N
    %                                 matrix, where M is the number of known
    %                                 landmarks and N is the number of
    %                                 observed landmarks. Each entry in the
    %                                 cost matrix contains the cost of a
    %                                 known landmark and observed landmark
    %                                 association. The cost matrix will be
    %                                 a dense matrix.
    %
    %   % EXAMPLE:
    %
    %   % Specify the known landmarks and known landmark covariance
    %   knownLandmarks = [15.8495 -12.9496; 25.2455 -15.4705];
    %   knownLandmarksCovariance = 1.1 * eye(4);
    %   % Specify the observed landmarks and observed landmark covariance
    %   observedLandmarks = [15.85 -12.96;
    %                        27.44 -8.38;
    %                        25.20 -15.40;
    %                        12.44 -2.75];
    %   observedLandmarksCovariance = [1 1];
    %
    %   % Calculate the cost matrix based on Squared Mahalanobis distance
    %   nisCostMatrix = nav.calculateNISCostMatrix( ...
    %                   knownLandmarks, knownLandmarksCovariance, ...
    %                   observedLandmarks, observedLandmarksCovariance);
    %
    %   See also ekfSLAM, nav.associateMaxLikelihood, matchpairs

    %   Copyright 2021 The MathWorks, Inc.

    %#codegen

    % Check that number of arguments is 4 or 5
    narginchk(4,5);

    % decide which code block to execute based on the type of covariance
    % (same or different) of both known landmarks and observed landmarks
    %
    %                                 knownLMCovar  | observedLMCovar
    % case 0 (default/otherwise)    |   different   |   different
    % case 1                        |     same      |   different
    % case 2                        |   different   |      same
    % case 3                        |     same      |      same
    %
    % if case number is not passed then run the case of different
    % covariances
    if nargin == 5
        switchCase = varargin{1};
    end

    switch switchCase
      case 3
        % if both observed landmarks and known landmarks have same
        % covariance for each observed landmark and known landmark
        nisCostMatrix = costCase3( ...
            knownLandmarks, knownLandmarksCovariance, ...
            observedLandmarks, observedLandmarksCovariance);
      case 2
        % if observed landmarks covariance is same for all the observed
        % landmarks
        nisCostMatrix = costCase2( ...
            knownLandmarks, knownLandmarksCovariance, ...
            observedLandmarks, observedLandmarksCovariance);
      case 1
        % if known landmark covariance is same for all the known
        % landmarks
        nisCostMatrix = costCase1( ...
            knownLandmarks, knownLandmarksCovariance, ...
            observedLandmarks, observedLandmarksCovariance);
      otherwise
        % if both observed landmarks and known landmarks have different
        % covariance for each observed landmark and known landmark
        nisCostMatrix = costCase0( ...
            knownLandmarks, knownLandmarksCovariance, ...
            observedLandmarks, observedLandmarksCovariance);
    end
end

%% if both observed landmarks and known landmarks have same covariances
% for each observed landmark and known landmark
function nisCostMatrix = costCase3(knownLandmarks, knownLandmarksCovariance, ...
                                   observedLandmarks, observedLandmarksCovariance)
    % get the number of known landmarks and observed landmarks
    numObsLM = size(observedLandmarks,1);
    numKnownLM = size(knownLandmarks,1);
    % preallocate cost matrix variable
    nisCostMatrix  = zeros(numKnownLM, numObsLM, 'like', knownLandmarks);
    S = knownLandmarksCovariance + observedLandmarksCovariance;
    for j = 1:numObsLM
        % Innovation
        del = repmat(observedLandmarks(j, :), numKnownLM, 1) - knownLandmarks;
        % Calculate Normalized Innovation Square
        nisCostMatrix(:,j) = diag(del/S*del');
    end
end

%% if observed landmarks covariance is same for all the observed landmarks
function nisCostMatrix = costCase2(knownLandmarks, knownLandmarksCovariance, ...
                                   observedLandmarks, observedLandmarksCovariance)
    % get the number of known landmarks and observed landmarks
    numObsLM = size(observedLandmarks,1);
    numKnownLM = size(knownLandmarks,1);
    dimension = size(knownLandmarks,2);
    % preallocate cost matrix variable
    nisCostMatrix  = zeros(numKnownLM, numObsLM, 'like', knownLandmarks);
    for k = 1:numKnownLM
        index = k*dimension + (-dimension+1:0);
        % Innovation covariance
        S = knownLandmarksCovariance(index,index) + observedLandmarksCovariance;
        % Innovation
        del = observedLandmarks - repmat(knownLandmarks(k, :), numObsLM, 1);
        % Calculate Normalized Innovation Square
        nisCostMatrix(k,:) = diag(del/S*del');
    end
end

%% if known landmark covariance is same for all the known landmarks
function nisCostMatrix = costCase1(knownLandmarks, knownLandmarksCovariance, ...
                                   observedLandmarks, observedLandmarksCovariance)
    % get the number of known landmarks and observed landmarks
    numObsLM = size(observedLandmarks,1);
    numKnownLM = size(knownLandmarks,1);
    dimension = size(knownLandmarks,2);
    % preallocate cost matrix variable
    nisCostMatrix  = zeros(numKnownLM, numObsLM, 'like', knownLandmarks);
    for j = 1:numObsLM
        index = j*dimension + (-dimension+1:0);
        % Innovation covariance
        S = knownLandmarksCovariance + observedLandmarksCovariance(index, index);
        % Innovation
        del = repmat(observedLandmarks(j, :), numKnownLM, 1) - knownLandmarks;
        % Calculate Normalized Innovation Square
        nisCostMatrix(:,j) = diag(del/S*del');
    end
end

%% if both observed landmarks and known landmarks have different
% covariance for each observed landmark and known landmark
function nisCostMatrix = costCase0(knownLandmarks, knownLandmarksCovariance, ...
                                   observedLandmarks, observedLandmarksCovariance)
    % get the number of known landmarks and observed landmarks
    numObsLM = size(observedLandmarks,1);
    numKnownLM = size(knownLandmarks,1);
    dimension = size(knownLandmarks,2);
    % preallocate cost matrix variable
    nisCostMatrix  = zeros(numKnownLM, numObsLM, 'like', knownLandmarks);
    for j = 1:numObsLM
        % get the covariance of current observedLandmark
        index = j*dimension + (-dimension+1:0);
        obsLMCov = observedLandmarksCovariance(index, index);
        for k = 1:numKnownLM
            index = k*dimension + (-dimension+1:0);
            % Innovation
            del = observedLandmarks(j, :) - knownLandmarks(k, :);
            % Innovation covariance
            S = knownLandmarksCovariance(index, index) + obsLMCov;
            % Calculate Normalized Innovation Square
            nisCostMatrix(k,j) = del/S*del';
        end
    end
end