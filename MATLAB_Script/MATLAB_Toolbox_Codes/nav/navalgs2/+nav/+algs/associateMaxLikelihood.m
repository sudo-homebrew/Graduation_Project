function [associations, newLandmarks] = associateMaxLikelihood( ...
    knownLandmarks, knownLandmarksCovariance, ...
    observedLandmarks, observedLandmarksCovariance, varargin)
    %ASSOCIATEMAXLIKELIHOOD Associate observed landmarks to known landmarks
    %   [ASSOCIATIONS, NEWLANDMARK] = ASSOCIATEMAXLIKELIHOOD( ...
    %   KNOWNLANDMARKS, KNOWNLANDMARKSCOVARIANCE, ...
    %   OBSERVEDLANDMARKS, OBSERVEDLANDMARKSCOVARIANCE) associates observed
    %   landmarks to known landmarks using the Maximum likelihood
    %   algorithm. The function assumes the value of validation gate as
    %   [5.991 5.991].
    %
    %   [ASSOCIATIONS, NEWLANDMARK] = ASSOCIATEMAXLIKELIHOOD( ...
    %   KNOWNLANDMARKS, KNOWNLANDMARKSCOVARIANCE, ...
    %   OBSERVEDLANDMARKS, OBSERVEDLANDMARKSCOVARIANCE, VALIDATIONGATE)
    %   associates observed landmarks to known landmarks using the Maximum
    %   likelihood algorithm and the validation gate, VALIDATIONGATE.
    %
    %   Input Arguments:
    %
    %   KNOWNLANDMARKS              - Known landmarks in the map,
    %                                 specified as an M-by-K matrix, where
    %                                 K is the dimension of landmarks and M
    %                                 is the number of landmarks.
    %
    %   KNOWNLANDMARKSCOVARIANCE    - Covariance of known landmarks
    %                                 specified as a K-element vector or as
    %                                 an M*K-by-M*K matrix, where K is the
    %                                 dimension of the landmarks and M is
    %                                 the number of landmarks. If specified
    %                                 as a K-element vector, then same
    %                                 covariance is assumed for all the
    %                                 known landmarks.
    %
    %   OBSERVEDLANDMARKS           - Observed landmarks in the environment,
    %                                 specified as an N-by-K matrix, where
    %                                 K is the dimension of the landmarks
    %                                 and N is the number of landmarks.
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
    %   VALIDATIONGATE              - Region of acceptance of true landmarks
    %                                 from observedLandmarks specified as a
    %                                 scalar or two-element vector of form
    %                                 [VALGATEASSOCIATION VALGATENEWLM].
    %                                 The value of VALGATEASSOCIATION must
    %                                 be less than or equal to VALGATENEWLM.
    %                                 VALGATEASSOCIATION specifies the
    %                                 maximum threshold for association.
    %                                 VALGATENEWLM specifies the minimum
    %                                 threshold for creation of new
    %                                 landmarks. New landmarks are the
    %                                 observerLandmarks that qualified as
    %                                 true landmarks.
    %                                 
    %                                 Notes:
    %                                 1. The default value of VALGATEASSOCIATION 
    %                                    is 5.991 from chi-square distribution
    %                                    for 95% correct association.
    %                                 2. If VALGATENEWLM is not specified, 
    %                                    then the value of VALGATEASSOCIATION
    %                                    is used instead.
    %
    %   Output Arguments:
    %
    %     ASSOCIATIONS              - List of associations of known
    %                                 landmarks to observed landmarks, 
    %                                 returned as a P-by-2 matrix. P is 
    %                                 the number of associations. The
    %                                 first column of the matrix contains
    %                                 the indices of associated known
    %                                 landmarks and the second column
    %                                 contains the indices of observed
    %                                 landmarks.
    %
    %     NEWLANDMARKS              - List of indices of observed
    %                                 landmarks that qualified as new
    %                                 landmark, i.e., those are known with
    %                                 surety. It is returned as a Q-element
    %                                 vector. Q is the number of observed
    %                                 landmarks that qualified as a new
    %                                 landmark.
    %                                 Note: If there is no known landmark, 
    %                                 then all the observed landmarks are
    %                                 returned as NEWLANDMARKS.
    %
    %   Either single or double datatypes are supported for the inputs to
    %   ASSOCIATEMAXLIKELIHOOD. Outputs have the same datatype as the
    %   knownLandmarks.
    %
    %   % EXAMPLE:
    %
    %   % Specify the known landmarks and known landmarks covariance
    %   knownLandmarks = [15.8495 -12.9496; 25.2455 -15.4705];
    %   knownLandmarksCovariance = 1.1 * eye(4);
    %   % Specify the observed landmarks and observed landmarks covariance
    %   observedLandmarks = [15.85 -12.96;
    %                        27.44 -8.38;
    %                        25.20 -15.40;
    %                        12.44 -2.75];
    %   observedLandmarksCovariance = [1 1];
    %
    %   % Calculate the associations with the default validationGate value
    %   [associations, newLandmarks] = nav.algs.associateMaxLikelihood( ...
    %           knownLandmarks, knownLandmarksCovariance, ...
    %           observedLandmarks, observedLandmarksCovariance);
    %
    %   % Calculate the associations with the specified validationGate value
    %   [associations, newLandmarks] = nav.algs.associateMaxLikelihood( ...
    %           knownLandmarks, knownLandmarksCovariance, ...
    %           observedLandmarks, observedLandmarksCovariance, ...
    %           [4.605, 40]);
    %
    %   See also: matchpairs, ekfSLAM

    %   Copyright 2021 The MathWorks, Inc.

    %#codegen

    % Check that number of arguments is 4 or 5
    narginchk(4,5);

    % validate observed landmarks and observed landmark covariance
    [observedLandmarksOp, observedLandmarkCovarianceOp, ...
     numMeas, flagSameObsLMCovar] = validateExpandLandmark( ...
         observedLandmarks, observedLandmarksCovariance, ...
         'observedLandmarks');

    % if there is no known landmark then return all the indices as
    % newLandmarks
    if isempty(knownLandmarks)
        associations = zeros([1,0],'like',observedLandmarksOp);
        newLandmarks = cast((1:numMeas)','like',observedLandmarksOp);
        return
    end

    % Check that the dimension of observed landmarks and known landmarks
    % is same
    coder.internal.errorIf((size(knownLandmarks,2) ~= size(observedLandmarks,2)), ...
                           'nav:navalgs:associatemaxlikelihood:LMMeasDimensionMismatch');

    % validate known landmarks and known landmark covariance
    [knownLandmarksOp, knownLandmarkCovarianceOp, ...
     ~, flagSameKnownLMCovar] = validateExpandLandmark( ...
         knownLandmarks, knownLandmarksCovariance, ...
         'knownLandmarks');

    % Data type would be determined by the knownLandmark argument.
    % Rules for data classes:
    % * The input arguments can be single or double. Mixture of the data
    %   types is allowed, but all the values will be converted to the
    %   class of landmark.
    % * The output has the same class as knownLandmarks.

    % Cast other arguments to knownLandmarks class
    knownLandmarkCovarianceOp    = ...
            cast(knownLandmarkCovarianceOp, 'like', knownLandmarks);
    observedLandmarksOp          = ...
            cast(observedLandmarksOp, 'like', knownLandmarks);
    observedLandmarkCovarianceOp = ...
            cast(observedLandmarkCovarianceOp, 'like', knownLandmarks);

    % switch case
    swCase = flagSameObsLMCovar * 2 + flagSameKnownLMCovar;

    % calculate the cost matrix
    nisMatrix = nav.algs.internal.ekfSLAM.calculateNISCostMatrix( ...
        knownLandmarksOp, knownLandmarkCovarianceOp, ...
        observedLandmarksOp, observedLandmarkCovarianceOp, swCase);

    % considering first varargin as validation gate values
    if nargin == 5
        validateattributes(varargin{1}, {'single', 'double'}, ...
                           {'real', 'nonnan', 'nonempty', 'finite', ...
                            'positive', 'vector'}, ...
                           'nav.algs.associateMaxLikelihood', ...
                           'validationGate');
        if numel(varargin{1}) == 1
            validationGate = [varargin{1}, varargin{1}];
        elseif numel(varargin{1}) == 2
            coder.internal.errorIf((varargin{1,1}(1) > varargin{1,1}(2)), ...
                                   'nav:navalgs:associatemaxlikelihood:lessValGateNewLM');
            validationGate = varargin{1};
        else
            coder.internal.error( ...
                'nav:navalgs:associatemaxlikelihood:InvalidNumElements', ...
                'validationGate');
        end
    else
        validationGate = [5.991, 5.991];
    end

    % Cast validationGate
    validationGate = cast(validationGate, 'like', knownLandmarks);

    % get the associations and the remaining observed landmarks
    [associations, ~, newLandmarks] = ...
        matchpairs(nisMatrix, validationGate(1)/2);
    % cast associations and newLandmarks like knownLandmarks
    associations = cast(associations, 'like', knownLandmarks);
    newLandmarks = cast(newLandmarks, 'like', knownLandmarks);

    % check for outliers in newLandmarks
    if ~isempty(newLandmarks)
        bestNIS = min(nisMatrix, [], 1);
        remMeasNIS = bestNIS(newLandmarks);
        newLandmarks = newLandmarks(remMeasNIS > validationGate(2));
    end
end

%% known landmarks and observed landmarks validation
function [position, posCovar, ...
          numEle, flagSameNoise] = validateExpandLandmark( ...
              position, positionCovariance, varName)
    validateattributes(position, {'single', 'double'}, ...
                       {'real', 'nonnan','finite', '2d', 'nonempty'}, ...
                       'nav.algs.associateMaxLikelihood', ...
                       varName);

    [numEle, dimension] = size(position);
    sizeCovar = numEle*dimension;

    varCovName = [varName 'Covariance'];

    % Check the condition for size of covariance
    covSizeCond = (numel(positionCovariance) == dimension) || ...
        all(size(positionCovariance) == [sizeCovar, sizeCovar]);
    % Throw error if covariance is not in expected format
    coder.internal.errorIf(~covSizeCond, ...
                           'nav:navalgs:associatemaxlikelihood:InvalidCovarianceMatrix', ...
                           varCovName, dimension, sizeCovar);

    % Validate covariance matrix
    if numel(positionCovariance) == dimension
        validateattributes(positionCovariance, ...
                           {'single', 'double'}, ...
                           {'real', 'nonnan', 'finite', 'nonempty', ...
                            '2d', 'nonsparse'}, ...
                           'nav.algs.associateMaxLikelihood', ...
                           varCovName);
        posCovar = diag(positionCovariance);
        flagSameNoise = true;
    elseif all(size(positionCovariance) == [sizeCovar, sizeCovar])
        validateattributes(positionCovariance, ...
                           {'single', 'double'}, ...
                           {'real', 'nonnan', 'finite', 'nonempty', ...
                            '2d', 'size', [sizeCovar, sizeCovar], ...
                            'nonsparse'}, ...
                           'nav.algs.associateMaxLikelihood', ...
                           varCovName);
        posCovar = positionCovariance;
        flagSameNoise = false;
    else
        posCovar = positionCovariance;
        flagSameNoise = false;
    end
end
