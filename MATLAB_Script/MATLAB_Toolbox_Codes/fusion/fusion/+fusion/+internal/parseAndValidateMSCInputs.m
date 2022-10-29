function [stateCol,vNoiseOut,dT,obsManeuver,hasZStates,isRow] = parseAndValidateMSCInputs(state,vNoise,funcName,varargin)
% This is an internal function and may be removed in a future release.
% parseAndValidateMSCInputs - parses and validates the inputs to the MSC
% state transition functions. funcName decides if parsing is performed for
% StateTransition or StateTransitionJacobianFcn.

% Copyright 2018 The MathWorks, Inc.

%#codegen

% Validate state input. For jacobian only single state is allowed.
if strcmpi(funcName,'constvelmscjac')
    validateattributes(state, {'single','double'},...
    {'real', 'finite', 'vector', 'nonsparse'}, funcName, 'state', 1);
    cond = numel(state)==6 || numel(state)==4;
    coder.internal.errorIf(~cond,'shared_tracking:motion:incorrectStateVecWithInfo','[4 6]');
    stateInCol = state(:);
    % Specify this output for codegen.
    isRow = true;
else
    validateattributes(state, {'single','double'},...
    {'real', 'finite', '2d', 'nonsparse'}, funcName, 'state', 1);
    % Check for row or column Orientation
    [rowSize,colSize] = size(state);
    isRowOrient = (colSize == 4) || (colSize == 6);
    isColOrient = (rowSize == 4) || (rowSize == 6);
    % Only column orientation is supported for vectorized state.
    cond = (~isRowOrient || ~isvector(state)) && ~isColOrient;
    coder.internal.errorIf(cond,'shared_tracking:motion:incorrectStateSizeWithInfo','[4 6]');
    if ~isColOrient
        stateInCol = state';
    else
        stateInCol = state;
    end
    isRow = ~isColOrient;
end
classToUse = class(state);
stateDim = size(stateInCol,1)/2;
hasZStates = stateDim == 3;

% Validate noise input. For jacobian only a vector is allowed.
if strcmpi(funcName,'constvelmscjac')
    validateattributes(vNoise,{classToUse},...
        {'real','finite','vector','nonsparse'},funcName,'vNoise',2);
else
    validateattributes(vNoise, {classToUse},...
    {'real', 'finite', '2d', 'nonsparse'}, funcName, 'vNoise', 2);
end

% vNoise may be given as a scalar. In that case, scalar expand it to the
% correct dimensions expected by the function.
if isscalar(vNoise)
    vNoiseExp = vNoise(1,1) * ones(stateDim,size(stateInCol,2),'like',vNoise);
else
    vNoiseExp = vNoise;
end

% Common branch for Jacobian and Transition when state is a vector, turn
% vNoise into a column orientation and make sure the size matches to 1/2
% number of states.
if isvector(stateInCol)
    cond = numel(stateInCol)/2 == numel(vNoiseExp);
    coder.internal.errorIf(~cond,'fusion:MSC:invalidNoiseInputVecSize',numel(stateInCol)/2);
    vNoiseInCol = vNoiseExp(:);
else
    % If matrix, check that the orientation is same as state. This is only
    % reachable if state was column oriented. Here we make sure that noise
    % must be column oriented as well.
    cond = size(vNoiseExp,1) == stateDim && size(vNoiseExp,2) == size(state,2);
    coder.internal.errorIf(~cond,'fusion:MSC:invalidNoiseInputSize',sprintf('%d-by-%d',int32(stateDim),int32(size(state,2))));
    vNoiseInCol = vNoiseExp;
end

% Time interval, dT
if numel(varargin) > 0
    validateattributes(varargin{1}, {'numeric'}, ...
        {'real', 'finite', 'scalar'}, funcName, 'dt', 3);
    dT = cast(varargin{1},'like',state);
else
    dT = ones(1,'like',state);
end

% Observer input is ALWAYS a vector. Hence, it should be validated same for jacobian
% and state transition.
if numel(varargin) > 1
    validateattributes(varargin{2},{classToUse},...
        {'real','finite','nonsparse','vector'},funcName,'u',4);
    cond = numel(varargin{2}) == stateDim || numel(varargin{2}) == 2*stateDim;
    coder.internal.errorIf(~cond,'fusion:MSC:invalidObserverInputSize',stateDim,2*stateDim);
    isInputAccel = numel(varargin{2}) == stateDim;
    if isInputAccel
        G1d = [dT^2/2;dT];
        % Create B without blkdiag for faster validations
        if stateDim == 2
            B = [G1d zeros(2,1);zeros(2,1) G1d];
            %B = blkdiag(G1d,G1d);
        else
            B = [G1d zeros(2,2);zeros(4,1) [G1d;zeros(2,1)] [zeros(2,1);G1d]];
            %B = blkdiag(G1d,G1d,G1d);
        end
        u = varargin{2}(:);
        obsManeuver = B*u;
    else
        obsManeuver = varargin{2}(:);
    end
else
    obsManeuver = zeros(2*stateDim,1,'like',state);
end

% Bring the state to a full 3-D convention, as the calculations are
% performed on the full vector.
if ~hasZStates
    zeroState = zeros(1,size(stateInCol,2),'like',stateInCol);
    stateCol = [stateInCol(1,:);stateInCol(2,:);zeroState;zeroState;stateInCol(3,:);stateInCol(4,:)];
    vNoiseOut = [vNoiseInCol;zeroState];
    % Pad manuever with just 0 as it's not a matrix.
    obsManeuver = [obsManeuver;0;0];
else
    vNoiseOut = vNoiseInCol;
    stateCol = stateInCol;
end



    



