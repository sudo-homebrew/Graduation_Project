function validateCovFusion(x,p,funcname)
%VALIDATECOVARIANCEFUSION Validate state and covariance for covariance fusion
%   validateCovarianceFusion(X,P,FUNC_NAME,VAR_NAME) validates whether the
%   input X, state and P, covariance represents valid inputs for state
%   covariance fusion function. FUNC_NAME is used in VALIDATEATTRIBUTES to
%   come up with the error id and message.
%
%   [X,P] = validateCovarianceFusion(...) outputs the validated value.

%   Copyright 2018 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>

validateattributes(x,{'single','double'},{'real', 'finite', 'nonsparse', '2d', 'nonempty'},funcname,'trackState',1);
validateattributes(p,{'single','double'},{'real', 'finite', 'nonsparse', '3d', 'nonempty'},funcname,'trackCov',2);

coder.internal.errorIf(((size(x,2) < 2)||(size(p,3) < 2)),...
    'fusion:covFusion:minTracksData','trackState','trackCov');
coder.internal.errorIf((~isequal(size(x,1),size(p,1))),...
    'fusion:covFusion:dimensionMisMatch','trackState','trackCov');
coder.internal.errorIf((~isequal(size(x,2),size(p,3))),...
    'fusion:covFusion:dataInconsistent','trackState','trackCov');

for i = 1:size(p,3)
    matlabshared.tracking.internal.isSymmetricPositiveSemiDefinite('trackCov', p(:,:,i));
end


% [EOF]