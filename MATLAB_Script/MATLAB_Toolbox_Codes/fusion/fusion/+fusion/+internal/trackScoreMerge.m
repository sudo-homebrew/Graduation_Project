function score = trackScoreMerge(mainScore, secondaryScore)
%trackScoreMerge  Provides the new track score after track merge
%   newScore = trackScoreMerge(mainScore, secondaryScore) returns the new
%   score, newScore, based on the score of the main track, mainScore, and
%   the score of the merged track, secondaryScore.
%
%   Inputs:
%       mainScore   - a real scalar for the main track's score.
%       mergedScore - a real scalar for the merged track's score.
%
%   Outputs: 
%       score - updated score of the main track, after merging.
%
%   Example:
%   --------
%       mainScore = 20;
%       mergedScore = 15;
%       score = trackScoreMerge(mainScore, mergedScore)

% References:
% [1] Samuel Blackman and Robert Popoli, "Design and Analysis of Modern
% Tracking Systems", Artech House, 1999.

%   Copyright 2017 The MathWorks, Inc.

%#codegen
narginchk(2,2)
validateattributes(mainScore,{'numeric'}, {'real','finite','scalar'}, mfilename, 'mainScore')
validateattributes(secondaryScore,{'numeric'}, {'real','finite','scalar'}, mfilename, 'secondaryScore')
delScore = mainScore - cast(secondaryScore,'like',mainScore);
score = mainScore + log(1+exp(-delScore));