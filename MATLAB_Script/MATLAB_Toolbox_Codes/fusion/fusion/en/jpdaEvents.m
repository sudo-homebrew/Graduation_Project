%jpdaEvents  Generate feasible joint events for trackerJPDA
% FJE = jpdaEvents(validationMatrix) uses depth first search algorithm to
% generate all the feasible joint event matrices, FJE, corresponding to the
% validation matrix, validationMatrix. validationMatrix is a m-by-n+1
% binary numeric or logical matrix, validationMatrix(i,j+1) is true if the
% i-th measurement can be associated with the j-th track. The first column
% represents the null track (clutter) and must be a column of true values.
% FJE is a m-by-n+1-by-p array of feasible joint events, where each page is
% a logical matrix of size m-by-n+1 and p is the total number of feasible
% joint events.
%
% [FJE, FJEProbs] = jpdaEvents(likelihoodMatrix, k) uses Murty's algorithm
% to generate the k-best feasible joint event matrices, FJE, corresponding
% to the posterior likelihood matrix, likelihoodMatrix. likelihoodMatrix is
% a m+1-by-n+1 matrix, likelihoodMatrix(i+1,j+1) defines the posterior
% likelihood of associating ith detection with jth track. The first column
% represents the null track and the first row represents the null
% measurement. The association of a detection with null track represents a
% clutter measurement and the association of a track with null measurement
% represents an unassigned track.
% FJE is a m-by-n+1-by-p array of feasible joint events, where each page is
% a logical matrix of size m-by-n+1 and p is the number of generated
% events, which is less than or equal to k.
% FJEProbs is a p-element vector of normalized event probabilities. The sum
% of FJEProbs is equal to 1.
%
% A feasible joint event is defined as an association of detection with a
% track such that:
%   i) a measurement is associated with at most one track or the clutter. 
%   ii) a track can be associated with one measurement at most.
%
% Example: 
% % Define an arbitrary validation matrix for 5 measurements and 6 tracks.
% M = [1     1     1     1     1     0     1
%      1     0     1     1     0     0     0
%      1     0     0     0     1     1     0
%      1     1     1     1     0     0     0
%      1     1     1     1     1     1     1]
%
% % Generate all feasible joint events 
% FJE = jpdaEvents(M); 
% nFJE = size(FJE,3);
% disp([num2str(nFJE) ' feasible joint event matrices were generated'])
%
% % Look at a few feasible joint event matrices
% toSee = [1:round(nFJE/5):nFJE, nFJE];
% for ii = toSee
%     disp("Feasible joint event matrix #" + ii + ":")
%     disp(FJE(:,:,ii))
% end
%
% See also: trackerJPDA

 
%   Copyright 2018-2019 The MathWorks, Inc.

