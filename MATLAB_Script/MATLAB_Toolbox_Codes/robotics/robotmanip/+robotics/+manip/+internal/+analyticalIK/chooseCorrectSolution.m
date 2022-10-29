function correctSolution = chooseCorrectSolution(solutionPair1, solutionPair2, solTolerance)
%chooseCorrectSolution Choose the solution that appears in both solutionPair1 and solutionPair2
%   This helper function is used to choose a correct solution when two
%   solutions are provided, e.g. as the result of a sum of squares. The
%   function accepts two 2-element vectors, solutionPair1 and
%   solutionPair2, which represent the solution options from the source and
%   constraint equations, respectively. The correct solution will be the
%   solution both of the source equation, as well as a constraint equation
%   for the same problem. This helper simply chooses the value that occurs
%   in both the original and constraint solutions, within a tolerance.
 
%   Copyright 2020 The MathWorks, Inc.
 
% Filter any imaginary values out of the solution pairs by replacing them
% with NaNs
realSolutionPair1 = zeros(1,2);
realSolutionPair2 = zeros(1,2);
for i = 1:2
    % Have to wrap to pi so that the two values are comparable
    realSolutionPair1(i) = robotics.internal.wrapToPi(replaceImagWithNaN(solutionPair1(i)));
    realSolutionPair2(i) = robotics.internal.wrapToPi(replaceImagWithNaN(solutionPair2(i)));
end
 
% To check equivalence, it's insufficient to just check whether the values
% are equal, because they are periodic. For example, -pi and pi are both
% valid outcomes of wrapToPi that fail a basic equality test but are
% equivalent in this context. Therefore, it's necessary to check that the
% difference of the two values, when wrapped to pi, is inside the expected tolerance.
correctSolution = nan(1,2);
for i = 1:2
    for j = 1:2
        isCorrect = abs(robotics.internal.wrapToPi(realSolutionPair1(i) - realSolutionPair2(j))) < solTolerance;
        if isCorrect
            correctSolution(i) = realSolutionPair1(i);
        end
    end
end
 
% Sort the output so that if there is one correct solution it is always in
% the first element slot
correctSolution = sort(correctSolution);
   
end
