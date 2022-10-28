function M = simplify(M,varargin)
%SIMPLIFY  Simplifies uncertainty model.
%
%   B = SIMPLIFY(A) uses model-reduction-like techniques to detect and 
%   eliminate redundant copies of uncertain elements. The AutoSimplify 
%   property of each uncertain element in A determines what reduction 
%   methods are used.  After reduction, any uncertain element which does 
%   not actually affect the result is deleted from the model.
%
%   B = SIMPLIFY(A,'full') ignores the AutoSimplify settings and uses 
%   'full' reduction techniques.
%
%   B = SIMPLIFY(A,'basic') ignores the AutoSimplify settings and uses 
%   'basic' reduction techniques.
%
%   See also UMAT, USS, UFRD, UncertainBlock.

%   Author(s): MUSYN
%   Copyright 2004-2011 The MathWorks, Inc.
if isUncertain(M)
   if nargin>1
      LevelOpt = ltipack.matchKey(varargin{1},{'basic','full','class'});
      if strcmp(LevelOpt,'class')
         % Nothing to do
         return
      else
         LevelOverride = find(strcmp(LevelOpt,{'basic';'full'})) + 1;
      end
   else
      LevelOverride = -1;
   end
   MaxLevel = Inf;
   try
      M = simplify_(M,LevelOverride,MaxLevel);
   catch ME
      throw(ME)
   end
   % Return numeric or LTI array if there are no more blocks
   if isBlockFree(M)
      M = getValue(M);
   end
end