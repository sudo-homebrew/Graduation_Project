function varargout = cpmargin(P,C,varargin)
%CPMARGIN  Obsolete, see NCFMARGIN.

%   Copyright 2011 The MathWorks, Inc.

%    [MARG,FREQ] = CPMARGIN(P,C) calculates the normalized coprime
%    factor/gap metric robust stability of the multivariable feedback
%    loop consisting of C in negative feedback with P.  C should only
%    be the compensator in the feedback path, not any reference channels,
%    if it is a "2-dof" architecture.  The output MARG contains upper
%    and lower bound for the normalized coprime factor/gap metric robust
%    stability margin. FREQ is the frequency associated with the upper
%    bound.
%
%    [MARG,FREQ] = CPMARGIN(P,C,TOL)  specifies a relative accuracy TOL
%    for calculating the normalized coprime factor/gap metric robust
%    stability margin. (TOL=1e-3 by default).
try
   [varargout{1:nargout}] = ncfmargin(P,C,-1,varargin{:});
catch ME
   throw(ME)
end