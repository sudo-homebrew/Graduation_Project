function obj = wcmarginOptions(varargin)
%WCMARGINOPTIONS  Creates option set for the WCMARGIN command.
%
%   OPT = WCMARGINOPTIONS returns the default options for WCMARGIN.
%
%   OPT = WCMARGINOPTIONS('Option1',Value1,'Option2',Value2,...) uses 
%   name/value pairs to override the default values for 'Option1','Option2',...
%
%   Supported options include:
%
%   Sensitivity       Sensitivity of worst-case gain ['on' | {'off'}].
%                     Each uncertain element contributes, in a coupled manner, 
%                     to the overall worst-case margin. When this option is  
%                     set to 'on', the sensitivity of the worst-case margin  
%                     is computed with respect to each individual uncertain
%                     element, giving an indication of which elements are  
%                     most problematic.
%
%   AbsTol, RelTol    Absolute and relative tolerances on computed worst-case 
%                     margin bounds. The algorithm terminates when 
%                       UpperBound-LowerBound <= max(AbsTol,Reltol*UpperBound).
%                     The default values are 0.02 and 0.05.
%
%   See also WCMARGIN, WCGAINOPTIONS.

%   Copyright 1984-2011 The MathWorks, Inc.
try
   obj = initOptions(rctoptions.wcmargin,varargin);
catch E
   throw(E)
end
