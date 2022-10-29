function obj = robuststabOptions(varargin)
%ROBUSTSTABOPTIONS  Creates option set for the ROBUSTSTAB command.
%
%   OPT = ROBUSTSTABOPTIONS returns the default options for ROBUSTSTAB.
%
%   OPT = ROBUSTSTABOPTIONS('Option1',Value1,'Option2',Value2,...) uses 
%   name/value pairs to override the default values for 'Option1','Option2',...
%
%   Supported options include:
%
%   Display          Show progress of MUSSV computations [{'off'} | 'on']
%                    Note that this option overrides silent ('s') option in
%                    MUSSV options argument below.
%
%   Sensitivity      Sensitivity of stability margin [{'on'} | 'off']
%                    Each uncertain element contributes, in a coupled
%                    manner, to the overall stability margin.  If this
%                    option is set to 'on', the sensitivity of the margin
%                    is computed with respect to each individual uncertain
%                    element, giving an indication of which elements are 
%                    most problematic.
%
%   VaryUncertainty  Percentage variation of uncertainty for sensitivity
%                    calculations (sensitivity estimate uses a finite
%                    difference calculation). The default value is 25 (%).
%
%   Mussv            Option string for MUSSV. Default is '', resulting
%                    in the default behavior of MUSSV.
%
%   Example:
%      sys = feedback(tf(1,[1 0])*(1+0.5*ultidyn('Delta',[1 1])),1);
%      opt = robuststabOptions('Display','on','Sensitivity','off');
%      [SM,DestabU,Report,Info] = robuststab(sys,opt);
%      Info.Sensitivity      % NaN, since it was not computed
%
%   See also ROBUSTSTAB.

%   Author(s): MUSYN
%   Copyright 1984-2011 The MathWorks, Inc.
try
   obj = initOptions(rctoptions.robuststab,varargin);
catch E
   throw(E)
end
