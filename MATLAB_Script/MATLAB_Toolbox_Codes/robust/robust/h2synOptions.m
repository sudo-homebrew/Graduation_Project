function obj = h2synOptions(varargin)
%H2SYNOPTIONS  Creates option set for the H2SYN command.
%
%   OPT = H2SYNOPTIONS returns the default options for H2SYN.
%
%   OPT = H2SYNOPTIONS('Option1',Value1,'Option2',Value2,...) uses 
%   name/value pairs to override the default values for 
%   'Option1','Option2',...
%
%   The main options are:
%
%   AutoScale   Automatic plant scaling [{'on'} | 'off']. 
%               H2SYN automatically scales the plant states, controls, 
%               and measurements to improve numerical accuracy. The 
%               controller K is always returned in the original
%               coordinates. Set AutoScale='off' to skip this step when
%               your plant is known to be well scaled.
%
%   Regularize  Automatic regularization [{'on'} | 'off']. 
%               H2SYN automatically regularizes the plant to enforce the 
%               requirements on P12 and P21. Set Regularize='off' to skip 
%               this step when your problem is far enough from singular.
%
%   Example: Turn off automatic scaling and regularization to speed up
%   computation of the H2-optimal controller:
%      opt = h2synOptions('AutoScale','off','Regularize','off');
%      [K,~,gam] = h2syn(P,ny,nu,opt)
%
%   See also H2SYN.

%   Copyright 2018 The MathWorks, Inc.
try
   obj = initOptions(rctoptions.h2syn,varargin);
catch E
   throw(E)
end
