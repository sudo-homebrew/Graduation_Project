function obj = hinfsynOptions(varargin)
%HINFSYNOPTIONS  Creates option set for the HINFSYN command.
%
%   OPT = HINFSYNOPTIONS returns the default options for HINFSYN.
%
%   OPT = HINFSYNOPTIONS('Option1',Value1,'Option2',Value2,...) uses 
%   name/value pairs to override the default values for 
%   'Option1','Option2',...
%
%   The main options are:
%
%   Display     Show progress and print a report [{'off'} | 'on'].
%
%   Method      Optimization algorithm [{'RIC'} | 'LMI' | 'MAXE'].
%               HINFSYN can use a Riccati-based or an LMI-based algorithm
%               for optimizing the closed-loop performance. The default 
%               Riccati method 'RIC' is faster but can't handle singular 
%               problems without first adding extra disturbances and errors 
%               (a process called "regularization"). The 'LMI' method needs 
%               no regularization but is computationally more intensive. 
%               'MAXE' is similar to 'RIC' but returns the max-entropy
%               solution.
%
%   RelTol      Relative accuracy on optimal HINF performance
%               (default=1e-2).
%
%   The following options are specific to Method='RIC' (Riccati approach):
%
%   AbsTol      Absolute accuracy on optimal HINF performance
%               (default=1e-6).
%
%   AutoScale   Automatic plant scaling [{'on'} | 'off']. 
%               HINFSYN automatically scales the plant states, controls, 
%               and measurements to improve numerical accuracy. The 
%               controller K is always returned in the original
%               coordinates. Set AutoScale='off' to skip this step when
%               your plant is known to be well scaled.
%
%   Regularize  Automatic regularization [{'on'} | 'off']. 
%               HINFSYN automatically regularizes the plant to enforce the 
%               requirements on P12 and P21. Set Regularize='off' to skip 
%               this step when your problem is far enough from singular.
%
%   LimitGain   Limit controller gains [{'on'} | 'off']. 
%               In continuous time, near-singular D12 or D21 can result in
%               controllers with large coefficients and fast dynamics. Use
%               this option to automatically seek a controller with the 
%               same performance but lower gains and better conditioning.
%
%   The following options are specific to Method='LMI' (LMI approach):
%
%   LimitRS     Value between 0 and 1 (default=0). 
%               Increase this value to slow the controller dynamics by
%               penalizing large-norm LMI solutions R,S.
%
%   TolRS       Reduced-order synthesis tolerance (default=1e-3).
%               HINFSYN computes a reduced-order controller when
%                  1 <= rho(R*S) <= 1+TOLRS
%
%   The following option is specific to Method='MAXE':
%
%   S0          Frequency S0 at which entropy is evaluated (default=Inf). 
%
%
%   Example: Compute the optimal H-infinity performance with 3-digit 
%   accuracy using the Riccati approach. Turn off automatic scaling and 
%   regularization:
%      opt = hinfsynOptions('Method','RIC','RelTol',1e-3,...
%                           'AutoScale','off','Regularize','off');
%      [K,~,gamOpt] = hinfsyn(P,ny,nu,opt)
%
%   See also HINFSYN.


%   RegLevel    Regularization level (default = 1e-12).
%               Relative size of perturbation used to enforce requirements
%               on P12 and P21 when Method='RIC'. Set RegLevel=0 to skip
%               regularization. Decreasing RegLevel improves the optimal 
%               performance but may cause the Riccati solvers to fail.
%               Increasing RegLevel improves numerical stability but may
%               sharply degrade performance.

%   Copyright 2017 The MathWorks, Inc.
try
   obj = initOptions(rctoptions.hinfsyn,varargin);
catch E
   throw(E)
end
