function [k,gam]=sdhinfsyn(varargin)
%SDHINFSYN  H-infinity controller synthesis for sampled-data system.
%
% [K,GAM]=SDHINFSYN(P,NMEAS,NCON) or
% [K,GAM]=SDHINFSYN(P,NMEAS,NCON, KEY1,VALUE1,KEY2,VALUE2,...)
%    Computes the discrete-time sampled-data H-infinity output-feedback
%    controller K for a continuous-time strictly proper LTI plant
%    P.  Using results from Bamieh and Pearson '92, the initial
%    problem is converted isomorphically to discrete-time.  The discrete-
%    time problem is bilinearly mapped to continuous-time where HINFSYN
%    is used to compute the minimal cost GAM in [GMIN,GMAX] for
%    which the closed-loop system G= LFT(P,K) satisfies
%                   HINFNORM(G) < GAM.
%    NMEAS is the number of measurement outputs from the plant and NCONT is
%    the number of control inputs to the plant; these may be omitted if
%    P=MKTITO(P,NMEAS,NCONT) or P=AUGW(SYS,W1,W2,W3).  The plant
%    P must be strictly proper (i.e., P.D must be zero) or an error
%    results.
%
%    inputs:
%       P   - an LTI system, with partitioning determined by
%       NMEAS  - the number of measurement outputs from the plant
%       NCONT  - the number of control inputs to the plant
%       NMEAS and NCONT may be omitted if P=MKTITO(P,NMEAS,NCONT).
%
%    Options are similar to HINFSYN, but adding 'Ts' and 'DELAY':
%      KEY      |VALUE   | MEANING
%     ----------------------------------------------------------------
%      'GMAX'   | real   | (Inf default) initial upper bound on GAM
%      'GMIN'   | real   | (0 default) initial lower bound on GAM
%      'TOLGAM' | real   | relative error tolerance for GAM (.01 default)
%      'Ts'     | real   | (1 default) sampling period of the controller
%               |        |    to be designed
%      'DELAY'  | real   | (0 default) a non-negative integer giving
%               |        |    the number of sample periods delays for the
%               |        |    control computation
%      'DISPLAY'|'on/off'| display synthesis information to screen,
%               |        |  (default = 'off')
%     -------------------------------------------------------------------
%     outputs:
%        K     -   H-infinity controller
%        GAM   -   H-infinity cost
% 	                   _________
%  	               |         |
%  	       <-------|  sdsys  |<--------
%  	               |         |
%  	      /--------|_________|<------\
%  	      |       __  		   |
%  	      |      |d |		         |
%  	      |  __  |e |   ___    __    |
%  	      |_|S |_|l |__| K |__|H |___|
%  	        |__| |a |  |___|  |__|
%  	             |y |
%  	             |__|
%
%    See also: HINFSYN, LTI/NORM, SDHINFNORM

%   Copyright 2003-2010 The MathWorks, Inc.

% LTI/SDHINFSYN
% M. G. Safonov 01/09/2003
[varargin{:}] = convertStringsToChars(varargin{:});
nag1=nargin;

% initialize
DocKeys={'GMAX ','GMIN ','TOLGAM ', 'TS ','DELAY ','DISPLAY '};
UndocKeys={'RICMETHOD ','EPR ','EPP '};
gmax=1e6;
gmin=sqrt(eps);
tol=.01; %tolgam
h=1;
delay=0;
k=ss;
gam=[];
verbose=0;      % no printing from hinfsyne
quiet=verbose;  % no printing from hinfsyne
ricmethd=0;   %  passed sub-sub-routine rctutil/hinf_gam; ricmethd=0 --> ric_gcare.m -->gcare.m
epr = 1e-10;  % default tol for when real(eig) of Hamiltonian is zero
epp = 1e-6;   % default tol for positive definiteness determination for Riccati solution

%  no input --> show usage
if ~nag1
   disp(getString(message('Robust:design:sdhinfsyn1')));
   return
end

% get plant
plant=varargin{1};
try
   plant=ss(plant);
catch
   error(message('Robust:design:sdhinfsyn2'));
end
% ns=size(get(plant,'a'),1); % no. of states of plant

Ts=plant.Ts;
if Ts
   error(message('Robust:design:sdhinfsyn3'));
elseif hasdelay(plant)
   error(message('Robust:design:sdhinfsyn4'));
end

% if NMEAS and NCONT not specified, then extract from plant using ISTITO
% and insert in VARARGIN array
if isequal(nag1,1) || ischar(varargin{2}) % pop nmeas and ncon from tito
   [tito,~,U2,~,Y2]=istito(plant);
   if ~tito
      error(message('Robust:design:sdhinfsyn5'));
   end
   ncon=length(U2);
   nmeas=length(Y2);
   temp=[varargin(1), {nmeas}, {ncon}, varargin(2:end)]; % push nmeas and ncon into varargin
   varargin=temp;
else
   nmeas=varargin{2};
   ncon=varargin{3};
end

% if input 4 is not a string, try old non-LTI syntax but with lti and return
if nag1>3 && ~ischar(varargin{4})
   [varargin{:}]=lti2mat(varargin{:});
   try  % Obsolete mutools sdhinfsyn command syntax
      [k,gam]=sdhfsyn(varargin{:});
      % warning('Obsolete mutools hinfsyn command syntax, with LTI plant');
   catch
      error(message('Robust:design:sdhinfsyn1'));
   end
   k=mat2lti(k);
   set(k,'Ts',h);
   return
end


% get (KEY,OPTION) pairs
key=varargin(4:2:end);
opt=varargin(5:2:end);
nopt=length(key);
nkey=length(opt);
if ~isequal(nkey,nopt)
   error(message('Robust:design:sdhinfsyn6'));
end

% replace defaults values with option values
for i=1:nkey
   keyi=keymatch(key{i},DocKeys,UndocKeys);
   switch keyi
      case 'GMAX '
         gmax=opt{i};
      case 'GMIN '
         gmin=opt{i};
      case 'TOLGAM '
         tol=opt{i};
      case 'TS '
         h=opt{i};
      case 'DELAY '
         delay=opt{i};
      case 'EPR '
         epr=opt{i};
      case 'EPP '
         epp=opt{i};
      case 'DISPLAY '
         verbose=opt{i};
         if ischar(verbose)
            if strcmpi(verbose(1:2),'on')
               quiet = 1; % ` -->  verbosity for sdhinfsyn
            else
               quiet = 0;
            end
            %               if isempty(verbose)
            %                  error([ '''' opt{i} '''is not a valid option for KEY ' key{i} ])
            %               end
            %            elseif ~isequal(quiet,1) & ~isequal(quiet,0)
            %               error(['Invalid option for key ' key{i} ])
         end
      case 'RICMETHOD '
         ricmethd=opt{i};
   end
end
if gmin>gmax
   error(message('Robust:design:sdhinfsyn7'));
end

%compute
[k,gam]= xsdhinfsyn(plant,nmeas,ncon,gmin,gmax,tol,h,delay,ricmethd,epr,epp,quiet);

