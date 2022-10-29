function varargout = h2hinfsyn(varargin)
% [K,CL,NORMZ,INFO] = H2HINFSYN(P,NMEAS,NCON,NZ2,WZ,key1,value1,...)
%   Mixed H2/H-infinity synthesis with regional pole-placement
%   constraints.
%  
%   Given an LTI plant P with partitioned state-space form:
%  	 dx/dt = A  * x + B1  * w + B2  * u
%  	  zinf = Ci * x + Di1 * w + Di2 * u
%  	   z2  = C2 * x + D21 * w + D22 * u
%             y  = Cy * x + Dy1 * w + Dy2 * u
%   H2HINFSYN employs LMI techniques to compute an output-feedback control 
%   law u = K(s)*y that
%    * keeps the HINFNORM gain G from w to zinf below the value OBJ(1)
%    * keeps the H2NORM  H from w to z2 below the value OBJ(2)
%    * minimizes a trade-off criterion of the form
%                WEIGHT(1) * G^2 + WEIGHT(2) * H^2
%    * places the closed-loop poles in the LMI region specified
%      by REGION.
%  
%   Input:
%    P        LTI plant
%    NMEAS    Number of measurements (length of y)
%    NCON     Number of control channels (length of u)   
%    NZ2      length of z2
%    WZ       1x2 vector of weights on zinf and z2 (i.e., weights for
%             H-infinity and H2 performance, respectively)
% 
% Option keywords and values:
%    REGION   Mx(2M) matrix  [L,M]  specifying the pole
%             placement region as
%                  { z :  L + z * M + conj(z) * M' < 0 }
%             Use the interactive function  LMIREG  to generate
%             REGION.  The default  REGION=[]  enforces just
%             closed-loop stability
%    H2MAX    upper bound on the H2 norm  w->z2   (default = Inf)
%    HINFMAX  upper bound on the HINFNORM gain w->zinf (default = Inf)
%    DKMAX    bound on the norm of the 
%             feedthrough gain DK of K(s)
%             (100=default, 0 yields a strictly proper controller)
%    TOL      desired relative accuracy on the objective
%             value  (default=1e-2)
%    DISPLAY  'on' displays synthesis information to screen, default 'off'  
%  
%   Output:
%    K        optimal output-feedback controller
%    CL       lft(P,K) (closed-loop system)
%    NORMZ    1x2 vector of closed-loop norms (HINFNORM gain w->zinf 
%             and H2 norm  w->z2   
%    
%    INFO     struct array:
%               INFO.R        solution R of LMI solvability condition
%               INFO.S        solution S of LMI solvability condition
%  
%   See also  LMIREG, HINFLMI, MSFSYN.


%   Copyright 2003-2009 The MathWorks, Inc.
nag1=nargin;
nag2=nargout;
[varargin{:}] = convertStringsToChars(varargin{:});


% initialize variables
DocKeys={'REGION ','H2MAX ','HINFMAX ','DKMAX ','TOL ', 'DISPLAY '};
UndocKeys={''};
varargout=cell(1,nargout);
nargkey=6;  % default position of first option keyword in input list

% Initialize input values & option key default values
% P=[];
% nmeas=[];
% ncon=[];
tol=1e-2;
nz2=1;
wz=[1 1];
hinfmax=Inf;
h2max=Inf;
region=[];
display = 'off';


% r=[nz2,nmeas,ncon];
% obj= [hinfmax, h2max, wz];
dkmax=100;

% Initialize outputs
info=struct;
info.R=[];
info.S=[];

%  if no inputs, show usage
if ~nag1  
    disp('Usage [K,CL,NORMS,INFO] = h2hinfsyn(P,NMEAS,NCON,NZ,WZ,KEY1,VALUE1,KEY2,VALUE2,...)')
    return
end


% get plant
plant=varargin{1};
if ~isa(plant,'DynamicSystem')
    error('LTI plant must be first input argument')
else
   try
      plant = ss(plant);
   catch 
      error('Plant must have a state-space realization')
   end
end
Ts = plant.Ts;
if Ts  % Discrete-time 
    error('Plant must be continuous time')
end


%Get P,nmeas,ncont
% if NMEAS and NCONT not specified, then extract from plant using ISTITO
% and insert in VARARGIN array
if isequal(nag1,1) || ( (nag1 > nargkey-3) && ischar(varargin{nargkey-2}) ) % pop nmeas and ncon from tito
    [tito,~,U2,~,Y2]=istito(plant);
    if ~tito
        error('You must specify NMEAS and NCON'), 
    end
    ncon=length(U2);
    nmeas=length(Y2);
    temp=[varargin(1), {nmeas}, {ncon}, varargin(2:end)]; % push nmeas and ncon into varargin
    varargin=temp;
    nag1=nag1+2;
else
    nmeas=varargin{2};
    ncon=varargin{3};
end
P=plant;

% adjust nargkey, if some inputs left to default
for i=nag1:-1:1
    if isa(varargin{i},'char')
        nargkey=i;
    end
end

%                Input Syntax of hinfmix vs. h2hinfsyn
%  old:  [gopt,h2opt,K,R,S] = hinfmix(P,r,obj,region,dkbnd,tol)
%  new:  [K,CL,NORMZ,INFO] = H2HINFSYN(P,NMEAS,NCON,NZ,WZ,key1,value1,…)
% --------------------------------------------------------------
% Inputs:
% Old  hinfmix               New h2hinfsyn           Comments  
%--------------------        --------------          ----------
% P                   --     P,plant                 LTI plant
% r                   --     [nz2,nmeas,ncon]                                                                redundant
%                                                      
% Obj(1)              --     key HINFMAX             default Inf       
% Obj(2)              --     key H2MAX               default Inf 
% Obj(3)              --     WZ(1)                   default 1
% Obj(4)              --     WZ(2)                   default 1
% tol                 --     tol                     default 1e-2
% region              --     key REGION              Same as hinfmix.m
% dkbnd               --     dkmax                   default 100
%                            controller D-matrix
% 
% Other, old/new translation issues
%   Obj(1)=0      <==>       key HINFMAX=Inf
%   Obj(2)=0      <==>       key H2MAX  =Inf

if nargin<3
    error('Too few input arguments')
end

% get nz2 and wz inputs
if nag1>3
   nz2=[varargin{4}];
end
if nag1>4
   wz=[varargin{5}];
   if ~isa(wz,'double') 
       error('WZ must be a 1x2 real vector')
   end
   switch length(wz)
     case {0,2}
       % wz=wz; % replace by default for empty input wz 
     case 1
       wz=[wz, 1];  % set default wz(2)=1
     otherwise
       error('WZ must be a 1x2 real vector')
   end
   wz=abs(wz);
end

% get (KEY,OPTION) pairs, if present 
key=varargin(nargkey:2:end);
opt=varargin(nargkey+1:2:end);
nopt=length(key);
nkey=length(opt);
if ~isequal(nkey,nopt)
    error('Number of keys and options must be equal')
end


% Overwrite default values with option values for
% DocKeys={'REGION ','H2MAX ','HINFMAX ','DKMAX ','TOL ','DISPLAY '};
% UndocKeys={''};
for i=1:nkey
    keyi=keymatch(key{i},DocKeys,UndocKeys);
    switch keyi
        case 'REGION '
            region=opt{i};
        case 'H2MAX '
            h2max=opt{i};
        case 'HINFMAX '
            hinfmax=opt{i};
        case 'DKMAX '
            dkmax=opt{i};
        case 'TOL '
            tol=opt{i};
        case 'DISPLAY '
            display=opt{i};
    end
end    
if isequal(hinfmax,Inf)
    hinfmax=0;
end
if isequal(h2max,Inf)
    h2max=0;
end

% Now call lti/hinfmix
r=[nz2,nmeas,ncon];
obj=[hinfmax,h2max,wz];
[gopt,h2opt,Kmat,R,S] = hinfmix(lti2mat(P),r,obj,region,dkmax,tol,display);
K=mat2lti(Kmat);
% Now assemble output
%  VARARGOUT=[K,CL,NORMZ,INFO] 
if nag2>1
   CL=lft(P,K);
end
nzinf=size(CL,1)-nz2;
if isempty(gopt)
    gopt=norm(subsref(CL,substruct('()',{1:nzinf ':'})),inf);
end
normz=[gopt,h2opt];
info.R=R;
info.S=S;
temp={K,CL,normz,info};
varargout=temp(1:nag2);


