% PSINFO  Query characteristics of a P-system
%
%   PSINFO(PS) displays the characteristics of the polytopic or
%   parameter-dependent system PS (created with PSYS).
%
%   [TYP,K,NS,NI,NO] = PSINFO(PS) returns TYP, the type of system 
%   ('pol' for polytopic, 'aff' for affine parameter-dependent), the number
%   K of system matrices involved in its definition, the number of states NS,
%   the number of inputs, NI, and the number of outputs NO.
%
%   PV = PSINFO(PS,'par') returns the parameter vector description (for
%   parameter-dependent systems only).
%
%   SK = PSINFO(PS,'sys',K) returns the K-th SYSTEM matrix  Sk  involved in 
%   the definition of PS.
%
%   SYS = PSINFO(PS,'eval',P) instantiates the polytopic or parameter-
%   dependent state-space model.  The result is a SYSTEM matrix  SYS  given
%   by
%              p1*S1 + ... + pk*Sk
%     * SYS =  -------------------    if PS is polytopic and
%                 p1 + ... + pk
%
%            P = (p1,...,pk), pj >= 0  is a set of polytopic
%            coordinates
%     * SYS = S0 + p1*S1 + ... + pk*Sk   if PS is affine and
%            P is a particular value of the parameter vector
%
%   See also  PSYS, PVEC, LTISYS.

% Author: P. Gahinet  6/94
% Copyright 1995-2004 The MathWorks, Inc.

function [type,nv,ns,ni,no]=psinfo(ps,task,aux)

if ~any(nargin==[1 2 3])
   error('For correct syntaxes, type:   help psinfo');
elseif ~ispsys(ps),
   error('PS is not a polytopic or parameter-dependent system');
end

pstype=ps(2,1); nv=ps(3,1); ns=ps(4,1); ni=ps(5,1); no=ps(6,1); pv=[];

if nargin > 1 & size(ps,2)>1+nv*(ns+ni+2),
  pv=ps(1:max(2,ps(2,2+nv*(ns+ni+2))),2+nv*(ns+ni+2):size(ps,2));
  ps=ps(:,1:1+nv*(ns+ni+2));
end



if nargin==1,
%%%%%%%%%%%%%
  if nargout,
    if pstype==1, type='pol'; elseif pstype==2, type='aff'; end
  else
    if pstype==1,
      disp(sprintf(['Polytopic model with %d vertex systems \n' ...
      '    Each system has %d state(s), %d input(s), and %d output(s)\n'],...
                   nv,ns,ni,no));
    elseif pstype==2,
      disp(sprintf(['Affine parameter-dependent model with %d parameters (%d systems) \n' ...
      '    Each system has %d state(s), %d input(s), and %d output(s)\n'],...
                   nv-1,nv,ns,ni,no));
    end
  end


elseif strcmp(task,'par'),
%%%%%%%%%%%%%%%%%%%%%%%%%%
  type=pv;



elseif strcmp(task,'sys'),
%%%%%%%%%%%%%%%%%%%%%%%%%%
  if length(aux)~=1 , error('K must be a scalar');
  elseif aux > nv, error('K exceeds the number of systems'); end

  rs=ns+no+1; cs=ns+ni+1;
  type=ps(1:rs,3+(aux-1)*(cs+1):1+aux*(cs+1));



elseif strcmp(task,'eval'),
%%%%%%%%%%%%%%%%%%%%%%%%%%
  np=length(aux);
  if (pstype==2 & np~=nv-1) | (pstype==1 & np~=nv),
     error(sprintf('P should be of length %d',...
          (pstype==2)*(nv-1)+(pstype==1)*nv));
  end

  if pstype==1,
    if min(aux) < 0 | sum(aux) == 0,
       error('The entries of P must be nonnegative for polytopic PS');
    end
    aux=aux/sum(aux);
  elseif pstype==2,
    aux=[1;aux(:)];
  end

  rs=ns+no; cs=ns+ni;

  if any(pstype==[1 2]),
    type=zeros(rs,cs);
    ps=ps(1:rs,:);
    for k=1:nv,
      b=2+(k-1)*(cs+2);
      type=type+aux(k)*ps(:,b+1:b+cs);
    end
    ae=type(1:ns,1:ns);
    if pstype==2, ae=ae+sqrt(-1)*(sum(aux)-1)*eye(ns); end
    if max(max(abs(imag(ae)))) < 100*eps, ae=real(ae); end
    type(1:ns,1:ns)=ae;
    type(rs+1,cs+1)=-Inf; type(1,cs+1)=ns;
  end

end
