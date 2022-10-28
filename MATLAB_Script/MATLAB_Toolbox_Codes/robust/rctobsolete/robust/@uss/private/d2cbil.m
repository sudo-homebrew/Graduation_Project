function csys = d2cbil(dsys,alpha)
% function csys = D2CBIL(dsys,alpha)
%
%   Transforms discrete-time system into a continuous-time
%   system with a bilinear transformation, defined as
%
%                    s + alpha
%           z =  -  -----------
%                    s - alpha
%
%   Note that the point s = j alpha gets mapped to z = j.
%   If ALPHA>0, then the right-half plane (for s) is mapped
%   outside the unit disk (for z), preserving the stability
%   characteristics.
%----------------------------------------------------------
%   WARNING: This is (unfortunate choice) not the same
%       transformation as that used in C2DBIL.  In C2DBIL, the
%       transformation below,
%
%                    s + (1/alpha)
%           z =  -  --------------
%                    s - (1/alpha)
%
%   is used.  Hence, for any SYSTEM matrix SYS, and any
%   ALPHA, it follows that
%
%       SYS    and    C2DBIL(D2CBIL(SYS, 1/ALPHA), ALPHA)
%
%   are equal.
%

% Copyright 2003-2004 The MathWorks, Inc.

% What do we do for sample time? Current set to -1

  if isa(dsys,'ss') && dsys.Ts ~=0
     mnum = length(dsys.StateName);
    if nargin == 1
      alpha = 1;
    elseif nargin == 2
      if alpha <= 0
        error('ALPHA must be positive')
        return
      end
    end
    fix = [-alpha*eye(mnum) -sqrt(2*alpha)*eye(mnum);...
           -sqrt(2*alpha)*eye(mnum) -eye(mnum)];
%   -----------  the following also works  ------------
%   fix = [-alpha*eye(mnum) sqrt(2*alpha)*eye(mnum);...
%          sqrt(2*alpha)*eye(mnum) -eye(mnum)];
%   ---------------------------------------------------
    [ad,bd,cd,dd] = ssdata(dsys);
    ddata = [ad bd;cd dd];
    sdata = lft(fix,ddata,mnum,mnum);
    csys = ss(sdata(1:mnum,1:mnum),sdata(1:mnum,mnum+1:end),...
               sdata(mnum+1:end,1:mnum),sdata(mnum+1:end,mnum+1:end),0);
  elseif isstatic(dsys)
     csys = dsys;
     csys.Ts = 0;
  elseif isa(dsys,'double')
     csys = dsys;
  elseif isempty(dsys)
     csys = [];
  else
    error('Input variable is not a SS object')
  end