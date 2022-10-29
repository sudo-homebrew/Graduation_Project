function dsys = c2dbil(csys,alpha)
%  function dsys = c2dbil(csys,alpha)
%
%   Transforms continuous-time system into a discrete-time
%   system with a bilinear transformation, defined as
%
%                  1     z-1
%           s = ------- -----
%                alpha   z+1
%
%                                                 j
%   Note that the point z=j gets mapped to s = -------
%                                               alpha
%
%   If ALPHA>0, then the right-half plane (for s) is mapped
%   outside the unit disk (for z), preserving the stability
%   characteristics.
%----------------------------------------------------------
%   WARNING: This is (unfortunate choice) not the same
%       transformation as that used in D2CBIL.  In D2CBIL, the
%       transformation below,
%
%                      z-1
%           s = alpha -----
%                      z+1
%
%   is used.  Hence, for any SYSTEM matrix SYS, and any
%   ALPHA, it follows that
%
%       SYS    and    C2DBIL(D2CBIL(SYS, 1/ALPHA), ALPHA)
%
%   are equal.
%

% Copyright 2003-2004 The MathWorks, Inc.

  if isa(csys,'ss') && csys.Ts ==0
     mnum = length(csys.StateName);
     if nargin == 1
        alpha = 1;
     elseif nargin == 2
        if alpha <= 0
           error('ALPHA must be positive')
           return
        end
     end
     fix = [eye(mnum) sqrt(2*alpha)*eye(mnum);...
            sqrt(2*alpha)*eye(mnum) alpha*eye(mnum)];
     [ac,bc,cc,dc] = ssdata(csys);
     cdata = [ac bc;cc dc];
     sdata = lft(fix,cdata,mnum,mnum);
     dsys = ss(sdata(1:mnum,1:mnum),sdata(1:mnum,mnum+1:end),...
        sdata(mnum+1:end,1:mnum),sdata(mnum+1:end,mnum+1:end),-1);
  elseif isa(csys,'double')
      dsys = csys;
  elseif isempty(csys)
      dsys = [];
  else
      error('Input variable is not a SS object')
      return
  end
