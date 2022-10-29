function [Gm,totbnd,hhsv] = balmr(G,mrtype,no)
%BALMR Square-root balanced truncation (unstable plant).
%
% [SS_M,TOTBND,HSV] = BALMR(SS_,MRTYPE,NO) performs balanced-
%    truncation model reduction on G(s):=(a,b,c,d) such that the infinity-
%    norm of the error (Ghed(s) - G(s)) <= sum of the 2(n-k) smaller Hankel
%    singular values. For unstable G(s) the algorithm works by first
%    splitting G(s) into a sum of stable and antistable part.
%
%   Based on the "MRTYPE" selected, you have the following options:
%
%    1). MRTYPE = 1  --- no: order "k" of the reduced order model.
%
%    2). MRTYPE = 2  --- find a k-th order model such that the total error
%                      is less than "no".
%
%    3). MRTYPE = 3  --- display all the Hankel SV and prompt for "k".
%
%    TOTBND = Error bound, HSV = Hankel Singular Values.

% R. Y. Chiang & M. G. Safonov 9/1/87
% Copyright 1988-2004 The MathWorks, Inc.
% All Rights Reserved.

%disp('  ')
%disp('        - - Working on Square-Root Balanced Model Reduction - -');

nag1=nargin;
%[emsg,nag1,xsflag,Ts,a,b,c,d,mrtype,no]=mkargs5x('ss',varargin); error(emsg);

[a,b,c,d,Ts] = ssdata(G);
if Ts, error('LTI inputs must be continuous time (Ts=0)'), end

if mrtype == 3
   no = 0;
end
[ra,ca] = size(a);
dd = eig(a);
%
% ------ Find the no. of stable roots :
%
indstab = find(real(dd) < 0);
m = length(indstab);
%
% ------ If completely stable :
%
if m == ra
   [Gm,augl,hhsv,slbig,srbig] = balsq(G,mrtype,no);
   augr = [0 0 0];
   totbnd = augl(1,2);
end
%
% ------ If completely unstable :
%
if m == 0
   [Gm,augr,hhsv,slbig,srbig] = balsq(-G,mrtype,no);
   Gm = -Gm;
   augl = [0 0 0];
   totbnd = augr(1,2);
end
%
% ------ If having both stable & unstable parts :
%
if m > 0 & m < ra
  [Gl,Gr,msat] = stabproj(G);
  hsvl = hksv(Gl);
  hsvr = hksv(Gr);
  hhsv = [hsvl;hsvr];
  [hhsv,index] = sort(hhsv);
  hhsv = hhsv(ra:-1:1,:);
  index = index(ra:-1:1,:);
  if mrtype == 1
      nor = 0;
      for i = 1:no
          if index(i) > msat
             nor = nor + 1;
          end
      end
     nol = no-nor;
   end
   if mrtype == 2
      tol = no;
      tails = 0;
      for i = ra:-1:1
          tails = tails + hhsv(i);
          if 2*tails > tol
             no = i;
             break
          end
      end
      nor = 0;
      for i = 1:no
          if index(i) > msat
             nor = nor + 1;
          end
      end
      nol = no-nor;
      mrtype = 1;
   end
   if mrtype == 3
      hankelsv(ss(a,b,c,d));
      nol = no;
      nor = no;
   end
   if mrtype == 3
       disp('For the Stable Part:');
   end
   [Glf,augl,hsvl,tll,tlr] = balsq(Gl,mrtype,nol);
   if mrtype == 3
      disp('For the Unstable Part:');
   end
   [Grf,augr,hsvr,trl,trr] = balsq(-Gr,mrtype,nor);
   Grf = -Grf;
   totbnd = augl(1,2)+augr(1,2);
   hhsv = [hsvl;hsvr];
   Gm = Glf+Grf;
end
%
nn = augl(1,1) + augr(1,1);
if mrtype == 3
   disp(' ')
   disp(['               ' int2str(nn), '    states removed !!'])
end
%
% ------- End of BALMR.M --- RYC 9/13/87 %
