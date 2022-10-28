%AFF2POL   Convert affine P-systems to polytopic representation
%
%   For uncertain state-space models, POLSYS = AFF2POL(AFFSYS) converts the
%   affine parameter-dependent representation AFFSYS to a polytopic 
%   representation POLSYS.
%
%   See also  PSYS, PVEC, PSINFO, PVINFO.

%   Author: P. Gahinet  6/94
%   Copyright 1995-2004 The MathWorks, Inc.

function polsys = aff2pol(affsys)

if nargin~=1,
  error('usage: polsys=aff2pol(affsys)')
elseif size(affsys,1)<2,
  error('First argument is not an affine parameter-dependent system');
elseif ~ispsys(affsys) | affsys(2,1)~=2,
  error('First argument is not an affine parameter-dependent system');
end

pv=psinfo(affsys,'par');
[typ,np,npv]=pvinfo(pv);


if strcmp(typ,'box'),  % box type

  % generate matrix of vertices (columns)
  [minval,maxval]=pvinfo(pv,'par',1);
  vmat=[minval,maxval];

  for j=2:np,
    [minval,maxval]=pvinfo(pv,'par',j);
    ls=size(vmat,2);
    vmat=[vmat vmat;minval*ones(1,ls) maxval*ones(1,ls)];
  end

else

  vmat=pv(1:np,2:size(pv,2));

end


polsys=[];
for t=vmat,
  polsys=[polsys, psinfo(affsys,'eval',t)];
end

polsys=psys(pv,polsys,1);
