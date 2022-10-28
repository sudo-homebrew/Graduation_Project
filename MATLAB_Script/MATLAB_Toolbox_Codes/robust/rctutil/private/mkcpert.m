function [Delta,lvec,rvec]=mkcpert(b,a,index,opt)
%
% Creates the complex blocks for a perturbation.
% Real blocks are returned as zeros. If the
% dimensions of b/a correspond to the row/column
% dimensions of the complex blocks of Delta, then
% a/b will be padded with blocks of zeros corresponding
% to the real blocks of Delta.
%
% If opt='dyad': The complex blocks satisfy
%       Delta_i*a_i = b_i
% where Delta_i is a dyad for full blocks.
% lvec/rvec are nblk-by-1 cell arrays with the
% left/right vectors of the full block dyads and
% empty entries for scalar blocks.
%
% If opt='unitary': The complex blocks satisfy
%     Delta_i*a_i = b_i * ||a_i||/||b_i||
% where Delta_i is a unitary matrix. lvec/rvec
% are returned as empty nblk-by-1 cell arrays.
%
%
%

% PJS 4/19/04
% code based on rupert.m
% AKP 2/27/11
% small efficiency fixes

%   Copyright 2011 The MathWorks, Inc.

nin = nargin;
if nin==3					
  dyad = 1;
elseif nin==4 && strcmp(opt,'dyad'),
  dyad = 1;
elseif nin==4 && strcmp(opt,'unitary'),	
  dyad = 0;
else	
  error('mkcpert.m called incorrectly')
end

% Set up indices: 
%   rb=real scalars, sb=complex scalars, fb=complex full
blk = index.simpleblk;
newloc =index.newloc;

sb.azidx = index.repcomp.rows;
sb.bwidx = index.repcomp.cols;
sb.loc = index.repcomp.origloc;

fb.azidx = index.full.rows;
fb.bwidx = index.full.cols;
fb.loc = index.full.origloc;

% If b/a correspond to the complex blocks
% of Delta, pad with zeros at
% locations of real blocks.
if length(a)~=index.rdimm || length(b)~=index.cdimm
  olda = a(:);
  a = zeros(index.rdimm,1);
  a(index.allcomp.allrows) = olda;
  
  oldb = b(:);
  b = zeros(index.cdimm,1);
  b(index.allcomp.allcols) = oldb;
end  

Delta = zeros(length(b),length(a));
nblk = size(blk,1);
lvec=cell(nblk,1);
rvec=cell(nblk,1);
for i=1:nblk
  idx = newloc(i);
  if  blk(i,2)==0 && blk(i,1)>0
    ai = a(sb.azidx{idx}); 
    bi = b(sb.bwidx{idx});      
    [maxmag,midx] = max(abs(ai));
    if maxmag < 100*eps 
      di = 0;
    else
      di = bi(midx)/ai(midx);
    end
    if ~dyad
      if abs(di)< 100*eps
        di = 1;
      else
        di = di/abs(di);
      end
    end
    Delta(sb.bwidx{idx},sb.azidx{idx}) = di*eye(blk(i,1));
  elseif blk(i,2)>0
    ai = a(fb.azidx{idx});
    bi = b(fb.bwidx{idx});
    nai = norm(ai);
    if nai < 100*eps 
      nai = 1;
      ai = zeros(size(ai));
    end
    LV = bi/nai;
    RV = (ai')/nai;
    Deltai = LV*RV;
    if dyad
      lvec{i} = LV;
      rvec{i} = RV;
    else      
      [u,s,v] = svd(Deltai);
      Deltai = u*eye(size(s))*v';
    end
    Delta(fb.bwidx{idx},fb.azidx{idx}) = Deltai;     
  end
end
