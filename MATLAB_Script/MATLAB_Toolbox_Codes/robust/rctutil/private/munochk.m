%function [bnds,rowd,sens,rowp,rowg,sm] = munochk(matin,blk,opt)
function [sm] = munochk(M,blk,opt)

% Copyright 2003-2004 The MathWorks, Inc.

% This must be in ROBUST/ROBUST, so that the PRIVATE routines can
% be accessed.
% calls: reindex, rubind, rub(deebal, rublow, newga, lamb, newub, dang, dand, geestep)


%--------------------------------------------------
% Check blk structure and get block indices
%--------------------------------------------------

nblk = size(blk,1);
for ii=1:nblk
   if all( blk(ii,:) == [ 1 0] )
      blk(ii,:)  = [ 1 1] ;
   end
   if all( blk(ii,:) == [-1 1] )
      blk(ii,:)  = [-1 0] ;
   end
end
for ii=1:nblk
   if all(blk(ii,:) == [-1 -1])
      blk(ii,:) = [-1 0];
   end
end

% Get indices from blk2idx plus a few other indices/masks needed for mu
% Also grab indices for real blocks only and for complex blocks only
index = rctutil.mkBlkData(blk,[]);

%--------------------------------------------------
% Loop to get scaled matrices
%--------------------------------------------------
szm = size(M);
sm = zeros(szm);
npts = prod(szm(3:end));
for i=1:npts
   sm(:,:,i)  = osbal(M(:,:,i),index);   
end

