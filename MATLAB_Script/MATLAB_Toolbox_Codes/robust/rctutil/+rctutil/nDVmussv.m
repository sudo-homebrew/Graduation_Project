function nDV = nDVmussv(blk)
% Number of Decision variables in MUSSV calcs, based on Block
nDV = 0;
nblk = size(blk,1);
for i=1:nblk
   if blk(i,1)<0
      nDV = nDV + 2*blk(i,1)*blk(i,1);
   elseif blk(i,2)==0
      nDV = nDV + blk(i,1)*blk(i,1);
   else
      nDV = nDV + 1;
   end
end
