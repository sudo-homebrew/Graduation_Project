function [out,mat,delta] = randumat(NY,NU)
%RANDUMAT  Generates random UMAT.
%
%  OUT = randumat(NY,NU) generates an uncertain matrix of size
%  NY-by-NU. RANDUMAT randomly selects between UREAL, UCOMPLEX and ULTIDYN 
%  uncertaint atoms.
%
%  If RANDUMAT is called with no input arguments, a 1-by-1 uncertain matrix
%  is generated with up to 4 uncertain objects.
%
%  See also RAND, RANDN, RANDATOM, RANDUSS, UCOMPLEX, ULTIDYN, UREAL. 

%   Copyright 2003-2011 The MathWorks, Inc.

if nargin == 0
   NY = 1;
   NU = 1;
elseif nargin == 1
   NU = NY;
end

if isa(NY,'double') && length(NY)==1 && NY>0 && floor(NY)==ceil(NY)
   % NY is an integer
else
   error('NY must be a positive integer')
end
if isa(NU,'double') && length(NU)==1 && NU>0 && floor(NU)==ceil(NU)
   % NU is an integer
else
   error('NU must be a positive integer')
end

blktype = {'ureal' 'ucomplex' 'ultidyn'};
delta = [];
nblks = ceil(4*rand);
for i=1:nblks
   blk = blktype{ceil(2*rand)};
   if i==1
      blk = 'ucomplex';
   end   
   if isequal(blk,'ultidyn')
      at = randatom(blk);
      delta = blkdiag(delta,at);      
   else
      ncop = ceil(3*rand);
      at = randatom(blk);
      for j=1:ncop
         delta = blkdiag(delta,at);
      end
   end
end
szd = size(delta);
ny = szd(2)+NY;
nu = szd(1)+NU;
mat = randn( ny,nu);

mat = mat([1:szd(2) 1:szd(2) szd(2)+1:szd(2)+NY],[1:szd(1) 1:szd(1) szd(1)+1:szd(1)+NU]);
mat = lft(-delta.NominalValue,mat);

out = lft(delta,mat);

