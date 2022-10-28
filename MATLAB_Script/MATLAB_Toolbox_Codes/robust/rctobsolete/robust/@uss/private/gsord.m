function sensord = gsord(sens,blk)
% function sensord = gsord(sens,blk);
%
% Reorders the SENS variable from MUN to be compatible with BLK

% P. M. Young, December, 1996.
% GJB modified 28aug08 to work with RCT commands

%   Copyright 2009-2018 The MathWorks, Inc.


if nargin < 2
   disp('usage: sensord = gsord(sens,blk)')
   return
end

[type,row,col,~] = minfo(sens);

if (strcmp(type,'empt')==1) && (isempty(blk))
   sensord = [];
   return
end

if (strcmp(type,'empt')==1) || (isempty(blk))
   error('SENS and BLK are incompatible in SORD')
end

if (row~=1) || (col~=length(blk(:,1)))
   error('SENS and BLK are incompatible in SORD')
end

if strcmp(type,'syst')==1
   error('SENS should be a CONSTANT or VARYING row vector')
end

if ~(isreal(blk) && size(blk,2)==2 && norm(rem(blk,1),1)==0 && all(blk(:,1)~=0))
   error('   BLK is invalid')
end

for ii=1:length(blk(:,1))
   if all( blk(ii,:) == [ 1 0] )
      blk(ii,:)  = [ 1 1] ;
   end
   if all( blk(ii,:) == [-1 1] )
      blk(ii,:)  = [-1 0] ;
   end
end

if any(blk(:,2)<0)
   error('   BLK is invalid');
end
if any(blk(:,2)~=0&blk(:,1)<0)
   error('   Real FULL blocks not allowed');
end
if any(abs(blk)>500)
   error('   No blocks larger than 500, please')
end

[~,~,~,~,~,~,~,~,Us] = LOCALrindexv4(blk);

%sensord = sel(sens,1,Us);
sensord = sens(1,Us);

% Local subroutine RINDEXV4
function [Or,Oc,Ur,Uc,K,I,J,Os,Us] = LOCALrindexv4(blk)
%	Create a set of index vectors for
%	the block structure blk to create
%	the block structure specified by K, I, and J.
%	Or, Oc, Ur, and Uc are index vectors so that
%	MO = M(Oc,Or), and M = MO(Uc,Ur).
%       Authors:  Matthew P. Newlin and Peter M. Young

%	Also creates analagously Os and Us to reorder SENS

%     Modified by P. M. Young, December, 1996.

if isempty(blk)
   K  = []; 	I  = []; 	J  = [];
   Oc = [];	Or = [];	Uc = [];	Ur = [];
   Os = [];	Us = [];
   return
end

% The following stuff is usually done to BLK before calling
% RINDEX anyway, but it's essential so let's do it here to be sure.
% It is ASSUMED that other error checks - for real full blocks, invalid
% block structures etc. have been done before calling RINDEX.

blk = round(real(blk));
for ii=1:length(blk(:,1))
   if all( blk(ii,:) == [ 1 0] )
      blk(ii,:)  = [ 1 1] ;
   end
   if all( blk(ii,:) == [-1 1] )
      blk(ii,:)  = [-1 0] ;
   end
end

b = blk;
ab = abs(b);
fb = [b(:,2)==0&b(:,1)<0, b(:,2)==0&b(:,1)>0, b(:,2)>0, 1+(b(:,2)>0)];
%	real/rep		complex/rep	complex/full	complex
Ir1 = []; Ir2 = []; Ir3 = []; Ic1 = []; Ic2 = []; Ic3 = [];

for ii = 1:length(b(:,2))
   oner = ones(ab(ii,      1),1);
   onec = ones(ab(ii,fb(ii,4)),1);
   Ir1 = [Ir1; oner*fb(ii,1)]; %#ok<*AGROW>
   Ir2 = [Ir2; oner*fb(ii,2)];
   Ir3 = [Ir3; oner*fb(ii,3)];
   Ic1 = [Ic1; onec*fb(ii,1)];
   Ic2 = [Ic2; onec*fb(ii,2)];
   Ic3 = [Ic3; onec*fb(ii,3)];
end

% for PMY and DGKIT added LOGICAL
if fb(:,1)==0, K = []; else, K = ab(logical(fb(:,1)),1); end
if fb(:,2)==0, I = []; else, I =  b(logical(fb(:,2)),1); end
if fb(:,3)==0, J = []; else, J =  b(logical(fb(:,3)),:)*[1; 0.001]; end

Or = [find(Ir1);find(Ir2);find(Ir3)];
Oc = [find(Ic1);find(Ic2);find(Ic3)];
[~,Ur] = sort(Or);
[~,Uc] = sort(Oc);

Os = [find(fb(:,1));find(fb(:,2));find(fb(:,3))];
[~,Us] = sort(Os);

