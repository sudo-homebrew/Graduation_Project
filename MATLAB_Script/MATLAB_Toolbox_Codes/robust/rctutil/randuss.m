function out = randuss(NX,NY,NU,ts,ncb,nexd)
%RANDUSS  Generates stable random USS.
%
%  USYS = RANDUSS(N) generates an Nth-order SISO uncertain system USYS. Only
%  UREAL, UCOMPLEX and ULTIDYN uncertain atoms are included in the uncertain
%  state-space models.
% 
%  USYS = RANDUSS(N,P) generates a single-input Nth-order uncertain system 
%  with P outputs.
% 
%  USYS = RANDUSS(N,P,M) generates an Nth-order uncertain system with P outputs 
%  and M inputs.
% 
%  USYS = RANDUSS(N,P,M,Ts) generates an Nth-order uncertain system with P outputs 
%  and M inputs with sample-time Ts.
%
%  If RANDUSS is called with no input arguments, a 1-by-1 uncertain system
%  is generated with up to 4 uncertain objects.
%
%  See also RAND, RANDN, RANDATOM, RANDUMAT, UCOMPLEX, ULTIDYN, UREAL. 

% Copyright 2003-2004 The MathWorks, Inc.

if nargin==0
   NX = [];
   NY = 1;
   NU = 1;
   ts = 0;
   ncb = 0;
   nexd = [];
elseif nargin==1
   NY = 1;
   NU = 1;
   ts = 0;
   ncb = 0;
   nexd = [];
elseif nargin==2
   NU = 1;
   ts = 0;
   ncb = 0;
   nexd = [];
elseif nargin==3
   ts = 0;
   ncb = 0;
   nexd = [];
elseif nargin==4
   ncb = 0;
   nexd = [];
elseif nargin==5
   nexd = [];
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

if isempty(NX)
   NX = ceil(7*rand);
else
   if isa(NX,'double') && length(NX)==1 && NX>0 && floor(NX)==ceil(NX)
      % NX is an integer
   else
      error('NX must be a positive integer')
   end
end

if ncb == 0 
   fac = 3;  %want all 'ureal' 'ultidyn' 'ucomplex'
else
   fac = 2;  %want just 'ureal' 'ultidyn'
end
blktype = {'ureal' 'ultidyn' 'ucomplex'};
delta = [];
for i=1:ceil(4*rand)
   blk = blktype{ceil(fac*rand)};
   if isequal(blk,'ultidyn')
      satom = randatom(blk);
      satom = set(satom,'AutoSimplify','full');
      delta = blkdiag(delta,satom);
   else
      ncop = ceil(3*rand);
      at = randatom(blk);
      for j=1:ncop
         delta = blkdiag(delta,at);
      end
   end
end
satom = randatom('ultidyn');
satom = set(satom,'AutoSimplify','full');
delta = blkdiag(delta,satom);
ny = size(delta,2)+NY;
nu = size(delta,1)+NU;
if isempty(nexd)
   exd = 1;
elseif nexd==-1
   nexd = floor(4*rand);
   exd = ceil(3*rand(1,nexd));
elseif isequal(size(nexd),[1 1])
   nexd = ceil(abs(nexd(1)));
   exd = ceil(3*rand(1,nexd));
else
   exd = nexd;
   nexd = length(exd);
end
x = num2cell(exd);
usflg = 1;
while usflg == 1
   if ts==0
      sys = rss( NX,ny,nu,x{:});
      p = pole(sys);
      if max(real(p(:)))<-1e-1
         usflg = 0;
      end
   else
      sys = drss( NX,ny,nu,x{:});
      sys.Ts = ts;
      p = pole(sys);
      if max(abs(p(:)))<0.95
         usflg = 0;
      end
   end
end
szd = size(delta);
sys = sys([1:szd(2) 1:szd(2) szd(2)+1:szd(2)+NY],[1:szd(1) 1:szd(1) szd(1)+1:szd(1)+NU]);
sys = lft(-delta.NominalValue,sys);
out = lft(delta,sys);

