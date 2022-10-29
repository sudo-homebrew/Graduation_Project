function [mIC,BlkStruct] = simplifyBlock(mIC,BlkStruct,idx,level)
%SIMPLIFYBLOCK  Eliminates redundant copies of a block in LFT model.
%
%   The LFT model is given as LFT(DELTA,M) where M is a 2D or 3D array.
%   The blocks are sorted by name and their structure is described by
%   BLKSTRUCT. The index IDX specifies which block in BLKSTRUCT is being
%   simplified, and LEVEL specifies the simplification level.

%   Copyright 2003-2011 The MathWorks, Inc.
if level==0 || BlkStruct(idx).Occurrences==0 || ~all(isfinite(mIC(:)))
   return
end
szmIC = size(mIC);
npts = size(mIC,3);

% Compute total row and column size for each unique block
BlockSize = cat(1,BlkStruct.Size);
BlockOccurrence = cat(1,BlkStruct.Occurrences);
TotalSize = BlockSize .* BlockOccurrence(:,[1 1]);

theblk = BlkStruct(idx);
bcopies = theblk.Occurrences;
blksize = theblk.Size;
switch theblk.Type
   case {'ureal', 'ucomplex'}
      traits.zizo = true;
      traits.linear = true;
      traits.scalar = true;
   case {'ultidyn', 'ucomplexm', 'umargin'}
      traits.linear = true;
      traits.zizo = true;
      traits.scalar = all(blksize==1);
   case {'udyn'}
      traits.linear = false;
      traits.zizo = false;
      traits.scalar = all(blksize==1);
end
totalblksz = bcopies*blksize;
brow = blksize(1);
bcol = blksize(2);

% Locate portion of mIC associate with block #idx
RowStart = sum(TotalSize(1:idx-1,2));
ColStart = sum(TotalSize(1:idx-1,1));

Arows = RowStart + (1:totalblksz(2));
nrows = 1:szmIC(1);    % everything
nrows( Arows ) = [];

Acols = ColStart + (1:totalblksz(1));
ncols = 1:szmIC(2);
ncols( Acols ) = [];

% at matrix level, the partitioning is
%  [D11 C1  D12]
%  [B1  A   B2 ]
%  [D21 C2  D22],
% since the block might be anywhere.
Bmat = mIC(Arows,ncols,:); % immediately 3-d at most
Cmat = mIC(nrows,Acols,:);
Amat = mIC(Arows,Acols,:);
Dmat = mIC(nrows,ncols,:);
[Amat,Bmat,Cmat,Dmat,bcopies,doreduce] = LOCALreduceBlock(Amat,Bmat,Cmat,Dmat,brow,bcol,bcopies,traits,level);

if doreduce==1
   % Reduction successful
   nelim = theblk.Occurrences-bcopies;
   szmICr = szmIC(1:2) - nelim*blksize([2 1]);
   totalblksz = bcopies*blksize;
   Arows = RowStart + (1:totalblksz(2));
   nrows = 1:szmICr(1);    % everything
   nrows( Arows ) = [];
   Acols = ColStart + (1:totalblksz(1));
   ncols = 1:szmICr(2);
   ncols( Acols ) = [];
   mIC = zeros([szmICr(1:2) npts]);
   mIC(Arows,Acols,:) = Amat;
   mIC(Arows,ncols,:) = Bmat;
   mIC(nrows,Acols,:) = Cmat;
   mIC(nrows,ncols,:) = Dmat;
   BlkStruct(idx).Occurrences = bcopies;
end

%----------------------------------------------------------------------------
function [Amat,Bmat,Cmat,Dmat,bcopies,doreduce] = ...
   LOCALreduceBlock(Amat,Bmat,Cmat,Dmat,brow,bcol,bcopies,traits,level)

npts = size(Amat,3);
doreduce = 0;

% Loop through until no progress
go = true;
while go && bcopies>0
   go = false;
   if level>=1
      [Amat,Bmat,Cmat,keep] = LOCALzizoelim(Amat,Bmat,Cmat,bcol,brow,bcopies,traits.zizo);
      if keep<bcopies
         doreduce = 1;
         bcopies = keep;
         go = true;
      end
   end
   if level>=1 && norm(Amat(:))==0 && bcopies>0
      [Bmat,Cmat,keep] = LOCALidrowcol(Bmat,Cmat,bcopies,brow,bcol,traits.linear);
      if keep<bcopies
         doreduce = 1;
         bcopies = keep;
         go = true;
         Amat = Amat(1:bcopies*bcol,1:bcopies*brow,:);
      end
   end
   % TODO: Misses on this (though scalars work)
   % >> f = ultidyn('f',[4 3]);
   % >> ff = 2*f - f*2
   % but gets
   % >> ff = f - f;
   % We know how to fix this.
   
   if level>=2 && bcopies>0
      if traits.linear
         if traits.scalar
            if LOCALisndssid(Amat)
               [Cnew,Bnew,keep] = LOCALlrmoda(Cmat,Bmat);
            end
            if keep<bcopies
               doreduce = 1;
               bcopies = keep;
               Amat = Amat(1:bcopies*bcol,1:bcopies*brow,:);
               Bmat = Bnew;
               Cmat = Cnew;
               go = true;
            end
         end
      end
   end
   if traits.linear && traits.scalar && npts==1 && bcopies>0
      % APPLY SMINREAL to ss(Amat,Bmat,Cmat,Dmat),
      % using direct call to SMREAL
      [tmpa,tmpb,tmpc] = smreal(Amat,Bmat,Cmat,[]);
      keep = size(tmpa,1);
      if keep<bcopies
         doreduce = 1;
         bcopies = keep;
         Amat = tmpa;
         Bmat = tmpb;
         Cmat = tmpc;
         go = true;
      end
   end
end % while go

if bcopies>=1 && traits.linear && traits.scalar && npts==1 && level>=3
   NX = bcopies;
   rhoA = max(abs(eig(Amat)));
   etol = NX*NX*100*eps;
   % Allow for reduction via Balancing, even if "unstable", by scaling the
   % matrix down to have spectral radius equal to 1/1.05.  Use the value
   % later to rescale back appropriately.
   if rhoA>=1-etol
      alpha = 1/(1.05*rhoA);
   else
      alpha = 1;
   end
   ABCDN = [alpha*Amat sqrt(alpha)*Bmat;sqrt(alpha)*Cmat Dmat];
   [keep,ABCDred,errc] = LOCALreduce(ABCDN,NX,1);
   if keep<bcopies  && errc==0
      % Overwrite the state-space data
      bcopies = keep;
      Amat = ABCDred(1:keep,1:keep)/alpha;
      Bmat = ABCDred(1:keep,keep+1:end)/sqrt(alpha);
      Cmat = ABCDred(keep+1:end,1:keep)/sqrt(alpha);
      Dmat = ABCDred(keep+1:end,keep+1:end);
      doreduce = 1;
   end
end


%----------------------------------------------------------------------------
function [keep,ABCDred,errc] = LOCALreduce(ABCD,NX,alpha)

sbmudlhand = sbmudl(ss,'gethandle');
if nargin==2
   [Abal,Bbal,Cbal,Dbal,hsv,errc] = sbmudlhand(...
      ABCD(1:NX,1:NX),ABCD(1:NX,NX+1:end),...
      ABCD(NX+1:end,1:NX),ABCD(NX+1:end,NX+1:end),[]);
else
   BLT = kron([-alpha -sqrt(2*alpha);-sqrt(2*alpha) -1],eye(NX));
   ABCDmod = lft(BLT,ABCD,NX,NX);
   [Abal,Bbal,Cbal,Dbal,hsv,errc] = sbmudlhand(...
      ABCDmod(1:NX,1:NX),ABCDmod(1:NX,NX+1:end),...
      ABCDmod(NX+1:end,1:NX),ABCDmod(NX+1:end,NX+1:end),[]);
end
if ~errc
   if isempty(hsv)
      keep = 0;
      ABCDred = Dbal;
   else
      %     tightened original tolerances 1e-8 --> 1e-12, 1e-10 --> 1e-14
      keep = min([length(find(hsv>(hsv(1)*1e-12))) length(find(hsv>1e-14))]);
      if keep<NX
         ABCDred = [Abal(1:keep,1:keep) Bbal(1:keep,:);Cbal(:,1:keep) Dbal];
         if nargin==3
            BLTI = kron([1 sqrt(2*alpha);sqrt(2*alpha) alpha],eye(keep));
            ABCDred = lft(BLTI,ABCDred,keep,keep);
         end
      else
         ABCDred = ABCD;
      end
   end
else
   keep = NX;
   ABCDred = ABCD;
end

function [a,b,c,repremain] = LOCALzizoelim(a,b,c,ra,ca,rep,zizoflg) %#ok<INUSL>
%  Simple reduction based on identically zero rows/columns.
%  Works on N-D too..

% this needs alot of extra baggage to handle the cases when A_{i,i}
% is not equal to zero.  Specific about Literal, Normalizer, and Range
% will determine whether it can be deleted.

del = 1;
ndel = length(del);
while ndel>0
   sza = size(a);
   szb = size(b);
   szc = size(c);
   repremain = sza(1)/ra;
   del = [];
   for itry=1:repremain
      rows = ((itry-1)*ra+1):(itry*ra);
      cols = ((itry-1)*ca+1):(itry*ca);
      tcola = a(:,cols,:);
      tcolc = c(:,cols,:);
      if zizoflg
         trowa = a(rows,:,:);
         trowb = b(rows,:,:);
         if norm(trowa(:),'fro')==0 && norm(trowb(:),'fro')==0
            del = [del;itry]; %#ok<*AGROW>
         elseif norm(tcola(:),'fro')==0 && norm(tcolc(:),'fro')==0
            del = [del;itry];
         end
      else
         if norm(tcola(:),'fro')==0 && norm(tcolc(:),'fro')==0
            del = [del;itry];
         end
      end
   end
   ndel = length(del);
   if ndel>0
      ridxelim = ra*kron(del-1,ones(ra,1)) + kron(ones(ndel,1),(1:ra)');
      cidxelim = ca*kron(del-1,ones(ca,1)) + kron(ones(ndel,1),(1:ca)');
      a(:,cidxelim,:) = [];
      a(ridxelim,:,:) = [];
      b(ridxelim,:,:) = [];
      c(:,cidxelim,:) = [];
      nsza = size(a);
      a = reshape(a,[nsza(1) nsza(2) sza(3:end)]);
      b = reshape(b,[nsza(1) szb(2) szb(3:end)]);
      c = reshape(c,[szc(1) nsza(2) szc(3:end)]);
   end
end



function [B1,C1,rep] = LOCALidrowcol(B,C,brep,brow,bcol,lflag)

% Copyright 2003-2004 The MathWorks, Inc.

szb = size(B);
nB = ndims(B);
szc = size(C);
nC = ndims(C);
if nB==2 && nC==2
   npts = 1;
elseif nB==3 && nC==3
   if szb(3)==szc(3)
      npts = szc(3);
   else
      error('Invalid BC pair');
   end
elseif nB==2 && nC==3
   npts = szc(3);
   B = repmat(B,[1 1 npts]);
elseif nB==3 && nC==2
   npts = szb(3);
   C = repmat(C,[1 1 npts]);
else
   error('Invalid BC pair');
end

%Find B's that are identical to each other
grp = {};
cnt = 0;
left = 1:brep;
while ~isempty(left)
   i = left(1);
   left(1) = [];
   ridx = (i-1)*bcol+1:i*bcol;  % col refers to col(DELTA),hence row of M
   Bi = B(ridx,:,:);
   if ~lflag || norm(Bi(:))>0
      cnt = cnt + 1;
      grp{cnt} = i;
      tmpleft = left;
      for j=1:length(tmpleft)
         jl = tmpleft(j);
         ridx = (jl-1)*bcol+1:jl*bcol;
         Bj = B(ridx,:,:);
         if norm(Bi(:)-Bj(:))==0
            jlidx = left==jl;
            left(jlidx) = [];
            grp{cnt} = [grp{cnt} jl];
         end
      end
   else
      % Do nothing at this point
   end
end

% If LFLAG==1, then GRP pnly has the nonzero B's grouped

B1 = zeros(length(grp)*bcol,szb(2),npts);
C1 = zeros(szc(1),length(grp)*brow,npts);
grpkeep = [];
for i=1:length(grp)
   iloc = grp{i}(1);
   iidxc = (i-1)*bcol+1:i*bcol;
   iidxc2 = (iloc-1)*bcol+1:iloc*bcol;
   B1(iidxc,:,:) = B(iidxc2,:,:);
   iidxr = (i-1)*brow+1:i*brow;
   for j=1:length(grp{i})
      jloc = grp{i}(j);
      jidxr2 = (jloc-1)*brow+1:jloc*brow;
      C1(:,iidxr,:) = C1(:,iidxr,:) + C(:,jidxr2,:);
   end
   tmp = C1(:,(i-1)*brow+1:i*brow,:);
   if norm(tmp(:))>0
      grpkeep = [grpkeep i];
   end
end
cnt = length(grpkeep);
keepC = LOCALMkblkidx(grpkeep,brow);
keepB = LOCALMkblkidx(grpkeep,bcol);
C1 = C1(:,keepC,:);
B1 = B1(keepB,:,:);

if lflag
   tot = [2 1 3];
   [C1t,B1t,rep] = LOCALidrowcol(permute(C1,tot),permute(B1,tot),cnt,bcol,brow,0);
   C1 = permute(C1t,tot);
   B1 = permute(B1t,tot);
else
   rep = cnt;
end

function blkidx = LOCALMkblkidx(idx,width)
% Make block index as ROW vector

n = length(idx);
wrep = LOCALRepmat(1:width,[1 n]);
sp = LOCALRepmat((idx-1)*width,[width 1]);
blkidx = reshape(sp,[1 n*width]) + wrep;

function B = LOCALRepmat(A,siz)
if length(A)==1
   nelems = prod(siz);
   if nelems>0
      % Since B doesn't exist, the first statement creates a B with
      % the right size and type.  Then use scalar expansion to
      % fill the array.. Finally reshape to the specified size.
      B(nelems) = A;
      B(:) = A;
      B = reshape(B,siz);
   else
      B = A(ones(siz));
   end
elseif ndims(A)==2 && length(siz)==2
   [m,n] = size(A);
   mind = (1:m)';
   nind = (1:n)';
   mind = mind(:,ones(1,siz(1)));
   nind = nind(:,ones(1,siz(2)));
   B = A(mind,nind);
end


function [Lnew,Rnew,rk] = LOCALlrmoda(L,R)

% Copyright 2003-2005 The MathWorks, Inc.

szl = size(L);
szr = size(R);
ndl = length(szl);
ndr = length(szr);
% Exact properties:
% factor L*R into Lnew*Rnew.  Either and/or both can
% be 3-d, in general the results Lnew and Rnew are both 3-d
% even if just one (not both) of the arguments (L/R) is.
% This functionality is consistent with its use in NDLFTREDV.

%maxdim = max([szl(1) szr(2)]);

if szl(2)==szr(1)
   amaxrk = szl(2);
   maxdim = min([szl(1) szl(2) szr(2)]);
   if ndl==2
      Lnew = zeros([szl(1) maxdim szr(3:end)]);
      Rnew = zeros([maxdim szr(2) szr(3:end)]);
      npts = prod(szr(3:end));
      maxrk = 0;
      for i=1:npts
         [u,s,v] = svd(L*R(:,:,i));
         Lnew(:,:,i) = u*sqrt(s(1:szl(1),1:maxdim));
         Rnew(:,:,i) = sqrt(s(1:maxdim,1:szr(2)))*v';
         if min(size(s))>1
            ds = diag(s);
         else
            ds = s;
         end
         tmprk = LOCALrank(maxdim,ds,L,R(:,:,i));
         if tmprk>maxrk
            maxrk = tmprk;
         end
      end
   elseif ndr==2
      Lnew = zeros([szl(1) maxdim szl(3:end)]);
      Rnew = zeros([maxdim szr(2) szl(3:end)]);
      npts = prod(szl(3:end));
      maxrk = 0;
      for i=1:npts
         [u,s,v] = svd(L(:,:,i)*R);
         Lnew(:,:,i) = u*sqrt(s(1:szl(1),1:maxdim));
         Rnew(:,:,i) = sqrt(s(1:maxdim,1:szr(2)))*v';
         if min(size(s))>1
            ds = diag(s);
         else
            ds = s;
         end
         tmprk = LOCALrank(maxdim,ds,L(:,:,i),R);
         if tmprk>maxrk
            maxrk = tmprk;
         end
      end
   elseif isequal(szl(3:end),szr(3:end))
      Lnew = zeros([szl(1) maxdim szl(3:end)]);
      Rnew = zeros([maxdim szr(2) szr(3:end)]);
      npts = prod(szr(3:end));
      maxrk = 0;
      for i=1:npts
         [u,s,v] = svd(L(:,:,i)*R(:,:,i));
         Lnew(:,:,i) = u*sqrt(s(1:szl(1),1:maxdim));
         Rnew(:,:,i) = sqrt(s(1:maxdim,1:szr(2)))*v';
         if min(size(s))>1
            ds = diag(s);
         else
            ds = s;
         end
         % size(ds)
         tmprk = LOCALrank(maxdim,ds,L(:,:,i),R(:,:,i));
         if tmprk>maxrk
            maxrk = tmprk;
         end
      end
   end
   maxrk = min([maxrk amaxrk]);
   Lnew = Lnew(:,1:maxrk,:);
   Rnew = Rnew(1:maxrk,:,:);
   rk = maxrk;
else
   error('Invalid inner dimensions on L/R');
end

function rk = LOCALrank(maxdim,ds,L,R)
ztol = maxdim*eps*norm(L)*norm(R);
rk = sum(ds>ztol);

function out = LOCALisndssid(a)
% Copyright 2003-2004 The MathWorks, Inc.
sza = size(a);
if sza(1)==sza(2)
   npts = prod(sza(3:end));
   out = 1;
   cnt = 1;
   while cnt<=npts && out==1
      if norm(a(:,:,cnt)-diag(a(1,1,cnt)*ones(1,sza(1))),'fro')==0
         cnt = cnt + 1;
      else
         out = 0;
      end
   end
else
   out = 0;
end

% function F = LOCALisarbd(a,ny,nu,r)
% F = norm(a - kron(eye(r),a(1:ny,1:nu)),'fro')==0;
% % TODO this is finished, but not used yet
%
% function table = LOCALfsm(B,nr,r)
% % TODO finish this to deal with 2*f - f*2
% cnt = 0;
% table = zeros(0,3);
% %    [i j alpha] ->  Bi = alpha Bj
% for i=1:r-1
%    Bi = B((i-1)*nr+1:i*nr,:);
%    for j=i+1:r
%       Bj = B((j-1)*nr+1:j*nr,:);
%       alpha1 = Bj(:)\Bi(:);
%       if norm(Bj*alpha1-Bi)<eps(norm(Bi))*10
%          cnt = cnt + 1;
%          table(cnt,:) = [i j alpha1];
%       else
%          alpha2 = Bi(:)\Bj(:);
%          if norm(Bi*alpha2-Bj)<eps(norm(Bj))*10
%             cnt = cnt + 1;
%             table(cnt,:) = [j i alpha2];
%          end
%       end
%    end
% end


