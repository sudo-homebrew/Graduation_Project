function [P,BLK,BLKnames] = musynData(P0,nY,nU)
% Computes SS or GENSS P and normalized uncertainty BLK such that
%   lft(DELTA_PERF,P0) = lft(BLK,P)
% where DELTA_PERF is the performance block associated with
% the I/O channels of P0.
%
% BLK is the Nx2 description of DELTA and includes the performance 
% block DELTA_PERF. The blocks are sorted so that the largest 
% unstructured block appears last.

%   Copyright 2019 The MathWorks, Inc.

% P0 has inputs [D;U] and outputs [E;Y]
[rs,cs] = iosize(P0);
nE = rs-nY;  
nD = cs-nU;

% Get LFT model
[D,~,~,muInfo] = ulftdata2(P0.Data_);
BLK = muInfo.muBlkNby2;
% Add performance channel
BLK = [BLK; nD nE]; 
BLKnames = {muInfo.BlkInfo.BlockName 'Performance'}';

% Sort blocks so that the largest unstructured block comes last
% (helps reduces order of D(s) after normalization)
nblk = size(BLK,1);
ix = find(BLK(:,2)>0);
[ordmax,imax] = max(sum(BLK(ix,:),2));  % order=rowsize+colsize
imax = ix(imax);
if ~(isempty(imax) || ordmax==nD+nE)
   % Reorder blocks
   bdim = abs(BLK);
   ix = find(bdim(:,2)==0);
   bdim(ix,2) = bdim(ix,1);
   bstart = sum(bdim(1:imax-1,:));
   i1 = bstart(1);  j1 = bstart(2);
   i2 = i1+bdim(imax,1);  j2 = j1+bdim(imax,2);
   % D maps [uBLK ; u ; uTUNED] to [yBLK ; y ; yTUNED].
   % Reordering should only affect uBLK and yBLK
   [rs,cs] = iosize(D.IC);
   rperm = 1:rs;    nyBLK = sum(bdim(:,2));
   rperm(1:nyBLK) = [1:j1,j2+1:nyBLK,j1+1:j2];
   cperm = 1:cs;    nuBLK = sum(bdim(:,1));
   cperm(1:nuBLK) = [1:i1,i2+1:nuBLK,i1+1:i2];
   D.IC = ioperm(D.IC,rperm,cperm);
   perm = [1:imax-1,imax+1:nblk,imax];
   BLK = BLK(perm,:);
   BLKnames = BLKnames(perm,:);
end

if isempty(D.Blocks)
   P = ss.make(ss(D));
else
   P = genss.make(D);
end