function [index,blk] = mkBlkData(blk,fixBlkIdx)
% blk has 2 columns: 
%   1) Repeated real scalar:      [-sz 0] 
%   2) Repeated complex scalar:   [sz 0] 
%   3) Full complex block:        [szr szc]

%   Copyright 2010-2016 The MathWorks, Inc.
if nargin<2
   fixBlkIdx = zeros(0,1);  % fixed block indices (performance blocks)
end

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

% Create 3 column block structure from old 2 col structure
blk2 = blk;      
nblk = size(blk2,1);
blk3 = zeros(nblk,3);
for i1 = 1:nblk
    if blk2(i1,2) == 0 % repeated real or complex scalars
        blk3(i1,:) = [1 1 blk2(i1,1)];
    elseif blk2(i1,2)>0 % complex full block
        blk3(i1,:) = [blk2(i1,1:2) 1];
    else
        error('blk is invalid. First column of blk must be non-negative integers');
    end
end

% Get standard indices from blk2idx and then
% create additional indices/masks needed for mu
index = blk2idx(blk3); 

% allreal has scalar reals and then repeated reals             
index.allreal.num = 0;
index.allreal.rows = {};
index.allreal.cols = {};
index.allreal.origloc = zeros(0,1);
index.allreal.repeated = zeros(0,1);

for i=1:index.sreal.num
  index.allreal.num = index.allreal.num + 1;
  index.allreal.rows{index.allreal.num} = index.sreal.rows{i};
  index.allreal.cols{index.allreal.num} = index.sreal.cols{i};
  index.allreal.origloc = [index.allreal.origloc; index.sreal.origloc(i)];
  index.allreal.repeated = [index.allreal.repeated; 1];
end
for i=1:index.repreal.num
  index.allreal.num = index.allreal.num + 1;
  index.allreal.rows{index.allreal.num} = index.repreal.rows{i};
  index.allreal.cols{index.allreal.num} = index.repreal.cols{i};
  index.allreal.origloc = [index.allreal.origloc; index.repreal.origloc(i)];
  index.allreal.repeated = [index.allreal.repeated; index.repreal.repeated(i)];
end
index.allreal.realblk = blk2(index.allreal.origloc,:);

% allcomp has repeated complex scalars and then full complex blocks             
index.allcomp.num = 0;
index.allcomp.rows = {};
index.allcomp.cols = {};
index.allcomp.origloc = zeros(0,1);
index.allcomp.repeated = zeros(0,1);

for i=1:index.repcomp.num
  index.allcomp.num = index.allcomp.num + 1;
  index.allcomp.rows{index.allcomp.num} = index.repcomp.rows{i};
  index.allcomp.cols{index.allcomp.num} = index.repcomp.cols{i};
  index.allcomp.origloc = [index.allcomp.origloc; index.repcomp.origloc(i)];
  index.allcomp.repeated = [index.allcomp.repeated; index.repcomp.repeated(i)];
end
for i=1:index.full.num
  index.allcomp.num = index.allcomp.num + 1;
  index.allcomp.rows{index.allcomp.num} = index.full.rows{i};
  index.allcomp.cols{index.allcomp.num} = index.full.cols{i};
  index.allcomp.origloc = [index.allcomp.origloc; index.full.origloc(i)];
  index.allcomp.repeated = [index.allcomp.repeated; 1];
end
index.allcomp.compblk = blk2(index.allcomp.origloc,:);

% allrep has repeated reals, scalar reals and then repeated complex              
index.allrep.num = 0;
index.allrep.rows = {};
index.allrep.cols = {};
index.allrep.origloc = zeros(0,1);
index.allrep.repeated = zeros(0,1);

for i=1:index.repreal.num
  index.allrep.num = index.allrep.num + 1;
  index.allrep.rows{index.allrep.num} = index.repreal.rows{i};
  index.allrep.cols{index.allrep.num} = index.repreal.cols{i};
  index.allrep.origloc = [index.allrep.origloc; index.repreal.origloc(i)];
  index.allrep.repeated = [index.allrep.repeated; index.repreal.repeated(i)];
end
for i=1:index.sreal.num
  index.allrep.num = index.allrep.num + 1;
  index.allrep.rows{index.allrep.num} = index.sreal.rows{i};
  index.allrep.cols{index.allrep.num} = index.sreal.cols{i};
  index.allrep.origloc = [index.allrep.origloc; index.sreal.origloc(i)];
  index.allrep.repeated = [index.allrep.repeated; 1];
end
for i=1:index.repcomp.num
  index.allrep.num = index.allrep.num + 1;
  index.allrep.rows{index.allrep.num} = index.repcomp.rows{i};
  index.allrep.cols{index.allrep.num} = index.repcomp.cols{i};
  index.allrep.origloc = [index.allrep.origloc; index.repcomp.origloc(i)];
  index.allrep.repeated = [index.allrep.repeated; index.repcomp.repeated(i)];
end

% Return new locations  of the blocks
nr = index.allreal.num;
nf = index.full.num;
nrc = index.repcomp.num;
newloc = zeros(nblk,1);
newloc(index.allreal.origloc) = 1:nr;
newloc(index.full.origloc) = 1:nf;
newloc(index.repcomp.origloc) = 1:nrc;
index.newloc = newloc;

% Store all rows/cols for full, real, and repeated scalar blocks
index.full.allrows = [index.full.rows{:}]; %cell2mat(index.full.rows);
index.full.allcols = [index.full.cols{:}]; %cell2mat(index.full.cols);
index.allreal.allrows = [index.allreal.rows{:}]; %cell2mat(index.allreal.rows);
index.allreal.allcols = [index.allreal.cols{:}]; %cell2mat(index.allreal.cols);
index.allcomp.allrows = [index.allcomp.rows{:}]; %cell2mat(index.allcomp.rows);
index.allcomp.allcols = [index.allcomp.cols{:}]; %cell2mat(index.allcomp.cols);
index.allrep.allrows = [index.allrep.rows{:}]; %cell2mat(index.allrep.rows);
index.allrep.allcols = [index.allrep.cols{:}]; %cell2mat(index.allrep.cols);
index.repcomp.allrows = [index.repcomp.rows{:}]; %cell2mat(index.repcomp.rows);
index.repcomp.allcols = [index.repcomp.cols{:}]; %cell2mat(index.repcomp.cols);

% Form masks used to vectorize code
Mr = index.rdimm;
Mc = index.cdimm;

Drep_mask = zeros(Mr);
for i1=1:index.allrep.num
    tmprep = index.allrep.repeated(i1);
    tmpidx = index.allrep.rows{i1};
    Drep_mask(tmpidx,tmpidx) = ones(tmprep);
end
Drep_mask = Drep_mask(index.allrep.allrows,index.allrep.allrows);
index.masks.Drep_mask = Drep_mask;

Dfull_maskr = zeros(index.full.num,Mr);
Dfull_maskc = zeros(index.full.num,Mc);
Dfull_idxr = zeros(index.full.num,1);
for i1 = 1:index.full.num
    Dfull_idxr(i1) = index.full.rows{i1}(1);
    Dfull_maskr(i1,index.full.rows{i1}) = ones(1,index.full.dim(i1,2));
    Dfull_maskc(i1,index.full.cols{i1}) = ones(1,index.full.dim(i1,1));
end
Dfull_maskr = Dfull_maskr(:,index.full.allrows);
Dfull_maskc = Dfull_maskc(:,index.full.allcols);
index.masks.Dfull_maskr = Dfull_maskr;
index.masks.Dfull_maskc = Dfull_maskc;
index.masks.Dfull_idxr = Dfull_idxr;

% real_mask used in mmupiter
real_mask = zeros(index.allreal.num,Mr);
for i1=1:index.allreal.num
    tmprep = index.allreal.repeated(i1);
    tmpidx = index.allreal.rows{i1};
    real_mask(i1,tmpidx) = ones(1,tmprep);
end
real_mask = real_mask(:,index.allreal.allrows);
index.masks.real_mask = real_mask;
index.masks.G_mask = real_mask'*real_mask;

% repc_mask used in mmupiter
repc_mask = zeros(index.repcomp.num,Mr);
for i1=1:index.repcomp.num
    tmprep = index.repcomp.repeated(i1);
    tmpidx = index.repcomp.rows{i1};
    repc_mask(i1,tmpidx) = ones(1,tmprep);
end
repc_mask = repc_mask(:,index.repcomp.allrows);
index.masks.repc_mask = repc_mask;

% Mask for full blocks of Delta
index.masks.DeltaFull_mask = Dfull_maskc'*Dfull_maskr;

% Mask used in osborne's method
nd = length(index.allrep.allrows)+index.full.num;
ridxm = index.ridxm;
cidxm = index.cidxm;
sr = zeros(nd,Mr);
sc = zeros(Mc,nd);
sidx = 1;
cnt = 1;
for i1=1:size(blk,1)
  cdim = abs(blk(i1,1));
  rdim = blk(i1,2);
  idxr=ridxm(i1):(ridxm(i1+1)-1);
  idxc=cidxm(i1):(cidxm(i1+1)-1);
  if rdim==0 && cdim>1
    idx = sidx:(sidx+cdim-1);
    sr(idx,idxr) = eye(cdim);
    sc(idxc,idx) = eye(cdim);
    sidx = sidx+cdim;
  else
    if rdim==0
        rdim=1;
    else
        afullidx(cnt)=sidx;
        cnt = cnt+1;
    end
    sr(sidx,idxr) = ones(1,rdim);
    sc(idxc,sidx) = ones(cdim,1);
    sidx = sidx+1;
  end  
end
index.masks.sr=sr;
index.masks.sc=sc;
if index.full.num>0
    index.masks.afullidx = afullidx;
else
    index.masks.afullidx = [];
end

index.FVidx = LOCALFixedVaryIndex(index, fixBlkIdx);

% Determine Problem Type
% Note: If there is one fixed full complex blk and one varying full complex
% blk then we classify the problem as WCGain. This case could also be 
% classified as RobGain but WCGain takes advantage of LMI (not GEVP) UB.
fixedBlkIdx = index.FVidx.fixedBlkIdx;
varyBlkIdx = 1:nblk;
varyBlkIdx(fixedBlkIdx) = [];
allVary = isempty(fixedBlkIdx);
if allVary
    problemType = 'robstab';
elseif isscalar(varyBlkIdx) && (blk(varyBlkIdx,2)>0 || blk(varyBlkIdx,1)==1)
    problemType = 'wcgain';
    % problemType = 'general'; % XXX PJS Uncomment to revert back to original
elseif isscalar(fixedBlkIdx) && (blk(fixedBlkIdx,2)>0 || blk(fixedBlkIdx,1)==1)
    problemType = 'robgain';
else
    problemType = 'general';
end
index.problemType = problemType;



%----------------------------------------------------
function Y = LOCALFixedVaryIndex(index, fixedBlkIdx)
% All indices are row vectors
nblk = size(index.simpleblk,1);
nR = index.rdimm;
nC = index.cdimm;

fixedBlkIdx = reshape(fixedBlkIdx,[1 numel(fixedBlkIdx)]);
varyBlkIdx = 1:nblk;
varyBlkIdx(:,fixedBlkIdx) = [];

RealRows = sort(index.allreal.allrows);
RealCols = sort(index.allreal.allcols);
ComplexRows = sort(index.allcomp.allrows);
ComplexCols = sort(index.allcomp.allcols);

VaryRows = zeros(1,0);
VaryCols = zeros(1,0);
for i=1:numel(varyBlkIdx)
   % Referenced to Rows/Cols of M
   VaryRows = [VaryRows , index.ridxm(varyBlkIdx(i)):index.ridxm(varyBlkIdx(i)+1)-1];
   VaryCols = [VaryCols , index.cidxm(varyBlkIdx(i)):index.cidxm(varyBlkIdx(i)+1)-1];
end
FixedRows = 1:nR;
FixedRows(:,VaryRows) = [];
FixedCols = 1:nC;
FixedCols(:,VaryCols) = [];

if isempty(index.simpleblk)
   VaryCIdx = zeros(1,0);
   FixedCIdx = zeros(1,0);
else
   Cidx = find(index.simpleblk(:,1)>0);
   VaryCIdx = varyBlkIdx(:,ismember(varyBlkIdx,Cidx));
   FixedCIdx = fixedBlkIdx(:,ismember(fixedBlkIdx,Cidx));
end

FixedUnionRealCols = unique([FixedCols,RealCols]);
FixedUnionRealRows = unique([FixedRows,RealRows]);
VaryComplexCols = VaryCols(:,ismember(VaryCols,ComplexCols));
VaryComplexRows = VaryRows(:,ismember(VaryRows,ComplexRows));

cidxm = index.cidxm; % block col indices for M
VcIdx = zeros(1,0);
for i1=1:nblk
   if ~any(i1==fixedBlkIdx)
      VcIdx = [VcIdx,cidxm(i1):cidxm(i1+1)-1];
   end
end

Y = struct('VaryRows',VaryRows,...
   'VaryCols',VaryCols,...
   'fixedBlkIdx',fixedBlkIdx,...
   'FixedRows',FixedRows,...
   'FixedCols',FixedCols,...
   'VaryCIdx',VaryCIdx,...
   'FixedCIdx',FixedCIdx,...   
   'FixedUnionRealCols',FixedUnionRealCols,...
   'FixedUnionRealRows',FixedUnionRealRows,...
   'VaryComplexCols',VaryComplexCols,...
   'VaryComplexRows',VaryComplexRows,...
   'VcIdx',VcIdx);


