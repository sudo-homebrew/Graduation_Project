function [wcval,pertsreal,pertrepreal,pertLvec,pertRvec,pertrepcomp,initcost] = ...
   wclowc(m,index,NTIMES,MAXCNT,mMXGAIN,aMXGAIN,RNG)
%   Worst-case over unit cube, unit-ball.  Can accomodate repeated
%   real scalars, and/or repeated complex scalars, and full complex blocks.
%   This allows for
%   FULL, SREAL, and REPREAL.  INDEX is created by the
%   subroutine BLK2IDX.  PERTSREAL and PERTREPREAL are DOUBLEs.
%   PERTFULL is a CELL-ARRAY.  WCVAL is a single DOUBLE.

%   Copyright 2003-2012 The MathWorks, Inc.

%   ONELOOPI
if nargin==2
   NTIMES = [];
   MAXCNT = [];
   mMXGAIN = [];
   aMXGAIN = [];
elseif nargin==3
   MAXCNT = [];
   mMXGAIN = [];
   aMXGAIN = [];
elseif nargin==4
   mMXGAIN = [];
   aMXGAIN = [];
elseif nargin==5
   aMXGAIN = [];
end

if isempty(NTIMES)
   NTIMES = 1;
end
if isempty(MAXCNT)
   MAXCNT = 1;
end
if isempty(mMXGAIN)
   mMXGAIN = 2;
end
if isempty(aMXGAIN)
   aMXGAIN = 3;
end

dims = ndims(m);
if dims>2
   error('M must be 2-dimensional');
end

if index.repfull.num>0
   error('WCLOW cannot accept complex, repeated fulls');
else
   comprows = [];
   compcols = [];
   cblk = [];
   for i=1:index.full.num
      comprows = [comprows index.full.rows{i}]; %#ok<*AGROW>
      compcols = [compcols index.full.cols{i}];
      cblk = [cblk;index.full.dim(i,:)];
   end
   for i=1:index.repcomp.num
      comprows = [comprows index.repcomp.rows{i}];
      compcols = [compcols index.repcomp.cols{i}];
      cblk = [cblk;index.repcomp.repeated(i) 0];
   end
end

szm = size(m);
ne = szm(1) - index.rdimm;
nd = szm(2) - index.cdimm;
rsel22 = index.rdimm+1:szm(1);
csel22 = index.cdimm+1:szm(2);

% Initialize worst-case gain (at center)
wcval = norm(m(rsel22,csel22));
initcost = wcval;
MXGAIN = aMXGAIN + mMXGAIN*initcost;
pertsreal = zeros(index.sreal.num,1);
pertrepreal = zeros(index.repreal.num,1);
pertLvec = {};
pertRvec = {};
for i = 1:index.full.num
   pertLvec{i} = zeros(index.full.dim(i,1),1);
   pertRvec{i} = zeros(1,index.full.dim(i,2));
   lvec{i} = zeros(index.full.dim(i,1),1);
   rvec{i} = zeros(1,index.full.dim(i,2));
end
for i=1:index.repcomp.num
   lvec{index.full.num+i} = 0;
   rvec{index.full.num+i} = 0;
end
pertrepcomp = zeros(index.repcomp.num,1);

% Lvecsreal = -ones(index.sreal.num,1);
% Lvecrepreal = -ones(index.repreal.num,1);
% Uvecsreal =  ones(index.sreal.num,1);
% Uvecrepreal =  ones(index.repreal.num,1);
Lvecsrealorig = -ones(index.sreal.num,1);
Lvecreprealorig = -ones(index.repreal.num,1);
Uvecsrealorig =  ones(index.sreal.num,1);
Uvecreprealorig =  ones(index.repreal.num,1);

j = 0;
cont2go=1;
while j<NTIMES && cont2go==1
   j = j+1;
   
   % create random starting point
   deltasreal   = Lvecsrealorig + rand(RNG,index.sreal.num,1).*(Uvecsrealorig-Lvecsrealorig);
   deltarepreal = Lvecreprealorig ...
      + rand(RNG,index.repreal.num,1).*(Uvecreprealorig-Lvecreprealorig);
   
   mloop = m;
   mlooporig = m;
   
   % move to starting point
   for i=1:index.sreal.num
      mloop = oneloopi(mloop,index.sreal.rows{i},index.sreal.cols{i},deltasreal(i));
   end
   for i=1:index.repreal.num
      mloop = oneloopi(mloop,index.repreal.rows{i},index.repreal.cols{i},deltarepreal(i));
   end
   gain = norm(mloop(rsel22,csel22));
   
   % Update bounds to reflect starting point
   Lvecsreal = Lvecsrealorig - deltasreal;
   Uvecsreal = Uvecsrealorig - deltasreal;
   Lvecrepreal = Lvecreprealorig - deltarepreal;
   Uvecrepreal = Uvecreprealorig - deltarepreal;
   
   mdchange = 1;
   cnt = 0;
   % notinf = 1;
   go = 1;
   while (mdchange==1 && cnt<MAXCNT && gain<MXGAIN && go==1) || cnt==0
      cnt = cnt + 1;
      mdchange = 0;
      % Create random cycling order each time through
      [~,oidxsreal] = sort(rand(RNG,index.sreal.num,1));
      [~,oidxrepreal] = sort(rand(RNG,index.repreal.num,1));
      gain0 = gain;
      ii = 1;
      an = [];
      while ii<=index.sreal.num && gain<MXGAIN
         i = oidxsreal(ii);
         % blkd = 1;
         tmpm = mloop([index.sreal.rows{i} rsel22],[index.sreal.cols{i} csel22]);
         [gain,adjustment] = wcslft(tmpm,Lvecsreal(i),Uvecsreal(i),1,ne,nd);
         if adjustment~=0
            an = [an abs(adjustment)];
            deltasreal(i) = deltasreal(i) + adjustment;
            Lvecsreal(i) = -1 - deltasreal(i);
            Uvecsreal(i) = 1 - deltasreal(i);
            mdchange = 1;
            if ~isinf(gain)
               mloop = oneloopi(mloop,index.sreal.rows{i},index.sreal.cols{i},adjustment);
            end
         end
         ii = ii + 1;
      end
      gain1 = gain;
      ii = 1;
      while ii<=index.repreal.num && gain<MXGAIN
         i = oidxrepreal(ii);
         blkd = index.repreal.repeated(i);
         tmpm = mloop([index.repreal.rows{i} rsel22],[index.repreal.cols{i} csel22]);
         [gain,adjustment] = wcslft(tmpm,Lvecrepreal(i),Uvecrepreal(i),blkd,ne,nd);
         if adjustment~=0
            an = [an abs(adjustment)];
            deltarepreal(i) = deltarepreal(i) + adjustment;
            Lvecrepreal(i) = -1 - deltarepreal(i);
            Uvecrepreal(i) = 1 - deltarepreal(i);
            mdchange = 1;
            if ~isinf(gain)
               mloop = oneloopi(mloop,index.repreal.rows{i},index.repreal.cols{i},adjustment);
            end
         end
         ii = ii + 1;
      end
      gain2 = gain;
      if gain>wcval
         pertsreal = deltasreal;
         pertrepreal = deltarepreal;
         % Preserve order here, based on original pullapart above
         for i=1:index.full.num
            pertLvec{i} = lvec{i};
            pertRvec{i} = rvec{i};
         end
         pertrepcomp = zeros(index.repcomp.num,1);
         for i=1:index.repcomp.num
            pertrepcomp(i) = lvec{i+index.full.num}*rvec{i+index.full.num};
         end
         wcval = gain;
      end
      if gain<MXGAIN && ~isempty(cblk)
         % this is a bit anal - we start from scratch, and reabsorb all
         % of the real blocks.
         mloop = mlooporig;
         for i=1:index.sreal.num
            mloop = oneloopi(mloop,index.sreal.rows{i},index.sreal.cols{i},deltasreal(i));
         end
         for i=1:index.repreal.num
            mloop = oneloopi(mloop,index.repreal.rows{i},index.repreal.cols{i},deltarepreal(i));
         end
         tmpm = mloop([comprows rsel22],[compcols csel22]);
         [lvec,rvec,~,alpha,~,DeltaCell] = onepiter(tmpm,cblk,ne,nd,RNG);
         if alpha>0
            % Preserve order here, based on original pullapart above
            for i=1:index.full.num
               mloop = oneloopi(mloop,index.full.rows{i},index.full.cols{i},DeltaCell{i});
            end
            for i=1:index.repcomp.num
               ifix = i + index.full.num;
               mloop = oneloopi(mloop,index.repcomp.rows{i},index.repcomp.cols{i},DeltaCell{ifix});
            end
            gain = norm(mloop(rsel22,csel22));
            %mls = mloop;
            %for i=1:index.repcomp.num
            %    ifix = i + index.full.num;
            %    mls = oneloopi(mls,index.repcomp.rows{i},index.repcomp.cols{i},-DeltaCell{ifix});
            %    tmpm = mls([index.repcomp.rows{i} rsel22],[index.repcomp.cols{i} csel22]);
            %    [mxgain,maxloc] = wcslftcs(tmpm,index.repcomp.repeated(i),ne,nd);
            %    if mxgain>gain
            %        gain = mxgain;
            %        DeltaCell{ifix} = maxloc;
            %    end
            %end
         else
            gain = inf;
         end
      end
      gain3 = gain; % may be smaller than gain 2, since power method does not always increase
      glist = [gain0 gain1 gain2 gain3];
      %[j gain min(diff(glist)) max(diff(glist))]
      %[glist]
      if gain>wcval
         pertsreal = deltasreal;
         pertrepreal = deltarepreal;
         % Preserve order here, based on original pullapart above
         for i=1:index.full.num
            pertLvec{i} = lvec{i};
            pertRvec{i} = rvec{i};
         end
         pertrepcomp = zeros(index.repcomp.num,1);
         for i=1:index.repcomp.num
            pertrepcomp(i) = lvec{i+index.full.num}*rvec{i+index.full.num};
         end
         wcval = gain;
      end
      if max(diff(glist))<0.0001*gain
         go = 0;
      end
   end
   
   if gain>=MXGAIN
      cont2go=0;
   end
end
%    wcval = center
%    i=1:TRYS
%        GO=1
%        gain = random point
%        j=1:MAXCNT & GO==1
%            update 1
%            update 2
%            if > wcval
%                save everything
%            end
%            update 3
%            if > wcval
%                save everything
%            end
%            if no progress, GO=0;
