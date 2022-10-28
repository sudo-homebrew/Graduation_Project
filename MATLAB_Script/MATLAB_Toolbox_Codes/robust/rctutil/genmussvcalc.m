function [bnd,q] = genmussvcalc(m,c,blk,opt)
%

%   Copyright 2003-2011 The MathWorks, Inc.

if nargin == 3
    opt = 'n';
end
bnd = []; q = []; %dd = []; gg = [];

mumexstatus1 = exist('amevlp','file');
mumexstatus2 = exist('amibndv','file');
mumexstatus3 = exist('amilowxy','file');
mumexstatus4 = exist('amiuppxy','file');
if mumexstatus1~=3 || mumexstatus2~=3 || mumexstatus3~=3 || mumexstatus4~=3
	disp('genmussvcalc Warning: MEX functions are unavailable.');
	return
end

diagnose = goptvl(opt,'d',1,-1);
clevel = goptvl(opt,'C',1,1);

if isa(m,'double') && isa(c,'double')
   szm = size(m); % nr = szm(1); nc = szm(2);
   %szc = size(c); cr = szc(1); cc = szc(2);
   if length(szm)==2
      opt = [opt 's'];
   end
   md = m;
   cd = c;
   tflag = 'dd';
elseif  isa(m,'frd') && isa(c,'double')
   % szm = size(m); nr = szm(1); nc = szm(2);
   szc = size(c); %cr = szc(1); cc = szc(2);   
   md = freqresp(m,m.Frequency,m.FrequencyUnit);
   nfreq = length(m.Frequency);
   cd = repmat(permute(c,[1 2 length(szc)+1 3:length(szc)]),[1 1 nfreq ones(1,length(szc)-2)]);
   tflag = 'fd';
elseif  isa(m,'double') && isa(c,'frd')
   szm = size(m); %nr = szm(1); nc = szm(2);
   %szc = size(c); cr = szc(1); cc = szc(2);
   tflag = 'fd';
   cd = freqresp(c,c.Frequency,c.FrequencyUnit);
   nfreq = length(c.Frequency);
   md = repmat(permute(m,[1 2 length(szm)+1 3:length(szm)]),[1 1 nfreq ones(1,length(szm)-2)]);
elseif  isa(m,'frd') && isa(c,'frd')
   if c.Ts==m.Ts || (c.Ts==-1 && m.Ts~=0) || (m.Ts==-1 && c.Ts~=0)
      %szm = size(m); nr = szm(1); nc = szm(2);
      %szc = size(c); cr = szc(1); cc = szc(2);
      if ~isequal(m.FrequencyUnit,c.FrequencyUnit)
         c = chgFreqUnit(c,m.FrequencyUnit);
      end
      if isequal(m.Frequency,c.Frequency)
         cd = freqresp(c,c.Frequency,c.FrequencyUnit);
         md = freqresp(m,m.Frequency,m.FrequencyUnit);
      else
         error('M and C have different Frequency vectors.');
      end
      tflag = 'ff';
   else
      error('M and C must have the same SampleTime');
   end
else
   error('M and C must be DOUBLEs or FRDs');
end
szmd = size(md); nr = szmd(1); nc = szmd(2);
szcd = size(cd); cr = szcd(1); cc = szcd(2);
[nblk,dum] = size(blk);
if dum~=2
    error('Invalid Block Structure');
end
ablk = abs(blk);
sblk = find(ablk(:,2)==0);
if ~isempty(sblk)
    ablk(sblk,2) = ablk(sblk,1);
end
if nblk>1
    dims = sum(ablk);
else
    dims = ablk;
end
if dims(1)~=nc || dims(2)~=nr || nc~=cc
    error('Inconsistent Matrix/Delta dimensions')
end

ned = max([length(szmd) length(szcd)]) - 2;
szmd = [szmd ones(1,ned+2-length(szmd))];
szcd = [szcd ones(1,ned+2-length(szcd))];
exd = max([szmd(3:end);szcd(3:end)],[],1);

if all(szmd(3:end)==1 | szmd(3:end)==exd) && ...
  all(szcd(3:end)==1 | szcd(3:end)==exd)
	md = repmat(md,[1 1 exd./szmd(3:end)]);
	cd = repmat(cd,[1 1 exd./szcd(3:end)]);
else
   error('Invalid extra dimensions.');
end

q = zeros([nr cr exd]);
bnd = zeros([1 1 exd]);
ngpts = prod(exd);
if ~any(opt=='s')
   fprintf('Points completed: ');
   lptxt = '';
end
for i=1:ngpts
   [singleQ,singleGamma] = LOCALgenmussvcalc(md(:,:,i),cd(:,:,i),blk,diagnose,clevel);
   q(:,:,i) = singleQ;
   bnd(:,:,i) = singleGamma;
   if ~any(opt=='s')
      fprintf(repmat('\b',[1 length(lptxt)]))
      lptxt = sprintf('%d/%d',i,ngpts);
      fprintf(lptxt);
   end
end
switch tflag
   case {'fd' 'ff'}
      q = frd(q,m.Frequency,m.Ts,'FrequencyUnit',m.FrequencyUnit);
      bnd = frd(bnd,m.Frequency,m.Ts,'FrequencyUnit',m.FrequencyUnit);
   case {'df'}
      q = frd(q,c.Frequency,c.Ts,'FrequencyUnit',c.FrequencyUnit);
      bnd = frd(bnd,c.Frequency,c.Ts,'FrequencyUnit',c.FrequencyUnit);
   otherwise
end
if ~any(opt=='s')
   fprintf('\n')
end

function [q,bnd] = LOCALgenmussvcalc(m,c,blk,diagnose,clevel)
szm = size(m); nr = szm(1); nc = szm(2);
if diagnose>=1
   [bnd,~,dac,dar,gacr,garc] = gebubd(m,c,blk,diagnose,clevel);
else
   [bnd,~,dac,dar,gacr,garc] = gebub(m,c,blk,diagnose,clevel);
end
bbnd = bnd;
if bnd==0
   bbnd = 1e-6;
end
[dd,gg] = a2ynrow(dar,dac,garc,gacr,blk,bbnd);
if any(blk(:,1)<0)
   [dl,dr,gl,gm,gr] = mussvunwrap(dd,gg,blk);
   el = eye(nr);
   er = eye(nc);
   ll = inv(sqrtm(sqrtm(el+gl*gl)));
   rr = inv(sqrtm(sqrtm(er+gr*gr)));
   newv = c/dr*rr;
   newr = ll*( (1/bbnd)*dl*m/dr-sqrt(-1)*gm )*rr;
   newu = (1/bbnd)*ll*dl;
   [q,~] = ruqvsol(newr,newu,newv);
else
   [dl,dr] = mussvunwrap(dd,blk);
%    el = eye(nr);
%    er = eye(nc);
   newv = c/dr;
   newr = dl*m/dr;
   newu = dl;
   [q,~] = ruqvsol(newr,newu,newv);
end
