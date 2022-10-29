function [dMd,dr,dci] = osbal(M,index,opt,CondNumberBound)
% function [dMd,dr,dci] = osbal(M,index,opt)
% 
% Let DD denote the (nonsingular, but not necessarily pos.def.)
% D-scale set associated with the uncertainty structure in index.
% This function constructs D-scales via a modified Osborne's method
% which solve inf_{dr/dc in DD} ||dr*M*inv(dc)||_F.  The scaled matrix
% dMd := dr*M*inv(dc) is returned along with dr and dci:=inv(dc).
%
% If opt='d', the D-scales are restricted to be diagonal (but still
% in DD) and the standard Osborne's iteration is used. 
%
% If opt='f', the D-scales are only restricted to lie in DD, i.e.
% they can have full-blocks if there are repeated real / complex
% blocks.  (default: opt = 'f')

%   Copyright 1986-2020 The MathWorks, Inc.

% This code is an updated version of deebal.m with minor
% modifications.
ni = nargin;
if ni<3 || isempty(opt)
   opt='f';
end
if ni<4 || isempty(CondNumberBound)
   CondNumberBound = 1e10;
end
   
% Get indices for repeated real/complex
% blk = index.simpleblk;
nrep = index.allrep.num;
nfull = index.full.num;
reprows = index.allrep.allrows;
repcols = index.allrep.allcols;
repeated = index.allrep.repeated;
repidx =  cumsum([1; repeated]); 


% Get Masks: sr/sc used to vectorize code
[Mr,Mc] = size(M);
nd = length(reprows)+nfull;
sr = index.masks.sr;
sc = index.masks.sc;
afullidx=index.masks.afullidx;


% Stopping Conditions
if opt=='f' && any(repeated>1)
  userot = 1;
  mxrotiter = 10;
else
  userot = 0;
  mxrotiter = 0;
end
reltol = 2e-3;  
mxosbiter = 30;
reltol2 = 1e-4; 

% Initialize
dMd = M;
dr = eye(Mr);
dci = eye(Mc);
drot = eye(length(reprows));
dvec = ones(1,nd);
rotcnt = 0;
cost = norm(dMd,'fro');
oldcost = max([2*cost 10*eps]);
go = 1;

while (go==1) && (rotcnt <= mxrotiter) && ( reltol*oldcost < (oldcost-cost) )
  rotcnt = rotcnt + 1;

  % Do a rotation using full blocks: 
  %  Leaves ||dMd||_F unchanged, but improves the effect
  %  of the diagonal scalings below
  if userot
    a=dMd(reprows,:)*dMd(reprows,:)'-dMd(:,repcols)'*dMd(:,repcols);
    for i = 1:nrep
      idx = repidx(i):(repidx(i+1)-1);
      [drot(idx,idx),~,~] = svd(a(idx,idx));
    end
    dr(reprows,:) = drot'* dr(reprows,:);
    dci(:,repcols) = dci(:,repcols)*drot;
    dMd(reprows,:) = drot'*dMd(reprows,:);
    dMd(:,repcols) = dMd(:,repcols)*drot;
  end	
    
  % Get diagonal scaling using Osborne's iteration
  a = sr*real(conj(dMd).*dMd)*sc;    
  for i1 = 1:nfull
    ridx = index.full.rows{i1};
    afullr = afullidx(i1);
    for i2 = 1:nfull
        cidx = index.full.cols{i2};
        afullc = afullidx(i2);
        a(afullr,afullc) = norm( dMd(ridx,cidx) )^2;
    end
  end
  a = a - diag(diag(a));			
  d = ones(1,nd);
  cost2 = sum(sum(a));			
  oldcost2 = max([2*cost2 10*eps]);
  itcnt = 0;
  astart = a; 
  while (itcnt < mxosbiter) && reltol2*oldcost2<(oldcost2-cost2)      
    sa = max(sum(a,1),10*eps);
    sat = max(sum(a,2),10*eps)';
    d = d.*sqrt(sqrt(sa./sat));
    d = min(max(d,1e-8),1e8);    
    a = astart.*(d'*(1 ./d));
    itcnt = itcnt+1;
    oldcost2 = max([cost2 10*eps]);		
    cost2 = sum(sum(a));
  end     
  
  % Store scalings          
  d = sqrt(d/d(1,1));
  dvec = dvec.*d;  
    
  drvec = diag(sr'*diag(d)*sr);
  dcivec = diag(sc*diag(1./d)*sc')';

  drmat = repmat(drvec,[1 Mc]);
  dcimat = repmat(dcivec,[Mr,1]);
  dMd = drmat.*dMd.*dcimat;

  drmat = repmat(drvec,[1 Mr]);
  dcimat = repmat(dcivec,[Mc,1]);
  dr  = drmat.*dr;
  dci = dci.*dcimat;
  
  oldcost = max([cost 10*eps]);
  cost = norm(dMd,'fro');
      
  % Stop the iteration if D becomes ill-conditioned  
  % XXX: deebal.m does not stop the iteration if D 
  % becomes ill-conditioned, but does give a warning 
  % if this happens.
  dcond = max(dvec)/min(dvec); % max(min(dvec),10*eps);
  if dcond>CondNumberBound
      go = 0;
  end

end   