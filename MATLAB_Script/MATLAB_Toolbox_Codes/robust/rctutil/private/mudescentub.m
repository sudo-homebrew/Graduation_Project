function [ub,Dr,DcF,DcV,Gcr] = mudescentub(M,index,DGinit,opt)
% Based on dang.m by Young/Newlin
% Does not work for rep full although the mods should be straightforward
%
% Notation:  M'*Dr*M+j*(Gcr*M-M'*Gcr') - ub^2*Dc <= 0
% ub is an upper bound for mu

%   Copyright 2010-2016 The MathWorks, Inc.

% Argument Checks
if nargin==2
    DGinit = [];
    opt = [];
elseif nargin==3
    opt = [];
elseif nargin~=4
    ctrlMsgUtils.error('Robust:analysis:MudescentubIn4');
end

[Mr,Mc,AD] = size(M);  % M could be 3-D at this point

% Get indices from blk2idx plus a few other indices/masks needed for mu
% Also grab indices for real blocks only and for complex blocks only.
nreal = index.allreal.num;
realrows = index.allreal.allrows; 
realcols = index.allreal.allcols; 

nrep = index.allrep.num;
reprows = index.allrep.allrows; 
repcols = index.allrep.allcols; 

nfull = index.full.num;
fullrows = index.full.allrows;
fullcols = index.full.allcols;

% Get masks used to vectorize code
Drep_mask = index.masks.Drep_mask;
Dfull_maskr = index.masks.Dfull_maskr;
Dfull_maskc = index.masks.Dfull_maskc;
G_mask = index.masks.G_mask;
DfullIdxR = index.masks.Dfull_idxr;

VcIdx = index.FVidx.VcIdx;
FcIdx = setdiff((1:Mc)',VcIdx);

% Initialize Scalings
% The ub is computed below in the first pass through the descent
% for-loop.  Hence it does not need to be initialized.
if isempty(DGinit) || isinf(DGinit.ub) || rcond(DGinit.Dc)<eps
    Dr = eye(Mr);
    Dc = eye(Mc);
    Gcr = zeros(Mc,Mr);
else
    Dr = DGinit.Dr;
    Dc = DGinit.Dc;
    Gcr = DGinit.Gcr;
end
DrOld = Dr;
DcOld = Dc;
GcrOld = Gcr;

% Iteration options
if isempty(opt)
    descent_steps = 30;  % IMPSET XXX 
%   descent_steps = 400;   
    mincodel_steps = 30;  % IMPSET XXX 
%   mincodel_steps = 200;  
else
    descent_steps = opt.descent_steps;
    mincodel_steps = opt.mincodel_steps;
end
dtol = 1e-3;  % IMPSET XXX PJS Tolerance for D>=dtol*I

% Start Descent
GcrM = zeros(Mc,Mc,AD);
NL = zeros(Mc,Mc,AD);
DcV = zeros(Mc,Mc);
DcF = zeros(Mc,Mc);
DcV(VcIdx,VcIdx) = Dc(VcIdx,VcIdx);
DcF(FcIdx,FcIdx) = Dc(FcIdx,FcIdx);

for i=1:AD
   GcrM(realcols,:,i) = Gcr(realcols,realrows)*M(realrows,:,i);
   NL(:,:,i)	= M(:,:,i)'*Dr*M(:,:,i) + 1i*(GcrM(:,:,i)-GcrM(:,:,i)') - DcF;
end

for i1 = 1:descent_steps    
    % Find (clustered) max generalized eigenvals of:
    %    [M'*Dr*M+j*(Gcr*M-M'*Gcr')]*U = (ub^2*DcV + DcF)*U
    % ub is an upper bound and U is used to define Del
    allEVEC = cell(1,AD);
    allEVAL = cell(1,AD);
    ubsq = 0;
    for i2=1:AD
       [Ui,evals] = eig(NL(:,:,i2),DcV); % ? NL <= u^2 DcV
       evals = diag(evals);
       finiteIdx = ~isinf(evals);  % should always be 1:numel(VcIdx)
       allEVEC{i2} = Ui(:,finiteIdx);
       allEVAL{i2} = real(evals(finiteIdx));
       ubsq = max(ubsq, max(allEVAL{i2}));
    end
    
%     disp(i1)
%     LMIa = [M'*Dr*M+j*(Gcr*M-M'*Gcr')] - (ubsq*DcV + DcF);
%     max(real(eig(LMIa)))
    % Find evals and evecs clustered near max
    U = cell(1,AD);
    V = cell(1,AD);
    Uidx = [];
    NU = 0;
    for i2=1:AD    
        eidx = find(allEVAL{i2}>=0.975*ubsq); % IMPSET XXX 
        if ~isempty(eidx)
            Uidx = [Uidx, i2]; %#ok<AGROW>
            NU = NU + numel(eidx);
            U{i2} = allEVEC{i2}(:,eidx);
            V{i2} = M(:,:,i2)*U{i2};
        end
    end
    U = U(Uidx);
    V = V(Uidx);

    % New code start
    % Find (clustered) min eigenvalues of D-dtol*I
    [U2,evals2] = eig( Dr(reprows,reprows) );
    evals2 = real(diag(evals2));
    eidx2 = ( evals2 <= 2*dtol );
    U2p = U2(:,~eidx2);
    U2 = U2(:,eidx2);
    tmpfull = diag(Dr(DfullIdxR,DfullIdxR));
    eidx3 = find(  tmpfull - dtol <= 0);
    % New code end
    
    % Stop and return if upper bound has not significantly improved
    if ubsq<10*eps 
        ub = sqrt( max(0,ubsq) );
        return;
     elseif (i1==1) || (nstep>1e-4 && ubsq<=ubsqOld) % New code
        DrOld = Dr;
        DcOld = Dc;
        GcrOld = Gcr;
        ubsqOld = ubsq;
    elseif (ubsq<ubsqOld)
        ub = sqrt(ubsq);        
        % disp('Exit Small Step');
        return;
    else
        Dr = DrOld;
        Gcr = GcrOld;
        DcV = zeros(Mc,Mc);
        DcF = zeros(Mc,Mc);
        DcV(VcIdx,VcIdx) = DcOld(VcIdx,VcIdx);
        DcF(FcIdx,FcIdx) = DcOld(FcIdx,FcIdx);
        ub = sqrt(ubsqOld);
        % disp('Exit Cost Increase');
        return;
    end

    %----------------------------------------------------------------
    % Find min co(Del) which yields a descent direction.
    % Note: Del is a subset of a real inner product space which
    % is defined using a "compressed" form of the D/G matrices. 
    % See p.70 of Young's thesis for the definition of this space.    
    %----------------------------------------------------------------

    % First: Pick an initial update direction in co(Del):
    %         Dhat_0:=(Dhatrep,Dhatfull,Ghat)
    % Dhatrep: blks related to repeated scalar (real & complex) D-scales
    % Dhatfull: blks related to complex full D-scales
    % Ghat: blks related to real G-scales
    eta_dim = NU;
    eta = ones( eta_dim ,1);
    Ve = sum(V{1},2);
    Ue = sum(U{1},2);
    for i3 = 2:numel(U)
        Ve = Ve + sum(V{i3},2);
        Ue = Ue + sum(U{i3},2);
    end
    Ve = Ve/norm(eta);
    Ue = Ue/norm(eta);
    ubUe = Ue;
    ubUe(VcIdx) = sqrt(ubsq)*ubUe(VcIdx);
    
    Dhatrep = Ve(reprows)*Ve(reprows)'- ubUe(repcols)*ubUe(repcols)';
    Dhatrep = Dhatrep.*Drep_mask;    % zero out off-diagonal blocks

    Dhatfull = Dfull_maskr*real(Ve(fullrows).*conj(Ve(fullrows)));
    Dhatfull = Dhatfull - Dfull_maskc*real(ubUe(fullcols).*conj(ubUe(fullcols)));

    VeUe = Ve(realrows)*Ue(realcols)';
    Ghat = 1i*(VeUe-VeUe'); 
    Ghat = Ghat.*G_mask;  % zero out off-diagonal blocks  
 
    % Second: Iterate towards min co(Del) using algorithm by [GIL]    
    % If eta_dim=1 then no iteration is needed since Del is a single vector
    for i2 = 1:( mincodel_steps*(eta_dim>1) )
        % Given Dhat_i compute:
        %   P_i = arg min_{P \in Del} <Dhat_i,P>
        % where P_i:=(Prep,Pfull,H)
        Drstep = zeros(Mr);
        Drstep(reprows,reprows) = Dhatrep;
        Drstep(fullrows,fullrows) = diag(Dhatfull'*Dfull_maskr);
        Dcstep = zeros(Mc);
        Dcstep(repcols,repcols) = Dhatrep;
        Dcstep(fullcols,fullcols) = diag(Dhatfull'*Dfull_maskc);

        mineval = inf;
        for i3=1:numel(U)
            Ui = U{i3};
            ubUi = Ui;
            ubUi(VcIdx,:) = sqrt(ubsq)*ubUi(VcIdx,:);
            Vi = V{i3};
            uGMu = Ui(realcols,:)'*Ghat*Vi(realrows,:); 
            [eta,evals] = eig(Vi'*Drstep*Vi + 1i*(uGMu - uGMu') - (ubUi'*Dcstep*ubUi));
            evals = real(diag(evals));
            [~,minidx]=min(evals);
            if evals(minidx)<mineval
                mineval = evals(minidx);
                mineta = eta(:,minidx);
                minUidx = i3;
            end
        end
        eta = mineta/norm(mineta);
        Ve = V{minUidx}*eta;
        Ue = U{minUidx}*eta;
        ubUe = Ue;
        ubUe(VcIdx) = sqrt(ubsq)*ubUe(VcIdx);

        Prep = Ve(reprows)*Ve(reprows)' - ubUe(repcols)*ubUe(repcols)';
        Prep = Prep.*Drep_mask;    % zero out off-diagonal blocks

        Pfull = Dfull_maskr*real(Ve(fullrows).*conj(Ve(fullrows)));
        Pfull = Pfull - Dfull_maskc*real(ubUe(fullcols).*conj(ubUe(fullcols)));

        VeUe = Ve(realrows)*Ue(realcols)';
        H = 1i*(VeUe-VeUe'); 
        H = H.*G_mask;  % zero out off-diagonal blocks  

        % Take a step:
        %   Dhat_{i+1} = min co{Dhat_i,P_i}
        %              = Dhat_i + t*(P_i-Dhat_i)
        % where t = -<Dhat_i,P_i-Dhat_i> / <P_i-Dhat_i,P_i-Dhat_i>
        Deltarep = Prep-Dhatrep;
        Deltafull = Pfull - Dhatfull;
        DeltaG = H-Ghat;
        n=0;
        d=0;
        if nrep>0         
            n = n + sum(sum(Dhatrep.*(Deltarep.')));
            d = d + sum(sum(Deltarep.*(Deltarep.')));
        end
        if nreal>0
            n = n + sum(sum( Ghat.*(DeltaG')));
            d = d + sum(sum(DeltaG.*(DeltaG')));
        end
        if nfull>0            
            n = n + Dhatfull'*Deltafull;
            d = d + Deltafull'*Deltafull;
        end
        
        if d<=1e-6
            t = (n<0); % XXX MINOR:  Just set t=0?
        else
            t = min([max([-real(n/d) 0]) 1]);
        end                    
        Dhatrep = Dhatrep + t*Deltarep;
        Dhatfull = Dhatfull + t*Deltafull;
        Ghat = Ghat + t*DeltaG;        
    end % for loop for [GIL]'s algorithm  

    % Project Dhatrep and Dhatfull onto D>=dtol*I
    if ~isempty(U2)
        [QQ,ee] = eig( U2'*Dhatrep*U2  );
        ee = real(diag(ee));
        ee( ee>0 ) = 0;        
        Dhatrep = U2p*(U2p'*Dhatrep*U2p)*U2p' + ...
            U2*( QQ*diag(ee)*QQ') *U2';
    end
    if ~isempty(eidx3)
        Dhatfull(eidx3) = min(Dhatfull(eidx3),0);
    end
    
    % Third: Convert into full D/G matrices.  This is a descent direction.
    Drstep = zeros(Mr);
    Drstep(reprows,reprows) = Dhatrep;
    Drstep(fullrows,fullrows) = diag(Dhatfull'*Dfull_maskr);
    Dcstep = zeros(Mc);
    Dcstep(repcols,repcols) = Dhatrep;
    Dcstep(fullcols,fullcols) = diag(Dhatfull'*Dfull_maskc);
    Gcrstep = zeros(Mc,Mr);
    Gcrstep(realcols,realrows) = Ghat;

%     if i1==1
%        DrstepPrev = Drstep;
%        DcstepPrev = Dcstep;
%        GcrstepPrev = Gcrstep;
%     else
%        fac = 1;
%        Drstep = fac*Drstep + (1-fac)*DrstepPrev;
%        Dcstep = fac*Dcstep + (1-fac)*DcstepPrev;
%        Gcrstep = fac*Gcrstep + (1-fac)*GcrstepPrev;
%        DrstepPrev = Drstep;
%        DcstepPrev = Dcstep;
%        GcrstepPrev = Gcrstep;
% %        Drstep = Drstep2;
% %        Dcstep = Dcstep2;
% %        Gcrstep = Gcrstep2;
%     end
    
    % Fourth: Take a step in the descent direction
    GcrstepM = zeros(Mc,Mc,AD);
    NLstep = zeros(Mc,Mc,AD);
    ALLeig = [];
    
    DcVstep = zeros(Mc,Mc);
    DcFstep = zeros(Mc,Mc);
    DcVstep(VcIdx,VcIdx) = Dcstep(VcIdx,VcIdx);
    DcFstep(FcIdx,FcIdx) = Dcstep(FcIdx,FcIdx);

    for i2=1:AD
       GcrstepM(realcols,:,i2) = Gcrstep(realcols,realrows)*M(realrows,:,i2);             
       NLstep(:,:,i2)	= M(:,:,i2)'*Drstep*M(:,:,i2) + ...
          1i*(GcrstepM(:,:,i2)-GcrstepM(:,:,i2)') - DcFstep;
       % For positive t, ubsq(t) initially decreases and then may
       % increase. The smallest value of t for which ubsq(t)=
       % ubsq(0) is given by the smallest strictly positive
       % generalized eigval of (NL-ubsq*Dc,NLstep-ubsq*Dcstep).
       % Choose the step length to be half this value.
       tmpeig = real(eig(NL(:,:,i2)-ubsq*DcV,NLstep(:,:,i2)-ubsq*DcVstep));
       ALLeig = [ALLeig;tmpeig]; %#ok<AGROW> % vertical
    end
    if any(isnan(ALLeig))
       t = 0;
    else
       ALLeig = sort(ALLeig);
       mineig = min(abs(ALLeig));  % should be some(?) at 0;
       idx = find(abs(ALLeig)==mineig);
       if idx(end)+1<=numel(ALLeig) % && mineig<1e-6
          t = ALLeig(idx(end)+1)/2;
       else
          t = inf;
       end
    end

    if rcond(Dcstep)<eps || rcond(Dc-dtol*eye(Mc))<eps
        tmpeig=real(eig(Dcstep,Dc-dtol*eye(Mc)));
    else
        tmp = (Dc-dtol*eye(Mc))\Dcstep;
        tmpeig = real(eig(tmp));
    end
    if any(isnan(tmpeig))
       tmax = 0;
    else
       tmax = 1/max( [tmpeig; 1e-9] );
    end
    nstep = norm([Dcstep(:); Gcrstep(:)]);

%    fprintf('mudes: i1=%d \t t=%0.3f \t tmax=%0.3f \t ubsq=%0.5f \n',i1,t,tmax,ubsq);
    
    % Take step 
    t = min([t tmax]);
    NL = NL-t*NLstep;  
    Gcr = Gcr-t*Gcrstep;
    Dr = Dr-t*Drstep;
    Dc = Dc-t*Dcstep;

    % WLOG we can scale max entry of Dc to be one.
    dmax = max(max(Dc));
    dmax = 1; % XXX, April 2016
    NL = NL/dmax;
    Gcr = Gcr/dmax;
    Dr = Dr/dmax;
    Dc = Dc/dmax;
    DcV = zeros(Mc,Mc);
    DcF = zeros(Mc,Mc);
    DcV(VcIdx,VcIdx) = Dc(VcIdx,VcIdx);
    DcF(FcIdx,FcIdx) = Dc(FcIdx,FcIdx);
        
    
    if rcond(Dc)<sqrt(eps)  % eps  XXX, 2016 
        Dr = DrOld;
        Gcr = GcrOld;
        DcV = zeros(Mc,Mc);
        DcF = zeros(Mc,Mc);
        DcV(VcIdx,VcIdx) = DcOld(VcIdx,VcIdx);
        DcF(FcIdx,FcIdx) = DcOld(FcIdx,FcIdx);
        ub = sqrt(ubsqOld);
        % disp('Exit IllConditioned');
        return;
    end    
end % while loop for gradient descent

% Check the results of the last iteration
ALLubsq = zeros(1,AD);
for i2=1:AD
   ALLubsq(i2) = max(real(eig(NL(:,:,i2),DcV)));
end
ubsq = max(ALLubsq);

if ubsq<ubsqOld
    ub = sqrt(ubsq);
else    
    Dr = DrOld;
    DcV = zeros(Mc,Mc);
    DcF = zeros(Mc,Mc);
    DcV(VcIdx,VcIdx) = DcOld(VcIdx,VcIdx);
    DcF(FcIdx,FcIdx) = DcOld(FcIdx,FcIdx);
    Gcr = GcrOld;
    ub = sqrt(ubsqOld);
end
