function   [lb,Delta,deltareal] = muwcglb(M,index,deltareal0,bnds,RNG,opt)
% New lower bound algorithm based on wcgain. The algorithm partitions
% the block structure into real (Deltar) and complex (Deltac) parts. The
% incoming matrix is partitioned into a 2x2 block conformably with
% this block structure partition: M = [Mrr Mrc; Mcr Mcc].
%
% In the first step, the algorithm focuses on the real part of the
% problem, i.e. Mrr and Deltar.  A disturbance is inserted
% on a single channel of the output of Mrr and a single error signal is
% pulled off the same channel. A variation of the wclowc/wcgain code
% is used to search for a ||Deltar||<= R which gives a large gain
% from the disturbance to error.  R is chosen adaptively to achieve a
% specified lower bound. The idea is that if this gain is large then
% it is likely that (I-Mrr*Deltar) will be close to singular.
%
% If the problem is a mixed-mu problem, then the real perturbation is
% wrapped back into the loop (if possible):
%            Mtil = Mcc+Mcr*Deltar*inv(eye(szreal)-Mrr*Deltar)*Mrc;
% The standard power iteration is then run on (Mtil, Deltac) to yield
% a perturbation: Delta = blkdiag(Deltar,Deltac).
%
% These two steps are attempted Ntry times starting from a random
% initial condition at each try.  Also, the location to insert d and
% pull of e is cycled through all possible channels.  Note that any
% Deltar returned in step 1 of the code will give a lower bound >= 1/R.
% lbtry:=1/R is chosen adaptively at each try in an attempt to find
% increasingly better lower bounds.
%
% 1/27/06 PJS

%   Copyright 2011-2012 The MathWorks, Inc.

% Set number of tries
if nargin < 6
    Ntry = 1;  % Number of tries
else
    % XXX Need to update if we want to pass in other options
    Ntry = opt.Ntry;
end

% Set options  
singtol_real = 1e-10;      % Singularity tolerance for real-mu problems
singtol_comp = 100*eps;   % Singularity tolerance for mixed-mu problems
stoptol = 0.97;           % Algorithm stops if lb/ub>stoptol

% Get size, indices for real/complex blocks, and real mask
[Mr,Mc]=size(M);

nreal = index.allreal.num;
realidx = index.allreal.realidx;
realr = index.allreal.allrows;
realc = index.allreal.allcols;
szreal = numel(realr);

ncomp = index.allcomp.num;
compidx = index.allcomp.compidx;
compr = index.allcomp.allrows;
compc = index.allcomp.allcols;

%real_mask = index.masks.real_mask';

% Partition M in 2x2 block conformal w/ real/complex blocks
Mrr = M(realr,realc);
Mrc = M(realr,compc);
Mcr = M(compr,realc);
Mcc = M(compr,compc);

% Search for ||Deltar||<=R which makes d->e have large gain and
% then use the power iteration on complex blocks. 
Delta = zeros(Mc,Mr);
deltareal = zeros(nreal,1);
%I = eye(szreal);
cnt = 1;
ub = bnds(1,1);
lb = bnds(1,2);
lbfac = 3/4;
while (cnt<=Ntry) && (lb<stoptol*ub) 
    
    % Set upper bound on Delta to achieve a specified lower bound
    lbtry = lb+lbfac*(ub-lb);
    R = 1/lbtry;
    
    % LOCALwclowc finds Deltar which makes d->e have large gain.
    channel = mod(cnt-1,szreal)+1;      % cycle through channels
    if cnt==1 && ~isempty(deltareal0)
        % Use a initial condition on first try if available        
        [~,Deltar,deltareal] = LOCALwclowc(Mrr,realidx,R,channel,RNG,deltareal0);
    else
        [~,Deltar,deltareal] = LOCALwclowc(Mrr,realidx,R,channel,RNG);
    end
    
    % Check result: 
    %tmp = 1+Mrr(channel,:)*Deltar*inv( eye(szreal)- Mrr*Deltar )*I(:,channel);
    %wcval2 = norm( tmp )
    %[wcval wcval2 abs(wcval2-wcval)]
    
    lb2 = 0;
    if ncomp==0
        
        % pure real mu problem
        if (min(abs(1-eig(Mrr*Deltar))) < singtol_real) || ...
              ((szreal==1) && abs(1 - Mrr*Deltar) < singtol_real)
            Delta2 = zeros(Mc,Mr);
            Delta2(realc,realr) = Deltar;
            lb2 = 1/norm(Delta2);
        end        
                        
    else
        
        % mixed mu problem
        if (min(abs(1-eig(Mrr*Deltar))) < singtol_comp) || ...
              ((szreal==1) && abs(1 - Mrr*Deltar) < singtol_comp)
            Delta2 = zeros(Mc,Mr);
            Delta2(realc,realr) = Deltar;
            lb2=1/norm(Delta2);   
        else
            % wrap in reals
            Mtil = Mcc+Mcr*Deltar*((eye(szreal)-Mrr*Deltar)\Mrc);
        
            % do power iteration on complex blocks
            [~,Deltac] = mmupiter(Mtil,compidx);
            
            % form pert from Deltar and Deltac
            Delta2 = zeros(Mc,Mr);
            Delta2(realc,realr) = Deltar;
            Delta2(compc,compr) = Deltac;
            lb2 = 1/norm(Delta2);            
        end        
    end
    
    if lb2>lb && lb2<=ub
        % Store Delta if it increased the lower bound
        lb = lb2; 
        Delta = Delta2;         
        %lbfac = min( 1/2 , 2*lbfac );
        lbfac = 1/2;
    else
        % Delta did not increase lb, reduce lbfac for next try
        lbfac = max( 1/64 , lbfac/2 );
    end
    
    cnt = cnt+1;    
end 


%-------------------------------------------------------------------

function [wcval,Delta,deltavals] = LOCALwclowc(M,index,R,channel,RNG,deltastart)
% Solves   max Fu(Delta,L)  subject to ||Delta||<=R
% where L:=[M I(:,channel); M(channel,:) 1] and there is only a
% single disturbance and error.  It is assumed that the block structure
% is purely real ( the code will fail otherwise ).
%
% This is a variant of wclowc with code modified to take
% advantage of the explicit structure of the problem
% created within muwcglb.  Specifically:
%   1) removed code related to complex blocks since muwcglb only
%           calls it with real blocks
%   2) removed code related to multim-dim arrays since muwcglb only
%           calls it with a single matrix (loop on data done in mussvcalc)
%   3) ne=1/nd=1 since muwcglb only adds a disturbance/error on
%           a single channel.  Thus the "22" block of m is a scalar.
%   4) LOCALwclowc has capability to start from a specified initial
%           condition.
%   5) LOCALwclowc combines the code associated with scalar and
%           repeated real perturbations for simplicity/speed.
if nargin<6
    deltastart=[];
end

% Set options 
MXGAIN = 1e16; % Code stops if d->e gain is larger than MXGAIN
MAXCNT = 500; % Number of times to cycle through all reals

% Get sizes and real mask
Mr = size(M,1);
nreal = index.allreal.num;
real_mask = index.masks.real_mask';

% Insert disturbance and pull off error on a single channel. Also,
% scale L so that code below assumes Delta is in the unit cube.
L = [R*M zeros(Mr,1); M(channel,:) 1]; 
L(channel,end) = R;
ne = 1;
nd = 1;

% Initialize worst-case gain at center of unit cube
wcval = 1;
%initcost = wcval;
deltavals_best = zeros(nreal,1);
Lreal = -ones(nreal,1); 
Ureal = ones(nreal,1);

if ~isempty(deltastart)
    % use specified initial condition on first try
    deltavals = deltastart/R;
    deltavals = max( min(1,deltavals) , -1 );
else
    % create random starting point 
    deltavals = Lreal+rand(RNG,nreal,1).*(Ureal-Lreal); 
end

% Move to starting point
mloop = zeros(Mr+1);
mloop(end,:) = L(end,:);
deltavec = real_mask*deltavals;
left = [eye(Mr); L(end,1:Mr).*(deltavec.')];
mid =  eye(Mr)-L(1:Mr,1:Mr).*repmat(deltavec.',Mr,1);
right = L(1:Mr,:);
mloop = mloop+ (left/mid)*right;
gain = abs( mloop(end,end) );

% Update bounds to reflect starting point
Lreal = -1-deltavals;
Ureal = 1-deltavals;

% Cycle through reals
%mdchange = 1;
cnt = 0;
%notinf = 1;
go = 1;
% Create random cycling order
[~,allRandIdx] = sort(rand(RNG,nreal,MAXCNT),1);
while (cnt<MAXCNT && gain<MXGAIN && go==1) || cnt==0
    cnt = cnt + 1;    
    mdchange = 0;            
    oidxreal = allRandIdx(:,cnt);
    gain0 = gain;
    ii = 1;
    while ii<=nreal && gain<MXGAIN
        i = oidxreal(ii);
        blkd = index.allreal.repeated(i);
        ridx = index.allreal.rows{i};
        cidx = index.allreal.cols{i};
        tmpm = mloop([ridx end],[cidx end]);
        [gain,adjustment] = wcslft(tmpm,Lreal(i),Ureal(i),blkd,ne,nd); 
        if adjustment~=0
            deltavals(i) = deltavals(i) + adjustment;
            Lreal(i) = -1 - deltavals(i);
            Ureal(i) = 1 - deltavals(i);
            mdchange = 1;
            if ~isinf(gain)
                mloopnew = mloop;
                mloopnew(ridx,:) = zeros(blkd,Mr+1);
                left = [mloop(1:ridx(1)-1,cidx)*adjustment; ...
                        eye(blkd); mloop(ridx(end)+1:Mr+1,cidx)*adjustment];
                mid = eye(blkd) - mloop(ridx,cidx)*adjustment;
                right = mloop(ridx,:);                    
                mloopnew = mloopnew + (left/mid)*right;
                mloop = mloopnew;                    
            end
        end
        ii = ii + 1;
    end        
    if gain>wcval % XXX shouldn't gain be non-decreasing?
        deltavals_best = deltavals;
        wcval = gain;
    end
    
    %[cnt deltavals_best.' gain] %XXX
    
    if (gain-gain0)<0.001*gain || mdchange ==0
        go = 0; 
    end
end

% Scale Delta
deltavals = deltavals_best*R;
Delta = diag(real_mask*deltavals);
