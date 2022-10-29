
%   Copyright 2009 The MathWorks, Inc.
function [bnd,sens,dcd,drid,g] = ...
grubresc(M,K,I,J,Kc,Kr,Ic,Ir,Jc,Jr,Fc,Fr,kc,kr,ic,ir,...
             jc,jr,fc,fr,sc,sr,csc,csr,csF,nK,nI,nJ,nF,nc,...
             nr,kcK,icI,jcJ,jrJ,fcF,dcb,drib,sens)
%function [sens,dcd,drid,g] = ...
%	  grubresc(M,K,I,J,Kc,Kr,Ic,Ir,Jc,Jr,Fc,Fr,kc,kr,
%	      ic,ir,jc,sr,fc,fr,sc,sr,csc,csr,csF,nK,nI,nJ,nF,nc,nr,kcK,
%	      icI,jcJ,jrJ,fcF,dcb,drib,sens)
%       Not intended to be called by the user.
%
% P. M. Young, December, 1996.
% Cleaned up GJB 22Aug08

if nargin < 39
	disp('   grubresc.m called incorrectly')
	return
end

%%%%%%%%%% 	Absorb the balancing

dMd = dcb*M*drib;	
dc = eye(csc(4));	
dri = eye(csr(4));
g = zeros(sum(K),1);	

[g,gamr,gamc,jg]     = LOCALgeerescv4(dMd,Kc,Kr,kcK,nK,csc);
[bnd,g,jg,gamc,gamr] = LOCALnewgrescv4(dMd,g,jg,gamc,gamr,Kc,Kr);

dcd  = dc*dcb;
drid = drib*dri;

gclip = 4;	
gmult = 1;
relgap = min([1 abs(1-bnd)]);
for ii = 1:nK
   % changed for PMY for DGKIT added LOGICAL
   tmpmin = min([max(abs(g(logical(kc(ii,logical(Kc) ) ) ))) gclip]);
   sensmult = 1 + gmult*relgap*sqrt(tmpmin);
   sens(ii) = sensmult*sens(ii);
end % for ii

%----------------------- Local Subroutines ------------------------%
function [g,gamr,gamc,jg] = LOCALgeerescv4(dMd,Kc,Kr,kcK,nK,csc)
%function [g,gamr,gamc,jg] = LOCALgeerescv4(dMd,Kc,Kr,kcK,nK,csc)
%	We start with a balanced matrix dMd.
%	This code can be made to bomb if called incorrectly.
%
% P. M. Young, December, 1996.

if nargin<6,	disp('   geeresc.m called incorrectly')
		return
end

sizM = size(dMd);

%	make g = imag diag

gamc = ones(sizM(1),1);
gamr = ones(sizM(2),1);

% mod for PMY for DGKIT: added LOGICAL statement
diM  = imag(diag(dMd(logical(Kc),logical(Kr))));
jg   = zeros(sizM(1),sizM(2));
jg(logical(Kc),logical(Kr)) = diag(j*diM);
dMdi = dMd - jg;
diM2 = diM.^2;
gamc(logical(Kc)) = (1+diM2).^(-0.25);
gamr(logical(Kr)) = gamc(logical(Kc));
g = diM;

function [bnd,g,jg,gamc,gamr] = LOCALnewgrescv4(dMd,g,jg,gamc,gamr,Kc,Kr)
%function [bnd,g,jg,gamc,gamr] = LOCALnewgrescv4(dMd,g,jg,gamc,gamr,Kc,Kr)
%	Gradient search on g
% P. M. Young, December, 1996.

if nargin<7
    disp('   newgrescv4 called incorrectly')
	return
end

Krlog = logical(Kr);
Kclog = logical(Kc);

sizM = size(dMd);

kk = 10;   
scale = min([0.5 0.5*max(abs(g))]);

%%%%%   MAIN LOOP HERE
while kk > 0 
    [u,s,v] = svd((dMd-jg).*(gamc*gamr'));
    sig	= max([max(diag(s)) 10*eps]);
    ind	= find((diag(s)/sig)>0.95);
    if length(ind)>5 || isempty(ind)  %Error cases, GJB 22Aug08??	
        break
    end
    % for PMY for DGKIT added LOGICAL
    u   = u(Kclog,ind);
    s   = s(ind,ind);
    v   = v(Krlog,ind);
    jgb = zeros(sizM(1),sizM(2));
    gbc = gamc;
    gbr = gamr;
    ginv = 1../(1+g.*g);
    galf = sqrt(ginv)*ones(1,length(ind));
    ginv = (g.*ginv)*ones(1,length(ind))/2;
    %	calculate partial derivatives, and step
    grad = -real(((abs(u).^2 + abs(v).^2).*ginv)*s + j*conj(u).*v.*galf);
	if norm(grad,'fro')<10*eps,
        break
    end
    grad2 = grad'*grad;
    % GJB 13Aug08
    % if abs(det(grad2)) > 1e-20 % Shouldn't this really be EPS? 
    if abs(det(grad2)) > eps
        %sajjad	 
        %gdir = -grad*(grad2\(diag(s)-0.90*sig)); % because Mathworks %lies??
        gdir = -grad*(inv(grad2)*(diag(s)-0.90*sig));	% because Mathworks lies
    else
        break
    end
    % Need a step length in scale
    delg = gdir;
    sizdelg = max(max(abs(delg)));
    if sizdelg > 10*eps   
        delg = delg/sizdelg;   
    else
    break
    end
    delg = delg*scale;

    if max(max(abs(delg)))>1e6
        delg = 1e6*delg/max(max(abs(delg)));		
    end
    %	take a step to decide on the final step size
    gb = g + delg;
    % for PMY for DGKIT added LOGICAL
    jgb(Kclog,Krlog) = diag(j*gb);
    gbc(Kclog) = (1+gb.*gb).^(-1/4);
    gbr(Krlog)    = gbc(Kclog);

    %sajjad  %sig1    = norm((dMd-jgb).*(gbc*gbr')); % MATLAB
    %sajjad: I added the following lines. if tmp_sig1 is "isnan and/or isinf", 
    % it returns the previous variables, i.e. no changes in new step!
    tmp_sig1=(dMd-jgb).*(gbc*gbr');
    %if any(isnan(vec(tmp_sig1)))==1 || any(isinf(vec(tmp_sig1)))==1
    if any(isnan(tmp_sig1(:)))==1 || any(isinf(tmp_sig1(:)))==1
        % Need to remove this and do the right thing, GJB 22Aug08
        fprintf('*')
        keyboard
        return
    end
    sig1    = norm(tmp_sig1);
    %	do the parabola
    delsig	= grad'*delg;
    crap	= max([abs(sig1-sig-delsig(1)) 10*eps]);
    steprat = min([100 max([-delsig(1)/2/crap -100])]);
    ii = 0;
    while steprat<0.8 && steprat>0
        delg = delg*max([0.125 steprat]);
        gb   = g + delg;             
        % for PMY for DGKIT added LOGICAL
        jgb(Kclog,Krlog) = diag(j*gb);
        gbc(Kclog) = (1+gb.*gb).^(-1/4);
        gbr(Krlog) = gbc(Kclog);
        sig1	= norm((dMd-jgb).*(gbc*gbr'));
        delsig	= grad'*delg;
        crap	= max([abs(sig1-sig-delsig(1)) 10*eps]);
        steprat	= -delsig(1)/2/crap;
        ii = ii + 1;	
        if ii==10
            steprat = 0;
        end
    end
	ii = 0;
    while steprat>1.5 || steprat < 0.0
        delg = delg*2;
        gb   = g + delg;
        % for PMY for DGKIT added LOGICAL
        jgb(Kclog,Krlog) = diag(j*gb);
        gbc(Kclog) = (1+gb.*gb).^(-1/4);
        gbr(Krlog) = gbc(Kclog);
        sig1	= norm((dMd-jgb).*(gbc*gbr')); 
        delsig	= grad'*delg; 
        crap	= max([abs(sig1-sig-delsig(1)) 10*eps]);
        steprat	= -delsig(1)/2/crap;
        if steprat<0.5 && steprat>0,
            delg = delg/2;
        end
	ii = ii + 1;	
        if ii==20
            steprat = 0;
        end
    end
    if steprat == 0;
        scale = max(max(abs(delg)));
    else
        scale = 0.5*abs(steprat)*max(max(abs(delg)));
    end
    delg = delg*steprat;
    g    = g + delg;
    g    = min(1e40,max(-1e40,g));
    % for PMY for DGKIT added LOGICAL
    jg(Kclog,Krlog) = diag(j*g);
    gamc(Kclog)  = (1+g.*g).^(-1/4);
    gamr(Krlog)  = gamc(Kclog);
    if scale < 5e-6
        break
    end
    kk = kk-1;
end % while kk
%%%%%%  END OF MAIN LOOP

bnd = norm((dMd-jg).*(gamc*gamr'));
