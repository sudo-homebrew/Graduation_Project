function gfac = gfactor(sys,sty);

% gf = gfactor(sys); computes the DC-gain matrix for MIMO system and put
% the result in a multi-dimenstion matrix. 
%
% Each page of "gfac" is a DC gain matrix associated with the ith mode.
%
% See also DCGAINMR, REDUCE.

% R. Y. Chiang & M. G. Safonov 7/2006
% Copyright 1988-2006 The MathWorks, Inc. 
% All Rights Reserved.

[a,b,c,d] = ssdata(sys);
if  isequal(size(a,1),0),
    gfac=zeros(1,0);
    return
end
if nargin == 1
    sty = 'nomod';
end

if ~issame(sty,'modal')
    cut = length(find(abs(eig(sys))< 1e-5));
    [g1,g2]= modreal(sys,cut); % numerically works better to split ridig body dynamics
    [ag1,bg1,cg1,dg1] = ssdata(g1);
    [ag2,bg2,cg2,dg2] = ssdata(g2);
    g2 = ss(ag2,bg2,cg2,zeros(size(d)));
    g1 = ss(ag1,bg1,cg1,d);
    sysm = g1+g2;
else
    sysm = sys;
end

[A,B,C,D] = ssdata(sysm);

no_state = length(A);
[m,n] = size(D);

lambda = eig(A);
[lammag,ind] = sort(abs(lambda));
lambda = lambda(ind);

accum = 0;

for ii = 1:no_state
    if lammag(ii) < 1e-10
        modeid(ii,:) = 'rigi';
        accum = accum + 1;
    else 
        if abs(imag(lambda(ii))) < 1e-10 
            modeid(ii,:) = 'real';
            accum = accum+1;
        else
            if imag(lambda(ii)) > 0 & imag(lambda(ii+1)) < 0 & lammag(ii) == lammag(ii+1)
                modeid(ii,:) = 'cmpx';
                modeid(ii+1,:) = 'cxcj';
                accum = accum + 2;
            end
        end
    end
    if accum == no_state
        break
    end
end
      
accum = 0;

for col = 1:n
    for row = 1:m
        for ii = 1:no_state
                if issame(modeid(ii,:),'rigi') | issame(modeid(ii,:),'real')
                    a = A(ii,ii);
                    b = B(ii,col);
                    c = C(row,ii);
                    if modeid(ii,:) == 'rigi'
                        gfac(row,col,ii) = inf;
                    else
                        gfac(row,col,ii) = norm(c,2)*norm(b,2)/lammag(ii);
                    end
                    accum = accum + 1;
                end
                if issame(modeid(ii,:),'cmpx') 
                        a = A(ii:ii+1,ii:ii+1);
                        b = B(ii:ii+1,col);
                        c = C(row,ii:ii+1);    
                        omega2 = abs(lambda(ii)); % formerly: omega2 = lambda(ii)*lambda(ii+1);
                        gfac(row,col,ii)   = norm(c,2)*norm(b,2)/omega2/2; % formerly: gfac(row,col,ii)   = norm(c,2)*norm(b,2)/omega2;
                        gfac(row,col,ii+1) = gfac(row,col,ii);  % complex conjugate
                        accum = accum + 2;
                end
                if accum == no_state
                    break
                end
        end
        accum = 0;
    end
    accum = 0;
end

% ----- End of gfactor.m