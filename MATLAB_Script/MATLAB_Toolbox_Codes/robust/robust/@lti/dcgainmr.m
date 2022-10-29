function [sysr,syse,gain] = dcgainmr(sys,ord)

% [sysr,syse,gain] = dcgainmr(sys,ord) returns an order ORD reduced order model  
% a continuous-time LTI system SYS by truncating modes with least DC gain.
% 
% The DC gain of a complex mode (1/(s+p))*c*b' is norm(b)*norm(c)/abs(p).
%
% Inputs: sys = LTI object, continuous-time system
%         ord = size of the reduced order model(s) (can be a vector array of
%               several reduced order model sizes, e.g., ord = [10:15])
% 
% Outputs: sysr = reduced order models (multi-dimension array, if more than
%                 one reduced model is requested)
%          syse = sys - sysr 
%          gain = g-factors (dc-gains)
%
% 
% See also REDUCE.

% R. Y. Chiang & M. G. Safonov 7/2006
% Copyright 1988-2006 The MathWorks, Inc. 
% All Rights Reserved.

[a,b,c,d,Ts] = ssdata(sys);

if ~isequal(Ts,0)
   error('DCGAINMR only supports continuous systems.')
end

if nargin<3,
    tol=1e-5;
end
cut = length(find(abs(eig(sys))< tol));
[g1,g2]= modreal(sys,cut); % numerically works better to split rigid body dynamics
[ag2,bg2,cg2,dg2]= ssdata(g2);
[ag1,bg1,cg1,dg1] = ssdata(g1);
g2 = ss(ag2,bg2,cg2,zeros(size(d)));
g1 = ss(ag1,bg1,cg1,d);

[m,n] = size(d);
no_state = length(a);

gfac = gfactor(g2,'modal');
gain=zeros(1,no_state-cut); % initialize
for i = 1:no_state-cut
    temp = svd(gfac(:,:,i));
    gain(:,i)=temp(1);
end

[sdcgain,ind] = sort(gain);

ind = ind(end:-1:1);
gain=[Inf*ones(1,cut),sdcgain(end:-1:1)]; % added MGS 


a = ag2(ind,ind);
b = bg2(ind,:);
c = cg2(:,ind);
d = dg2;
n=size(a,1);

if nargin < 2,
    if n>0,
        bar(gain(1,:))
        xlabel(['modes 1 --> ' num2str(length(a))]);
    end
    ord = input('Please enter the desired order: ');
end
if ~isa(ord,'double') |ord<0|~isequal(ord,fix(ord)),
    error('Desired order ORD must be a non-negative integer')
end
ord=min(ord,no_state);


for i = 1:length(ord)
    ordd = ord(i) - cut;
    if (ord(i)<n),
        if (abs(a(ordd,ordd+1)) > eps), 
            ordd=ordd-1; % don't cut the a matrix in the middle of a jordan block
        end
    end
    sysr2 = ss(a(1:ordd,1:ordd),b(1:ordd,:),c(:,1:ordd),d);
    syse = ss(a(ordd+1:end,ordd+1:end),b(ordd+1:end,:),c(:,ordd+1:end),0*d);
    sysrr = g1+sysr2;
    sysr(:,:,i) = sysrr;
end

if nargout == 0
    for i = 1:length(ord)
        figure(i)
        sigma(sys,'r',sysr(:,:,i),'b')
        legend('Original Model','Reduced Order Model');
    end
end

%
% ---- end of dcgainmr.m % RYC
