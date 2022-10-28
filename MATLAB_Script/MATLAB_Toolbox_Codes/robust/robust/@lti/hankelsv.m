function [HSV_STAB,HSV_UNSTAB] = hankelsv(G,option,style)

% HANKELSV	Compute Hankel Singular Values of a continuous/discrete & stable/unstable system.
%
% [HSV_STAB,HSV_UNSTAB] = HANKELSV(G,OPTION,STYLE) returns a column vector HSV_STAB containing 
% the Hankel singular values of the stable part of LTI object G, and additional column
% of anti-stable part(if present).
%
% Option specifies the error type for computation:
%
%	OPTION = 'add', Hankel SV's of G (default)
%	OPTION = 'mult',  Stochastic SV's of the phase matrix of a stable G (ref.: bstmr)
%	OPTION = 'ncf',  Hankel SV's of normalized coprime factor (ref.: ncfmr)
%
%   For option = 'add' or 'mult', Hankel SV's of jw-axis or unit disk poles are set
%   to inf.
%
% HANKELSV(G) with no output arguments draws a bar graph of the Hankel 
% singular values, where
%
%       STYLE = 'abs', draw the absolute value
%       STYLE = 'log', draw the Hsv in log scale
%
%  See also STABPROJ.

% R. Y. Chiang & M. G. Safonov 4/15/02
% Copyright 1988-2004 The MathWorks, Inc. 
% All Rights Reserved. 

nout = nargout;
nin = nargin;

try 
    G = ss(G);
catch
    error('First input argument should be a SS object !');
end

xsize = size(G);
if length(xsize)>2
    error('Not supported for arrays of LTI models !');
end

if nin == 1 
    option = 'add';
    style = 'abs';
end
if nin == 2
    style = 'abs';
end

option = ltipack.matchKey(option,{'add','mult','ncf'});
if isempty(option)
   error('2nd input argument only accepts ''add'', ''mult'', or ''ncf''.');
end
style = ltipack.matchKey(style,{'log','abs'});
if isempty(style)
   error('3rd input argument only accepts ''abs'' or ''log''.');
end
    
Ts = abs(get(G,'Ts'));

% Handle discrete case:

paaz = 2^-4*sqrt(eps);

if Ts
    a = get(G,'a');
    if (max(abs(eig(a)+1))>paaz)  % if no poles near z=-1
        paaz = 0;
    end
    G = bilin(G,-1,'S_Tust',[Ts, 1-paaz]);
end

hanksvstab = [];
hanksvunstab = [];
Gorig = G;

% Split spectrum into stable/unstable/jw-axis
jwtol = sqrt(eps) * norm(balance(G.A),1);
[H,H0] = modsep(G,3,@(p) localAssignRegion(p,jwtol));
nx = order(H);
m = nx(1);  no_unstable = nx(2);  no_jw_pole = nx(3); % no. stable, unstable, jw-axis poles
mjws = m+no_jw_pole;   ra = mjws+no_unstable;
G = H0 + subparen(H,{':',':',1}) + subparen(H,{':',':',2});
[a,b,c,d] = ssdata(G);

switch option
    case 'add'
        if m > 0 && no_unstable == 0      % completely stable
           p = gram(a,b);
           q = gram(a',c');
           [up,sp,vp] = svd(p);
           lr = up*diag(sqrt(diag(sp)));
           [uq,sq,vq] = svd(q);
           lo = uq*diag(sqrt(diag(sq)));
           hanksvstab = svd(lo'*lr);
           hanksvunstab = [];
        elseif m == 0 % completely unstable
           p = gram(-a,-b);
           q = gram(-a',c');
           [up,sp,vp] = svd(p);
           lr = up*diag(sqrt(diag(sp)));
           [uq,sq,vq] = svd(q);
           lo = uq*diag(sqrt(diag(sq)));
           hanksvunstab = svd(lo'*lr);
           hanksvstab = [];
        elseif m > 0 & no_unstable > 0
           [Gl,Gr,msta] = stabproj(G);
           [al,bl,cl,dl] = ssdata(Gl);
           [ar,br,cr,dr] = ssdata(Gr);
            % stable
           pl = gram(al,bl);
           ql = gram(al',cl');
           [up,sp,vp] = svd(pl);
           lr = up*diag(sqrt(diag(sp)));
           [uq,sq,vq] = svd(ql);
           lo = uq*diag(sqrt(diag(sq)));
           hanksvstab = svd(lo'*lr);
           % unstable
           pr = gram(-ar,-br);
           qr1 = gram(-ar',cr');
           [up,sp,vp] = svd(pr);
           lr = up*diag(sqrt(diag(sp)));
           [uq,sq,vq] = svd(qr1);
           lo = uq*diag(sqrt(diag(sq)));
           hanksvunstab = svd(lo'*lr);
        end
     
    case 'mult'
        [md,nd]= size(d);
        if md > nd
           %disp('Note: The palnt has been transposed for robust stochastic SV computation.');
           btemp = b;
           b = c';
           c = btemp';
           d = d';
           a = a';
           G = ss(a,b,c,d);
        end
        if rank(d*d') < min([md, nd])
             %disp('   WARNING: D MATRIX IS NOT FULL RANK - - -');
             %disp('            THE PROBLEM IS ILL-CONDITIONED !');
             %d = 1e-8*eye(size(d));
             d = max(abs(eig(a)))/1000*eye(size(d));
        end
        if m > 0 && no_unstable == 0       % completely stable
           P = gram(a,b);
           G = P*c'+b*d';
           phi = d*d';
           %
           qrnQ = [zeros(ra-no_jw_pole) -c';-c -phi];
           [K,Q,Qerr] = lqrc(a,G,qrnQ);
           %
           [up,sp,vp] = svd(P);
           lr = up*diag(sqrt(diag(sp)));
           [uq,sq,vq] = svd(Q);
           lo = uq*diag(sqrt(diag(sq)));
           hanksvstab = svd(lo'*lr);
           hanksvunstab = [];
        elseif m == 0 % completely unstable
           a = -a; b = -b;
           P = gram(a,b);
           G = P*c'+b*d';
           phi = d*d';
           %
           qrnQ = [zeros(ra-no_jw_pole) -c';-c -phi];
           [K,Q,Qerr] = lqrc(a,G,qrnQ);
           %
           [up,sp,vp] = svd(P);
           lr = up*diag(sqrt(diag(sp)));
           [uq,sq,vq] = svd(Q);
           lo = uq*diag(sqrt(diag(sq)));
           hanksvunstab = svd(lo'*lr);
           hanksvstab = [];
        elseif m > 0 & no_unstable > 0
           [Gl,Gr,msta] = stabproj(G);
           [al,bl,cl,dl]= ssdata(Gl);
           [ar,br,cr,dr]= ssdata(Gr);
           dl = 0.5*d;, dr = 0.5*d;
           % stable
           P = gram(al,bl);
           G = P*cl'+bl*dl';
           phi = dl*dl';
           %
           qrnQ = [zeros(msta) -cl';-cl -phi];
           [K,Q,Qerr] = lqrc(al,G,qrnQ);
           %
           [up,sp,vp] = svd(P);
           lr = up*diag(sqrt(diag(sp)));
           [uq,sq,vq] = svd(Q);
           lo = uq*diag(sqrt(diag(sq)));
           hanksvstab = svd(lo'*lr);
           % unstable
           ar = -ar; br = -br;
           P = gram(ar,br);
           G = P*cr'+br*dr';
           phi = dr*dr';
           %
           qrnQ = [zeros(ra-msta-no_jw_pole) -cr';-cr -phi];
           [K,Q,Qerr] = lqrc(ar,G,qrnQ);
           %
           [up,sp,vp] = svd(P);
           lr = up*diag(sqrt(diag(sp)));
           [uq,sq,vq] = svd(Q);
           lo = uq*diag(sqrt(diag(sq)));
           hanksvunstab = svd(lo'*lr);
         end
     case 'ncf'
       
       [A,B,C,D] = ssdata(Gorig);
       n = length(A);
       [p,mm]= size(d); 
       
       %  Solve the GFARE soln X=S'*S

       Ham = [A' zeros(n); -B*B' -A] - [C'; -B*D']*inv(eye(p)+D*D')*[D*B' C];
      [U,Hamm]=schur(Ham);
      if any(imag([A B;C D]))
	     [U,Hamm]=ocsf(U,Hamm);
      else 
    	 [U,Hamm]=orsf(U,Hamm,'s'); 
      end
     [S,SS]=schur((U(1:n,1:n)'*U(n+1:2*n,1:n)+U(n+1:2*n,1:n)'*U(1:n,1:n))/2);
      S=S*sqrt(abs(SS))*S';
     [qdr,Dr]=qr([D;eye(mm)]);Dr=inv(Dr(1:mm,1:mm));
     [qdl,Dl]=qr([D';eye(p)]);Dl=inv(Dl(1:p,1:p)');

     % change coordinates using U(1:n,1:n)

      nmuinv= norm(inv(U(1:n,1:n)));
     if nmuinv>1e12
       disp('Warning from HANKELSV: SYS is close to being undetectable.');
       disp('                       Results maybe unreliable.')
       % return
      end %if nmuinv>1e12,
      B=U(1:n,1:n)'*B; %temp variable
      Cl=Dl*(C/U(1:n,1:n)');
      C=C*U(n+1:2*n,1:n); %temp variable
      Al=Hamm(1:n,1:n)';
      Bl=(B-C'*D)*Dr*Dr';        
      Hl=-(C'+B*D') *Dl'*Dl;       

     % realization of nlcf [N M] is now pck(Al,[Bl Hl],Cl,[Dl*D;Dl])
     % Calculate observability Gramian for nrcf [Nl Ml], R*R'

      nm11=n:-1:1;
      R=sjh6(Al(nm11,nm11),Cl(:,nm11));
      R=R(nm11,nm11)';

    % calculate the Hankel-singular values of [Nl Ml]

      [W,T,V] = svd(S*R);
      hanksvstab = diag(T);
      hanksvunstab = [];
end

if m == 0 & ra == 0
    hanksvstab = zeros(0,0);
    hanksvunstab = zeros(0,0);
end

% Plotting mode

if nout == 0

   figure
   
   if (m == 0 && m == ra) || strcmp(option, 'ncf')  % complete stable case
      
      if strcmp(style,'abs')
         bar(hanksvstab,'r')
      else
         semilogy(hanksvstab,':*');
         set(get(gca,'children'),'color','r')
      end
      xlabel('Order')
      ylabel(style);
      if no_jw_pole ~= 0
         title(['Hankel Singular Values (' num2str(no_jw_pole) ' of them = inf, not shown)']);
      else
          title('Hankel Singular Values');
      end
      if strcmp(option, 'ncf')
          title('Hankel SV of Coprime Factors [Nl Ml]')
      elseif strcmp(option, 'mult')
          if no_jw_pole ~= 0
             title(['Hankel Singular Values of Phase Matrix (' num2str(no_jw_pole) ' of them = inf, not shown)']);
          else
              title('Hankel Singular Values of Phase Matrix')
          end
      end
   
   elseif m > 0 || no_unstable > 0 || no_jw_pole > 0
      xaxis = 1:no_unstable+mjws;
 
      if strcmp(style,'abs')
         bar(xaxis, [zeros(no_unstable,1);inf*ones(no_jw_pole,1);hanksvstab],'r');
      else
         semilogy(xaxis, [zeros(no_unstable,1);inf*ones(no_jw_pole,1);hanksvstab],':*');
         axisid = get(gca,'children');
         set(axisid,'color','r');
      end
      
      hold on
      
      if strcmp(style,'abs')
         bar(xaxis,[hanksvunstab;inf*ones(no_jw_pole,1);zeros(m,1)],'b')
      else
         semilogy(xaxis,[hanksvunstab;inf*ones(no_jw_pole,1);zeros(m,1)],':o')
         axisid2 = get(gca,'children');
         set(axisid2(1),'color','b');
      end
      
      xlabel('Order')  
      ylabel(style);
      
      if strcmp(option, 'add')
          if no_jw_pole ~= 0
              if Ts
                 title(['Hankel Singular Values (' num2str(no_jw_pole) ' of unit disk pole(s) = inf, not shown)']);
              else
                 title(['Hankel Singular Values (' num2str(no_jw_pole) ' of jw-axis pole(s) = inf, not shown)']);
              end
          else
             title('Hankel Singular Values');
          end      
      elseif strcmp(option, 'mult')
          if no_jw_pole ~= 0
             if Ts
                 title(['Hankel Singular Values of Phase Matrix (' num2str(no_jw_pole) ' of unit disk pole(s) = inf, not shown)']);
             else
                 title(['Hankel Singular Values of Phase Matrix (' num2str(no_jw_pole) ' of jw-axis pole(s) = inf, not shown)']);
             end
          else
             title('Hankel SV of Phase Matrix');
          end 
      end
      if m > 0 && no_unstable > 0
          legend('HSV(Stable Part of G)','HSV(Unstable Part of G)')
      end
      
      hold off

   end
else
    HSV_STAB = [inf*ones(no_jw_pole,1);hanksvstab];
    HSV_UNSTAB = hanksvunstab;
end


function r = localAssignRegion(p,jwtol)
if real(p)<-jwtol
   r = 1;
elseif real(p)>jwtol
   r = 2;
else
   r = 3;
end