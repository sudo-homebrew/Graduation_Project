function [K,clpic,maxbnd,dkinfo] = dksyn(P,NMEAS,NCONT,varargin)
%   DKSYN is obsolete, please use MUSYN instead.
%
%   See also MUSYN.

% Old help
%
%DKSYN  Robust controller design via mu-synthesis.
%
%   [K,CL,CLPERF] = DKSYN(P,NMEAS,NCONT) synthesizes a controller K for  
%   the open-loop plant model P via the D-K or D-G-K iteration approach to 
%   mu-synthesis.  P is an uncertain state-space model (see USS) and it is 
%   assumed that the last NMEAS outputs and NCONT inputs of P are the 
%   measurement and control channels, respectively. DKSYN returns the 
%   controller K, the closed-loop model CL, and the robust closed-loop 
%   performance CLPERF (see DKSYNPERF). These variables are related by:
%
%      CL = lft(P,K);
%      BND = dksynperf(CL);
%      CLPERF = BND.UpperBound;
%
%   The last relation may hold only approximately due to the frequency 
%   gridding used in DKSYN.
%
%   [K,CL,CLPERF] = DKSYN(P,NMEAS,NCONT,OPT) specifies options for the D-K 
%   or D-G-K algorithm. Use DKSYNOPTIONS to create OPT and see DKSYNOPTIONS
%   for available options.
%
%   [K,CL,CLPERF,DKINFO] = DKSYN(P,NMEAS,NCONT,...) returns a log DKINFO
%   of the algorithm execution. DKINFO is a N-by-1 cell array where N is
%   the total number of iterations performed. The i-th cell contains a
%   structure with the following fields:
%             K: Controller at i-th iteration, SS object.
%          Bnds: Robust performance bound for the closed-loop system.
%            DL: Left D-scale, SS object
%            DR: Right D-scale, SS object
%            GM: Offset G-scale, SS object
%            GR: Right G-scale, SS object
%           GFC: Center G-scale, SS object
%     MussvBnds: Upper and lower mu bounds, an FRD object.
%     MussvInfo: Structure returned from MUSSV at each iteration.
%
%   If DKINFO is the result of a first DKSYN run, you can restart the
%   algorithm from the J-th iteration recorded in DKINFO by setting
%   OPT.StartingIterationNumber=J+1 and using the syntax
%      DKSYN(P,NMEAS,NCONT,DKINFO,OPT)
%   DKSYN then uses DKINFO to determine correct initial D,G-scalings.
%
%   See also DKSYNOPTIONS, DKSYNPERF, H2SYN, HINFSYN, DISKMARGIN, MUSSV, 
%   MUSSVEXTRACT, ROBGAIN, WCGAIN, WCSENS, WCDISKMARGIN.

%   Copyright 2003-2019 The MathWorks, Inc.

warning(message('Robust:design:dksyn1'))  % obsolete!

K = [];
clpic = [];
maxbnd = [];
dkinfo = [];

if nargin==3
   prevdkinfo = [];
   nopt = [];
elseif nargin==4 %4th argument is either PREVDKINFO or OPT
   fnames = fieldnames(varargin{1});
   if length(fnames)==1 && ...
         all(strcmp(fnames{1:4},...
         {'K','Bnd','DL','DR'}))
      prevdkinfo = varargin{1};
      nopt = [];
      muinfo = prevdkinfo.MussvInfo;
   elseif isa(varargin{1},'rctoptions.dksyn')
      nopt = varargin{1};
      prevdkinfo = [];
   else %error in 4th input
      error('Invalid Structure for OPT or PREVDKINFO')
   end
elseif nargin==5
   if ~isempty(varargin{1}) && isa(varargin{1},'rctoptions.dksyn')
      nopt = varargin{1};
      prevdkinfo = varargin{2};
   elseif ~isempty(varargin{2}) && isa(varargin{2},'rctoptions.dksyn')
      nopt = varargin{2};
      prevdkinfo = varargin{1};
   else
      error('The OPT input argument must a rctoptions.dksyn object.')
   end
   if ~isempty(prevdkinfo)
      fnames = fieldnames(prevdkinfo{1});
      infofnames = {'K';'Bnd';'DL';'DR'};
      if ~all(strcmp(fnames(1:4),infofnames))
         error('Invalid PREVDKINFO structure')
      end
   end
elseif nargin<3
   error('Must have at least three input arguments')
else
   error('Invalid number of input arguments')
end
if isempty(nopt)
   nopt = dksynOptions;
elseif ~isa(nopt,'rctoptions.dksyn')
   error('Fourth argument must be empty or a rctoptions.dksyn object.');
end
   
% Convert rctoptions.dksyn object to a STRUCT
opt = nopt;
MixedMU = strcmp(opt.MixedMU,'on');

%check dimensions
szp = size(P);
if length(szp)>2
   error('DKSYN does not handle extra array dimensions')
end
if (szp(1)-NMEAS)<1
   error('Insufficient number of P outputs and measurements')
end
if (szp(2)-NCONT)<1
   error('Insufficient number of P inputs and controls')
end
if strcmpi(opt.AutoIter,'On')
   auto = 1;
else
   auto = 0;
end
if strcmpi(opt.DisplayWhileAutoIter,'On') || ...
      strcmpi(opt.AutoIter,'Off')
   auto_visual = 1;
else
   auto_visual = 0;
end
Pnom = [];

if isempty(opt.FrequencyVector)
   if isa(P,'uss')
      %       unclist = pvget(P,'Uncertainty');
      %       unames = pvget(unclist,'UNames');
      %       for i=1:length(unclist)
      %          nam = pvget(unclist,unames{i});
      %          uclass = class(nam);
      %          if isequal(uclass,'udyn')
      %             error(['DKSYN is not implemented for ''' uclass ''' atom types']);
      %          end
      %       end
      %       Pnom = pvget(P,'NominalValue');
      unclist = P.Uncertainty;
      unames = fieldnames(unclist);
      for i = 1:length(unclist)
         name = unclist.(unames{1});
         uclass = class(name);
         if ~( isequal(uclass,'ureal') ||  isequal(uclass,'ucomplex') ||  isequal(uclass,'ultidyn') )
            error(['DKSYN is not implemented for ''' uclass ''' atom types']);
         end
      end
      Pnom = P.NominalValue;
   elseif isa(P,'ss') || isa(P,'tf') || isa(P,'szp')
      Pnom = P;
   end
   if ~isempty(Pnom)
      [~,OMEGA] = sigma(Pnom);
   else
      OMEGA = [];
   end
else
   OMEGA = opt.FrequencyVector;
end
if P.Ts==0
   DISCRETE = 0;
else
   DISCRETE = 1;
end

% HINFSYN parameters which will disappear, should all these to be passed in
GMAX = Inf;
GMIN = 0;
GTOL = 1;
GMAX_PLAN = 2;

axisvar = 'liv,m';
if abs(max(diff(OMEGA)) - min(diff(OMEGA))) < 1e-5
   axisvar = 'iv,m';
end

if isa(P,'ss') || isa(P,'tf') || isa(P,'szp') % no uncertainty
   iter_num = opt.StartingIterationNumber+1;
   NOMINAL = P;
   [K,clpic,clpic_g,gf,OMEGA,errormsg] = ...
      LOCALhinfsyn(NOMINAL,NMEAS,NCONT,opt,iter_num,NOMINAL,...
      OMEGA,GMIN,GMAX,GTOL,axisvar);
   if ~isempty(errormsg)
      dkinfo{iter_num}.errormsg = errormsg;
      maxbnd = [];
   else
      maxbnd = norm(clpic,inf);
   end
   return
elseif isa(P,'umat') || isa(P,'ufrd')
   %call cmmusyn
   error('DKSYN is not set up to handle UMATs or UFRDs')
elseif isa(P,'uss')
   %remove zero copies from P and get BLK info
   % newsys = rmzerocp(P);
   newsys = P;

   % The next line will fold in all parametric blocks
   %newsys.Data_ = foldBlocks(newsys.Data_, logicalfun( @isParametric , newsys.Data_.Blocks ));
   
   [~,~,~,muB] = ulftdata(newsys.Data_);
   BLK = muB.muBlkNby2;   
   %BLK = getmutoolsblkstruct(newsys); % old mutools blk structure
   BLK = [BLK; szp(2)-NCONT szp(1)-NMEAS]; % Add performance channel
   NOMINAL = lftdata(newsys); % M is an SS with Delta on top
   sznom = size(NOMINAL);
else
   error('DKSYN requires a USS object')
end

if isempty(opt.InitialController)
   K = [];
else
   K = opt.InitialController;
   szk = size(K);
   if szk(1)~=NCONT || szk(2)~=NMEAS
      error('Invalid dimension of opt.InitialController')
   else
      clp = lft(NOMINAL,K);
      evls = pole(clp);
      if (P.Ts==0 && any(real(evls)>0)) || (P.Ts~=0 && any(abs(evls)>1))
         warning('Initial controller does not stabilize the closed loop and will be ignored.')
      end
   end
end

iter_num  = opt.StartingIterationNumber;
iter_stop = opt.StartingIterationNumber + opt.NumberOfAutoIterations - 1;
autoord   = opt.AutoScalingOrder;
stophist  = zeros(0,3);

if iter_num == 1
   dl = eye(sznom(1),sznom(1));
   dr = eye(sznom(2),sznom(2));
   if any(BLK(:,1)<0)
      gm = zeros(sznom(1),sznom(2));
      gr = eye(sznom(2));
      gfc = zeros(sznom(1),sznom(2));
   else
      gm = [];
      gr = [];
      gfc = [];
   end
   dispdata = [];
else
   if ~isempty(prevdkinfo)
      %Dscale = prevdkinfo{iter_num-1}.MussvInfo.dvec;
      dl     = prevdkinfo{iter_num-1}.DL;
      dr     = prevdkinfo{iter_num-1}.DR;
      if MixedMU   % Real/Complex mu synthesis
         gm     = prevdkinfo{iter_num-1}.GM;
         gr     = prevdkinfo{iter_num-1}.GR;
         gfc    = prevdkinfo{iter_num-1}.GFC;
      else
         gm = [];
         gr = [];
         gfc = [];
      end
      sens   = prevdkinfo{iter_num-1}.MussvInfo.sens;
      bnds   = prevdkinfo{iter_num-1}.MussvBnds;
      muinfo = prevdkinfo{iter_num-1}.MussvInfo;
      K      = prevdkinfo{iter_num-1}.K;
      dkinfo = prevdkinfo(1:iter_num-1);
      dispdata = [];
   else
      error('Starting iteration is invalid -- PREVDKINFO does not exist')
   end
   if iter_num == 2     % haven't fit D's yet
      dl = [];
      dr = [];
      %use Dscale Freq, so it matched MUSSV Freq
      clpic = lft(NOMINAL,K);
      clpic_g = frd(clpic,bnds.Frequency);
   else    %(>2)
      if ~MixedMU
         IC = dl*NOMINAL/dr;    % absorb D's
      end
      clpic = lft(NOMINAL,K);
      clpic_g = frd(clpic,bnds.Frequency);
   end
end
again = 1;


if iter_num > 1
   if (length(OMEGA)~=length(bnds.Frequency) || any(OMEGA(:)~=bnds.Frequency)) ...
         && (auto == 0 || auto_visual == 1)
      disp(' ')
      disp('**** New OMEGA frequency data will be incorporated ****')
      disp('****       after next H-infinity control design       ****')
   end
end
%if ~isempty(get(0,'children'))
%   [mw,sw,notours] = findmuw;
%   if ~isempty(notours) && auto_visual==1
%      figure(notours(1))
%      clf
%   end
%end
while again == 1 && iter_num<=iter_stop
   if MixedMU    % Real/Complex mu synthesis
      dkinfo{iter_num} = struct('K',[],'Bnd',[],'DL',[],'DR',[],'GM',[],'GR',[],...
         'GFC',[],'MussvBnds',[],'MussvInfo',[],'errormsg',[]);
   else              % Complex mu synthesis
      dkinfo{iter_num} = struct('K',[],'Bnd',[],'DL',[],'DR',[],'MussvBnds',[],...
         'MussvInfo',[],'errormsg',[]);
   end
   if iter_num > 1
      maxsens = norm(sens,inf);
      if maxsens==0
         szs = size(sens);
         sens = sens+ones(1,szs(2));
      end
      if MixedMU    % complex/real mu synthesis
         %disp('Mixed mu synthesis')
         if auto == 0
            [icnew,tmpbnd,dlnew,drnew,gmnew,grnew,gfcnew] =  ...
               gmsf(NOMINAL,bnds,muinfo,sens,BLK,clpic_g,autoord,[]);
         else
            [icnew,tmpbnd,dlnew,drnew,gmnew,grnew,gfcnew] =  ...
               gmsfbtch(NOMINAL,bnds,muinfo,sens,BLK,clpic_g,autoord,auto_visual);
         end
         omclp = clpic_g.Frequency;
         kg = frd(K,omclp);
         tmpicg = frd(icnew,omclp);
         xdk = lft(tmpicg,kg);
         mnum = fnorm(xdk);
         if auto==0 || auto_visual == 1
            clf
            uplot(axisvar,[tmpbnd bnds(1,1)],'-',mnum,'--');
            title(['MU bnds (solid) and ||D*M*D^-1|| (dashed): ITERATION  ',int2str(iter_num)])
            drawnow;
            pause(2)
         end
         dl = dlnew;
         dr = drnew;
         gm = gmnew;
         gr = grnew;
         gfc = gfcnew;
         IC = icnew;
         
         if GMAX_PLAN == 1
            maxmnum = norm(mnum,inf);
            GMAX = 1.2*maxmnum;
         elseif GMAX_PLAN == 2
            maxmnum = norm(lft(IC,K),inf);
            GMAX = 1.02*maxmnum;
         end
      else
         %disp('Complex mu synthesis')
         if auto == 0
            [dlnewj,drnewj] =  ...
               xmsf(clpic_g,bnds,muinfo,sens,abs(BLK),[],DISCRETE);
         else
            [dlnewj,drnewj] =  ...
               xmsfbatch(clpic_g,bnds,muinfo,sens,abs(BLK),...
               autoord(1),auto_visual,DISCRETE);
         end
         tmpdlg = frd(dlnewj,clpic_g.Frequency);
         tmpdrg = frd(drnewj,clpic_g.Frequency);
         x_g = tmpdlg*clpic_g/tmpdrg;
         mnum = fnorm(x_g);
         if auto==0 || auto_visual == 1
            clf
            uplot(axisvar,bnds,'-',mnum,'--');
            title(['MU bnds (solid) and ||D*M*D^-1|| (dashed): ITERATION  ',int2str(iter_num)])
            drawnow;
            pause(2)
         end
         dl = blkdiag(dlnewj,eye(NMEAS));
         dr = blkdiag(drnewj,eye(NCONT));
         IC = dl*NOMINAL/dr;
         %oldgm = GMAX;
         if GMAX_PLAN == 1
            maxmnum = norm(mnum,inf);
            GMAX = 1.2*maxmnum;
         elseif GMAX_PLAN == 2
            x = lft(NOMINAL,K);
            maxmnum = norm(dlnewj*x/drnewj,inf);
            GMAX = 1.02*maxmnum;
         end
      end
      % GTOL = 0.02*(GMAX - GMIN);
      % Tighten relative tolerance GTOL as GMAX decreases, with safeguards
      % 0.001<=GTOL<=10
      GTOL = 10^min(1,max(-3,round(log10(GMAX)-2)));
   else
      IC = NOMINAL;
   end
   if auto==0 || auto_visual==1
      disp(' ');disp(' ');disp(' ')
      disp(['Iteration Number:  ', int2str(iter_num)])
      disp(char(ones(1,19+length(int2str(iter_num)))*'-'));disp(' ')
   end
   
   %   Design an H-infinity control law for the interconnection structure in IC
   if auto==0
      disp('Information about the Interconnection Structure IC:');
      size(IC)
   end
   %    [K,clpic,clpic_g,gf,OMEGA,errormsg] = ...
   %      LOCALhinfsyn(IC,NMEAS,NCONT,opt,iter_num,NOMINAL,OMEGA,GMIN,GMAX,GTOL,axisvar);
   GMAX = Inf; % EVIDENCE THIS CAN BE SET TOO TIGHT
   [K,g,g_g,gf,OMEGA,errormsg] = ...
      LOCALhinfsyn(IC,NMEAS,NCONT,opt,iter_num,NOMINAL,OMEGA,GMIN,GMAX,GTOL,axisvar);
   if ~isempty(errormsg)
      dkinfo{iter_num}.errormsg = errormsg;
   else
      dkinfo{iter_num}.errormsg = [];
   end
   
   if auto==0
      disp(' ')
   end
   clpic = lft(NOMINAL,K);
   clpic_g = frd(clpic,OMEGA);
   
   %   now calculate an upper and lower bounds for mu for the control design
   if isempty(errormsg)
      if MixedMU  % Real/Complex mu synthesis
         if auto_visual==1
            disp('Calculating MU of closed-loop system:')
            [bnds,muinfo] = mussv(clpic_g,BLK,'c');
         else
            [bnds,muinfo] = mussv(clpic_g,BLK,'s');
         end
         if auto_visual==1
            uplot(axisvar,bnds)
            title(['CLOSED-LOOP MU: CONTROLLER #', int2str(iter_num)])
            xlabel('FREQUENCY  (rad/s)')
            ylabel('MU')
            disp('')
            if auto==0
               disp(' MU plots for control design:      Strike any key to continue')
               pause;
            else
               drawnow
            end
         end
      else     % Complex mu synthesis
         if auto_visual==1
            disp('Calculating MU of closed-loop system:')
            [bnds,muinfo] = mussv(clpic_g,abs(BLK),'c');
            if min(min(BLK)) < 0
               mbnds = mussv(clpic_g,BLK);
            end
         else
            [bnds,muinfo] = mussv(clpic_g,abs(BLK),'s');
            if min(min(BLK)) < 0
               mbnds = mussv(clpic_g,BLK,'s');
            end
         end
         Dscale = muinfo.dvec;
         Dscale_data = Dscale.ResponseData;
         szd = size(Dscale_data);
         Dscale_data = reshape(Dscale_data,[szd(2) szd(3)])';
         Dscale_dataNorm = Dscale_data(:,szd(2)*ones(1,szd(2))).\Dscale_data;
         Dscale_dataNorm = reshape(Dscale_dataNorm',[1 szd(2) szd(3)]);
         Dscale.ResponseData = Dscale_dataNorm; % normalized D
         
         if min(min(BLK)) < 0 && auto_visual==1
            clf; subplot(211)
            uplot(axisvar,bnds)
            title(['CLOSED-LOOP COMPLEX MU: CONTROLLER #', int2str(iter_num)])
            xlabel('FREQUENCY  (rad/s)')
            ylabel('COMPLEX MU')
            subplot(212)
            uplot(axisvar,mbnds)
            title(['CLOSED-LOOP MIXED MU: CONTROLLER #', int2str(iter_num)])
            xlabel('FREQUENCY  (rad/s)')
            ylabel('MIXED MU')
            %subplot(111)
            disp('')
            if auto==0
               disp(' MU plots for control design:      Strike any key to continue')
               pause;
            else
               drawnow
            end
         elseif auto_visual==1
            uplot(axisvar,bnds)
            title(['CLOSED-LOOP MU: CONTROLLER #', int2str(iter_num)])
            xlabel('FREQUENCY  (rad/s)')
            ylabel('MU')
            disp('')
            if auto==0
               disp(' MU plots for control design:      Strike any key to continue')
               pause;
            else
               drawnow
            end
         end
      end
      
      %----------------------------------------------------%
      %                                                    %
      %    YOU CAN EASILY ADD MORE PLOTS/CALCULATIONS      %
      %          AND DISPLAYS AT THIS POINT                %
      %                                                    %
      %----------------------------------------------------%
      
      if isa(K,'double')
         nx_K = 0;
      else
         nx_K = length(K.StateName);
      end
      if isa(dl,'double')
         nx_dl = 0;
      else
         nx_dl = length(dl.StateName);
      end
      if isa(dr,'double')
         nx_dr = 0;
      else
         nx_dr = length(dr.StateName);
      end
      prevd = nx_dl + nx_dr;    % order of D's
      maxbnds = norm(bnds(1,1),inf);
      mtype = [iter_num; nx_K; prevd; gf; maxbnds];
   else
      mtype = [iter_num; 0; 0; inf; inf];
   end
   dispdata = [dispdata mtype];
   [mrows,mcols] = size(dispdata);
   if mcols > 5
      mtype = dkdispla(dispdata(:,mcols-4:mcols));
   else
      mtype = dkdispla(dispdata);
   end
   if isempty(errormsg)
      % update DKINFO structure
      maxbnd = norm(bnds(1,1),inf);
      dkinfo{iter_num}.K         = K;
      dkinfo{iter_num}.Bnd       = maxbnd;
      dkinfo{iter_num}.DL        = dl;
      dkinfo{iter_num}.DR        = dr;
      if MixedMU    % Real/Complex mu synthesis
         dkinfo{iter_num}.GM        = gm;
         dkinfo{iter_num}.GR        = gr;
         dkinfo{iter_num}.GFC       = gfc;
      end
      dkinfo{iter_num}.MussvBnds = bnds;
      dkinfo{iter_num}.MussvInfo = muinfo;
      sens = muinfo.sens;
      if strcmpi(opt.AutoIterSmartTerminate,'On') && ...
            strcmpi(opt.AutoIter,'On')
         again = 1;
         if iter_num > opt.StartingIterationNumber+1
            stophist(1) = norm(dkinfo{iter_num-2}.MussvBnds(1,1),inf);
            stophist(2) = norm(dkinfo{iter_num-1}.MussvBnds(1,1),inf);
            stophist(3) = norm(dkinfo{iter_num}.MussvBnds(1,1),inf);
            stopdiff = diff(stophist);
            rtol = opt.AutoIterSmartTerminateTol*stophist(end);
            if all(abs(stopdiff)<rtol) || stopdiff(end)>20*rtol
               again = 0; % terminate DK iteration due to no progress or increase
            else
               again = 1;
            end
         end
      end
   else
      dkinfo{iter_num}.K = [];
      dkinfo{iter_num}.Bnd = inf;
      dkinfo{iter_num}.DL = [];
      dkinfo{iter_num}.DR = [];
      if MixedMU    % Real/Complex mu synthesis
         dkinfo{iter_num}.GM  = [];
         dkinfo{iter_num}.GR  = [];
         dkinfo{iter_num}.GFC = [];
      end
      dkinfo{iter_num}.MussvBnds = [];
      dkinfo{iter_num}.MussvInfo = [];
      again = 0;
   end
   %    first = 0;     % no longer first iteration
   iter_num = iter_num + 1;
   if auto_visual==1
      clc;home
      disp(mtype)
      disp(' ')
   end
   if auto==0
      ag = notherdk(1);
      if ag > 0
         again = 1;
      else
         again = 0;
      end
   end
end  %while again
if auto_visual==1
   disp([' Next MU iteration number:  ', int2str(iter_num)]);
end

%Construct output variables K, clpic and maxbnd from closed-loop system
% that achieved the smallest maxbnd
bnd = arrayfun(@(x) x{1}.Bnd,dkinfo);
[maxbnd,mini] = min(bnd);
if maxbnd<Inf
  K = dkinfo{mini}.K;
  clpic = lft(P,K);
else
  K=[];
  clpic=[];
end

%----------------------- End of DKSYN ------------------------------------------%
function [K,clpic,clpic_g,gf,OMEGA,errormsg] = ...
   LOCALhinfsyn(IC,NMEAS,NCONT,opt,iter_num,NOMINAL,OMEGA,GMIN,GMAX,GTOL,axisvar)

K = []; clpic = []; clpic_g = []; gf = []; errormsg = [];
% hinfsyn parameters which will disappear
% RICMTHD = 2;
EPP = 1e-6;
EPR = 1e-8;
hinf_go = 1;
cnt = 0;
if strcmpi(opt.DisplayWhileAutoIter,'On') || ...
      strcmpi(opt.AutoIter,'Off')
   auto_visual = 1;
   hinfdisp = 'On';
else
   auto_visual = 0;
   hinfdisp = 'Off';
end
while hinf_go
   if ~isempty(opt.InitialController) && opt.StartingIterationNumber==iter_num
      K = opt.InitialController;
      g = lft(IC,K);
      gf = norm(g,inf);
   else
      % calls RCT HINFSYN
      [K,g,gf] = hinfsyn(IC,NMEAS,NCONT,'GMAX',GMAX,'GMIN',GMIN,'TOLGAM',GTOL,...
         'DISPLAY',hinfdisp);
      % figure(1), sensplot(g)
   end
   SKIP = 1;
   if isinf(gf)
      errormsg = 'GMAX found to be infeasible';
      SKIP = 0;
      hinf_go = 0;
   end
%    if isempty(gf)
%       SKIP= 0;
%       if strcmpi(opt.AutoIter,'Off')
%          disp('----  Upper bound for GAMMA is too small!!  ----');
%          disp('-----  Need to increase GAMMA Upper bound  -----');
%          [GMAX,GMIN,GTOL,EPP,EPR] = chhsparm(GMAX,GMIN,GTOL,EPP,EPR);
%       else
%          % GMIN = GMAX;
%          GMAX = 2*GMAX;
%          GTOL = 2*GTOL;
%          cnt = cnt + 1;
%          if cnt == 15
%             errormsg = 'Cannot find a stabilizing controller, check GMAX';
%             SKIP = 0;
%             hinf_go = 0;
%          end
%       end
%    elseif gf==-1           % A, B_2 unstabilizable
%       SKIP = 0;
%       errormsg = 'Open-Loop Interconnection is not stabilizable through U (B_2)';
%       hinf_go = 0;
%    elseif gf==-2           % A, C_2 detectable
%       SKIP = 0;
%       errormsg = 'Open-Loop Interconnection is not detectable through Y (C_2)';
%       hinf_go = 0;
%    end
   if SKIP==1
      chflg = 1;
      while chflg == 1
         clpic = lft(NOMINAL,K);
         clpic_g = frd(clpic,OMEGA);
         g_g = frd(g,OMEGA);  % has rational scalings
         if auto_visual==1
            g_gs = svd(g_g);
            uplot(axisvar,g_gs)
            title('SINGULAR VALUE PLOT: CLOSED-LOOP RESPONSE')
            xlabel('FREQUENCY (rad/s)')
            ylabel(' MAGNITUDE')
            drawnow
         end
         if strcmpi(opt.AutoIter,'Off')
            disp(' '); disp(' ')
            disp('Singular Value plot of closed-loop system in GRAPHICS window')
            disp('Make sure that chosen Frequency range is appropriate')
            RE = reomega(1);
            if RE == 1
               [OMEGA,chflg] = chomega(OMEGA(:));
               axisvar = 'liv,m';
               if abs(max(diff(OMEGA)) - min(diff(OMEGA))) < 1e-5
                  axisvar = 'iv,m';
               end
            else
               chflg = 0;
            end
            hinf_go = 0;
         else
            chflg = 0;
            hinf_go = 0;
         end
      end
   end
end     % while hinf_go
