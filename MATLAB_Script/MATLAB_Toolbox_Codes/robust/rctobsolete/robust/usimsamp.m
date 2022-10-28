function NVout = usimsamp(sname,bw)
%USIMSAMP  Generates a random instance of all uncertain atoms in 
%a Simulink model
%
%   SAMPLE = USIMSAMP(SNAME) returns a structure of random samples of all
%   uncertain atoms associated with the USS System blocks within the  
%   Simulink model SNAME.  SAMPLE is a 1-by-1 structure array whose field 
%   names and values are the names and sample values of the uncertain atoms 
%   within all of the USS System blocks contained in the Simulink model.
%
%   SAMPLE = USIMSAMP(SNAME,BW) the scalar parameter BW effectively
%   defines a bandwidth which determines how ULTIDYN elements within SNAME 
%   are sampled. See USAMPLE help for details.
%
%   USIMSAMP is useful for Monte-Carlo analysis of uncertain systems.
%
%   See also usample, usimfill, usiminfo, usubs.

% Authors: Gary J. Balas and Andy K. Packard
%   Copyright 2007-2012 The MathWorks, Inc.
warning('Robust:obsolete:USIMSAMP',...
   'USIMSAMP and the "USS System" block are deprecated. Use SLUPDATE to upgrade these blocks and USAMPLE to sample uncertain variables.')
N = 1;
if nargin==1
   bw = [];
elseif nargin>2
   error('Incorrect call to USIMSAMP');
end
hw = ctrlMsgUtils.SuspendWarnings; %#ok<NASGU>
%lwarn = lastwarn; warnstate = warning('off','all');

[cflag,allupaths,allunames,upaths,unames,csumchar,syscell] = ...
    usiminfo(bdroot(sname));
newsys = [];

TSPresent = zeros(0,1);
allsys = cell(0,1);
NVout = struct([]);

if cflag==1 %uncertain parameters are consistent in Simulink diagram
   if ~isempty(syscell)
      for i=1:length(syscell)
         TS = get(uss(syscell{i}),'Ts');
         loc = find(TS==TSPresent);
         if isempty(loc)
            TSPresent = [TSPresent;TS];
            allsys = [allsys;syscell(i)];
         elseif length(loc)==1
            allsys{loc} = blkdiag(allsys{loc},syscell{i});
         end
      end
      if length(TSPresent)==1
         if isempty(bw)
            [out,NVout] = usample(allsys{1},N);
         else
            [out,NVout] = usample(allsys{1},N,bw);
         end
      else
         % Use USAMPLE.  Note, since CFLAG==1, everything is consistent
         % and we can do this in any order we want.
         for i=1:length(allsys)
            fnout = fieldnames(NVout);
            if ~isempty(fnout) && isuncertain(allsys{i})
               tmpsys = usubs(allsys{i},fieldnames(NVout),'Nominal');
            else  % we should make USUBS work on USUBS({},'Nominal',...)
               tmpsys = allsys{i};
            end
            % GJB: Check if TMPSYS is uncertain?
            if isuncertain(tmpsys)
               if isempty(bw)
                  [dum,NVouti] = usample(tmpsys,N);
               else
                  [dum,NVouti] = usample(tmpsys,N,bw);
               end
               fni = fieldnames(NVouti);
            end
            if isempty(fnout)
               NVout = NVouti;
            else
               % merge the two structures (isn't there an easier way?)
               for k=1:length(fni)
                  NVout.(fni{k}) = NVouti.(fni{k});
               end
            end
         end
      end
   else
      NVout = struct([]);
   end
   %warning(warnstate);lastwarn(lwarn);
else
   %warning(warnstate);lastwarn(lwarn);
   error('USS System blocks have incompatible uncertainty descriptions')
end
