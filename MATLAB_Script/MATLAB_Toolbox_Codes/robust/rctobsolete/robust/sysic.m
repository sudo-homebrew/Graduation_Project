function sysout = sysic
% SYSIC  System Interconnection.
%
%   SYSIC forms interconnections of linear systems (SS/TF/ZPK), as
%   well as uncertain systems (USS), constant matrices, and frequency
%   response data (FRD/UFRD).  Variables 'systemnames', 'outputvar'
%   and 'inputvar' must be in the calling workspace, along with
%   'input_to_' variables for the named systems listed in 'systemnames'.
%   
%   Variable Specifications
%
%   SYSTEMNAMES: Use a character string to specify all systems (by their
%   variable name) in the interconnection with the variable 'systemnames'.
%   For example, an interconnection involving 4 systems (SS, FRD, TF, ZPK,
%   USS, etc) named 'Plant', 'Actuator', 'K', 'refmodel' would use
%        systemnames = 'Plant Actuator K refmodel'
%   The order of the names is not important.
%
%   INPUTVAR: Specify the input signal arguments in a vector form, using a
%   character string, with square brackets.  Non-scalar signals can be
%   defined using a {}, with a given dimension.  Undimensioned signals are
%   interpreted as scalars.  For example, the command
%        inputvar = '[ref{2};Wind;Force{3};Noise{4}]'
%   specifies a 10-by-1 input vector consisting of a 2-by-1 signal named
%   'ref', a scalar signal named 'Wind', a 3-by-1 signal named 'Force' and
%   a 4-by-1 signal named 'Noise'.
%
%   OUTPUTVAR: The outputs of the interconnection are linear combinations
%   of the signals listed in INPUTVAR and outputs of the systems listed in
%   SYSTEMNAMES.  Use a character string, with square brackets to specify
%   the full output vector.  For example,
%        outputvar = '[Plant+Noise;K(1);Actuator(1:2)+Force(1:2)]';
%   specifies that the output of the interconnection will be all of the
%   outputs of Plant added to the Noise signal, followed by the first
%   output of K, followed by the first two outputs of Actuator added to the
%   first two Force signals.
%
%   INPUT_TO_:  The inputs to every system listed in SYSTEMNAMES must be
%   specified in the same manner as OUTPUTVAR.   Inputs to each system are
%   linear combinations of outputs of any system and the variables defined
%   in INPUTVAR.
%
%   Example
%   p = tf(1,[1 1]);
%   c = tf([1.8 4],[1 0]);
%   systemnames = 'p c';
%   inputvar = '[r]';
%   outputvar = '[p]';
%   input_to_c = '[r - p]';
%   input_to_p = '[c]';
%   G = sysic;  % yields same result as  G = feedback(p*c,1);
%
%   See also: ICONNECT, ICSIGNAL.

%   Copyright 2003-2011 The MathWorks, Inc.

Esn = evalin('caller','exist(''systemnames'',''var'');');
Isn = evalin('caller','exist(''inputvar'',''var'');');
Osn = evalin('caller','exist(''outputvar'',''var'');');
Csn = evalin('caller','exist(''cleanupsysic'',''var'');');
Ssn = evalin('caller','exist(''sysoutname'',''var'');');
if ~Esn
   ctrlMsgUtils.error('Robust:umodel:MissingWSVariable','systemnames')
else
   systemnames = evalin('caller','systemnames;');
end
if ~Isn
   ctrlMsgUtils.error('Robust:umodel:MissingWSVariable','inputvar')
else
   inputvar = evalin('caller','inputvar;');
end
if ~Osn
   ctrlMsgUtils.error('Robust:umodel:MissingWSVariable','outputvar')
else
   outputvar = evalin('caller','outputvar;');
end
if ~Csn
   cleanupsysic = 'no';
else
   cleanupsysic = evalin('caller','cleanupsysic;');
end
if ~Ssn
   sysoutname = '';
else
   sysoutname = evalin('caller','sysoutname;');
end

[numsys,sysnames,err] = LOCALnamstuff(systemnames);
if err == 1
   ctrlMsgUtils.error('Robust:umodel:CheckWSVariable','SYSTEMNAMES')
end
sysvals = cell(numsys,1);

[numxinp,xndf,xipnt,xinames,err,inpnames] = LOCALinpstuff(inputvar);
if err == 1
   ctrlMsgUtils.error('Robust:umodel:CheckWSVariable','INPUTVAR')
end

flagvec = zeros(numsys,1);
para = [];
inpoint = zeros(1,numsys+xndf);
outpoint = zeros(1,numsys+xndf);
sysdata = zeros(3,numsys+xndf);
for i=1:numsys
   Ssn = evalin('caller',['exist(''' sysnames{i} ''',''var'');']);
   if Ssn ~= 1
      ctrlMsgUtils.error('Robust:umodel:SystemNotFound',sysnames{i});
   end
   tmp = evalin('caller',sysnames{i});
   [sysvals{i},flagvec(i)] = LOCALmat2ss(tmp);
   szm = size(sysvals{i});
   sysdata(1,i) = 0; % unsure what this is...
   sysdata(2,i) = szm(1); % number of outputs
   sysdata(3,i) = szm(2); % number of inputs
   % INPOINT(i)+j points to the j'th input of SYSTEM_i.
   % OUTPOINT(i)+k points to the k'th output of SYSTEM_i.
   % Both have the external inputs included at the end (numsys+xndf)
   para = blkdiag(para,sysvals{i});
   if i > 1
      inpoint(i) = sysdata(3,i-1) + inpoint(i-1);
      outpoint(i) = sysdata(2,i-1) + outpoint(i-1);
   end
end
if (any(flagvec==-1) || any(flagvec==-2)) && any(flagvec==1)
   error('Incompatible objects: mutools and uncertain or LTI objects')
end

inpoint(numsys+1) = inpoint(numsys) + sysdata(3,numsys);
outpoint(numsys+1) = outpoint(numsys) + sysdata(2,numsys);
sysdata(:,numsys+1) = [0; xipnt(1,2) ; xipnt(1,2)];
for i=2:xndf
   inpoint(numsys+i) = inpoint(numsys+i-1) + xipnt(i-1,2);
   outpoint(numsys+i) = outpoint(numsys+i-1) + xipnt(i-1,2);
   sysdata(:,numsys+i) = [0; xipnt(i,2) ; xipnt(i,2)];
end
names = [sysnames;xinames];

para = blkdiag(para,eye(numxinp));
szp = size(para);
allrows = szp(1);
allcols = szp(2);
numsysin = allcols - numxinp;
numsysout = allrows - numxinp;

% DETERMINE NUMBER OF OUTPUTS
numout = 0;
[ard,arl,er] = LOCALpass1(outputvar);
if er ~= 0
   ctrlMsgUtils.error('Robust:umodel:SyntaxProblem','OUTPUTVAR');
end
outpnames = cell(length(arl),1);
numwiththis = zeros(size(ard,1),1);
cnt = 1;
for i=1:length(arl)
   [od,odl,fsys,gains,er] = LOCALpass2(i,ard,arl,names,sysdata); %#ok<*ASGLU>
   try
      numwiththis(i) = numel(eval(['[' od(1,:) ']']));
   catch %#ok<*CTCH>
      % This error was originally displayed slightly differently
      ctrlMsgUtils.error('Robust:umodel:SyntaxProblem','OUTPUTVAR');
   end
   if er ~= 0
      ctrlMsgUtils.error('Robust:umodel:SyntaxProblem','OUTPUTVAR');
   end
   sz = 0;
   for j = 1:length(odl)
      [out,er] = LOCALpass3(j,od,odl);
      if er ~= 0
         ctrlMsgUtils.error('Robust:umodel:InvalidCommaColon');
      end
      if sz == 0
         sz = length(out);
      else
         if sz ~= length(out)
            ctrlMsgUtils.error('Robust:umodel:InconsistentSignalNumber',['''' ard(i,1:arl(i)) '''']);
         end
      end
   end
   numout = numout + length(out);
   for j=1:numwiththis(i)
      % XXX
      if numwiththis(i)>1
         yexstr = ['(' int2str(j) ')'];
      else
         yexstr = '';
      end
      outpnames{cnt} = ['[' deblank(ard(i,:)) ']' yexstr];
      cnt = cnt + 1;
   end
end

% Feedback interconnection matrix starts as EMPTY
myk = [];
for k=1:numsys+1
   location = 0;  %  location in feedback matrix for input_to_sysi
   if k <= numsys
      Vname = ['input_to_' sysnames{k}];
      IVsn = evalin('caller',['exist(''' Vname ''',''var'');']);
      if IVsn ~= 1
         ctrlMsgUtils.message('Robust:umodel:NoInputs',sysnames{k});
      else
         Vk = evalin('caller',[Vname ';']);
      end
      ff = zeros(sysdata(3,k),allrows);
   else
      Vname = 'outputvar';
      Vk = outputvar;
      ff = zeros(numout,allrows);
   end
   [ard,arl,er] = LOCALpass1(Vk);
   if er>0
      initMess =  ctrlMsgUtils.message('Robust:umodel:CheckExp',[Vname '= ''' Vk '''']);
      switch er
         case 1
            finalMess = ctrlMsgUtils.message('Robust:umodel:MissingBrackets');
         case 2
            finalMess = ctrlMsgUtils.message('Robust:umodel:InconsistentParen');
         case {3,4}
            finalMess = ctrlMsgUtils.message('Robust:umodel:ParenWrong');
         case 5
            finalMess = ctrlMsgUtils.message('Robust:umodel:BracketsInParen');
      end
      error([initMess ' ' finalMess]);
   end
   szin = 0;
   for i=1:length(arl)
      %     for each group, LOCALpass2 splits ARD into the summation of its
      %     parts - each part has the same number of signals, just from
      %     different components (also have constant scaling factors)
      [od,odl,fsys,gains,er] =  LOCALpass2(i,ard,arl,names,sysdata);
      if er>0
         initMess =  ctrlMsgUtils.message('Robust:umodel:CheckExp',[Vname '= ''' Vk '''']);
         switch er
            case 1
               finalMess = ctrlMsgUtils.message('Robust:umodel:TooManySM');
            case 2
               finalMess = ctrlMsgUtils.message('Robust:umodel:ImproperParen');
            case 3
               finalMess = ctrlMsgUtils.message('Robust:umodel:ImbalanceParen');
            case 4
               finalMess = ctrlMsgUtils.message('Robust:umodel:WrongSM');
            case 5
               finalMess = ctrlMsgUtils.message('Robust:umodel:UnknownSystem');
         end
         error([initMess ' ' finalMess]);
      else
         szout = 0;
         for j=1:length(odl)
            [out,er] = LOCALpass3(j,od,odl);
            if szout == 0
               szout = length(out);
            elseif szout ~= length(out)
               initMess =  ctrlMsgUtils.message('Robust:umodel:CheckExp',[Vname '= ''' Vk '''']);
               finalMess = ctrlMsgUtils.message('Robust:umodel:BadSignalCombo');
               error([initMess ' ' finalMess]);
            end
            if er == 1
               initMess =  ctrlMsgUtils.message('Robust:umodel:CheckExp',[Vname '= ''' Vk '''']);
               finalMess = ctrlMsgUtils.message('Robust:umodel:TwoCommas');
               error([initMess ' ' finalMess]);
            elseif er == 2
               initMess =  ctrlMsgUtils.message('Robust:umodel:CheckExp',[Vname '= ''' Vk '''']);
               finalMess = ctrlMsgUtils.message('Robust:umodel:TwoColons');
               error([initMess ' ' finalMess]);
            else
               if max(out)>sysdata(2,fsys(j))
                  ctrlMsgUtils.error('Robust:umodel:IndexExceeds',upper(Vname));
               end
               sysloc = outpoint(fsys(j));
               ff = LOCALgaino(out,gains(j),ff,location,sysloc);
            end
         end
         szin = szin + szout;
         location = location + length(out);
      end
   end
   if k <= numsys
      if sysdata(3,k) ~= szin
         initMess =  ctrlMsgUtils.message('Robust:umodel:CheckExp',[Vname '= ''' Vk '''']);
         finalMess = ctrlMsgUtils.message('Robust:umodel:WrongInputNumber');
         error([initMess ' ' finalMess]);
      end
   end
   myk = [myk ; ff]; %#ok<*AGROW>
end
input2sysMat = myk(1:end-numout,:);
outputMat = myk(end-numout+1:end,:);
% END OF MAIN LOOP

% WRAP IT UP
topd = allcols - numxinp;
topid = eye(topd);
paraout = myk*para;
ic = lft(topid,paraout,topd,topd);
  
try
   if isa(ic,'DynamicSystem')
      ic.InputName = inpnames;
      ic.OutputName = outpnames;
      if isa(para,'DynamicSystem')
         for i=1:numout
            tmp = find(outputMat(i,:));
            if length(tmp)==1 && outputMat(i,tmp)==1
               if ~isempty(para.OutputName{tmp})
                  ic.OutputName{i} = para.OutputName{tmp};
               end
            end
         end
         for i=1:numsysin  % number of systemInputs
            tmp = find(input2sysMat(i,:));
            if length(tmp)==1 && tmp>numsysout && input2sysMat(i,tmp)==1
               if ~isempty(para.InputName{i})
                  ic.InputName{tmp-numsysout} = para.InputName{i};
               end
            end
         end
      end
   end
catch
   ctrlMsgUtils.warning('Robust:umodel:/UnassignedIONames');
end


if any(flagvec==1) % some mutools objects
   ic = lti2mat(ic);
end
if nargout==1
   sysout = ic;
else
   if ~isempty(sysoutname)
      assignin('caller',sysoutname,ic);
   else
      assignin('caller',sysoutname,'ic_ms');
   end
end
if strcmp(cleanupsysic,'yes')
   evalin('caller','clear cleanupsysic');
   if Ssn
      evalin('caller','clear sysoutname');
   end
   evalin('caller','clear outputvar inputvar systemnames');
   for i=1:numsys
      evalin('caller',['clear input_to_' names{i}]);
   end
end
% End of SYSIC.  LOCAL functions below...


function [numxinp,numdiffinp,inppoint,names,err,inpnames] = LOCALinpstuff(inputvar)
err = 0;
startp = 0;
inppoint = zeros(0,2); % from []
names = cell(0,1); % from []
%tmpln = length(inputvar);
tmp = inputvar(inputvar ~= ']' & inputvar ~= '[');
tmp = tmp(tmp ~= ' ');
semic = [0 find(tmp == ';') length(tmp)+1];
numdiffinp = length(semic) - 1;
numxinp = 0;
for i=1:numdiffinp
   var = tmp(semic(i)+1:semic(i+1)-1);
   %  lparen = find(var == '(');
   %  rparen = find(var == ')');
   %  backwards compatible, but supports braces now
   lparen = find(var == '(' | var == '{');
   rparen = find(var == ')' | var == '}');
   if length(lparen) ~= length(rparen)
      err = 1;
      return
   elseif length(lparen) >= 1
      lparen = lparen(1);
      rparen = rparen(end);
      if lparen >= rparen
         err = 1;
         return
      elseif rparen ~= length(var)
         err = 1;
         return
      elseif lparen == rparen - 1
         err = 1;
         return
      else
         numass = eval(var(lparen+1:rparen-1));
         nn = var(1:lparen-1);
         names = [names ; {nn}];
         inppoint = [inppoint ; [startp numass]];
         startp = startp + numass;
         numxinp = numxinp + numass;
      end
   else
      nn = var;
      names = [names ; {nn}];
      inppoint = [inppoint ; [startp 1]];
      startp = startp + 1;
      numxinp = numxinp + 1;
   end
end
inpnames = cell(numxinp,1);
cnt = 1;
for i=1:numdiffinp
   %XXX
   for j=1:inppoint(i,2)
      if inppoint(i,2)>1
         uexstr = ['(' int2str(j) ')'];
      else
         uexstr = '';
      end
      inpnames{cnt} = [names{i} uexstr];
      cnt = cnt + 1;
   end
end


function [numsys,names,err] = LOCALnamstuff(systemnames)
%XXX Should be some error handling at this point
err = 0;
tmp = [' ' systemnames(systemnames ~= ']' & systemnames ~= '[') ' '];
blks = find(tmp == ' ');
blksd = diff(blks);
imp = find(blksd>1);
numsys = length(imp);
names = cell(numsys,1);
for i=1:numsys
   names{i} = tmp(blks(imp(i))+1:blks(imp(i)+1)-1);
end

function [arraydata,arraylen,err] = LOCALpass1(var)
%  simply pulls apart a    INPUT_TO_     or     OUTPUTVAR
%  variable to get the various channels that make it up
%   EXAMPLE:   >> var = '[ andy(3,4) + johanna ; -tedd - keith + john(7)]';
%              >> [arraydata,arraylen,err] = pass1(var);
%
%      THEN  arraydata = (character string array, a line for each
%                                                     set of channels)
%                         +andy(3,4)+johanna
%                         -tedd-keith+john(7)
%
%            arraylen = (integer array, lengths of the strings)
%                         18
%                         19
%
%            err = 0 (integer, signifies all is OK)
err = 0; arraydata = []; arraylen = 0;
var = var(var~=' ');
if var(1) ~= '[' || var(length(var)) ~= ']'
   err = 1;
   arraydata = [];
   arraylen = 0;
else
   var = var(2:length(var)-1);
   if any(var=='[' | var==']')
      err = 5;
      return
   end
   scol = find(var==';');
   lparens = find(var=='(');
   rparens = find(var==')');
   if length(lparens) ~= length(rparens)
      err = 2;
   else
      %    lefts have to occur before rights
      if min(rparens-lparens) <= 0
         err = 3;
      else
         go = 1;
         j = 1;
         while go == 1
            if ~isempty(scol) && length(find(scol(j)>rparens)) ~= length(find(scol(j)>lparens))
               err = 4;
               return
            end
            j = j+1;
            if j > length(scol)
               go = 0;
               arraydata = [];
               arraylen = [];
               tlen = length(var);
               places = [0 scol length(var)+1];
               for i=1:length(scol)+1
                  data = var(places(i)+1:places(i+1)-1);
                  if data(1) ~= '+' && data(1) ~= '-'
                     data = ['+' data];
                  end
                  arraylen = [arraylen ; length(data) ];
                  %arraydata = [arraydata ; data mtblanks(tlen-length(data)) ];
                  arraydata = [arraydata ; data repmat(' ',[1 tlen-length(data)]) ];
               end
               arraydata = arraydata(:,1:max(arraylen));
            end
         end
      end
   end
end


function [od,odl,fromsys,gains,er] = LOCALpass2(i,ard,arl,names,sysdata)
%function [od,odl,fromsys,gains,er] = LOCALpass2(i,ard,arl,names,sysd)
% this works on the data between colons on a INPUT_TO_SYS, determining
% which system the outputs come from (FROMSYS), which particular
% outputs (OD), and the scalar gain (GAINS).
er = 0;
maxl = 0;
od = [];
odl = [];
fromsys = [];
gains = [];
[num,~] = size(ard);
var = ard(i,1:arl(i));
lvar = length(var);
signs = find(var=='+'|var=='-');
signs = [signs length(var)+1];
for j=1:length(signs)-1
   if var(signs(j)) == '+'
      pm = 1;
   else
      pm = -1;
   end
   data = var(signs(j)+1:signs(j+1)-1);
   anymult = find(data=='*');
   if isempty(anymult)
      gains = [gains ; pm];
   elseif length(anymult) > 1
      er = 1;
      return
   else
      %    test to see if const premult the input or output
      if LOCALchstr(data(1:anymult-1)) == 1
         er = 4;
         return
      end
      gains = [gains ; pm*eval(data(1:anymult-1))];
      data = data(anymult+1:length(data));
   end
   lparen = find(data=='(');
   rparen = find(data==')');
   if isempty(lparen) && isempty(rparen)
      [iloc,finderr] = LOCALfindsys(names,data);
      if finderr == 1
         er = 5;
         return
      end
      fromsys = [fromsys ; iloc];
      tmp = ['1:' int2str(sysdata(2,iloc)) ];
      %od = [od ; [tmp mtblanks(3*lvar-length(tmp))] ];
      od = [od ; [tmp repmat(' ',[1 3*lvar-length(tmp)])] ];
      odl = [odl ; length(tmp)];
      if length(tmp) > maxl
         maxl = length(tmp);
      end
   elseif length(lparen) == 1 && length(rparen) == 1
      if (rparen(1) == length(data)) && (lparen(1) < rparen(1))
         [iloc,finderr] = LOCALfindsys(names,data(1:lparen(1)-1));
         if finderr == 1
            er = 5;
            return
         end
         fromsys = [fromsys ; iloc];
         tmp = data(lparen(1)+1:rparen(1)-1);
         %od = [od ; [tmp mtblanks(3*lvar-length(tmp))] ];
         od = [od ; [tmp repmat(' ',[1 3*lvar-length(tmp)])] ];
         odl = [odl ; length(tmp)];
         if length(tmp) > maxl
            maxl = length(tmp);
         end
      else
         er = 2;
         return
      end
   else
      er = 3;
      return
   end
end
od = od(:,1:maxl);

function [out,err] = LOCALpass3(j,od,odl)
var = od(j,1:odl(j));
err = 0;
out = [];
commas = find(var==',');
commas = [0 commas length(var)+1];
if any(diff(commas)==1)
   err = 1;
else
   for jj=1:length(commas)-1
      chunk = var(commas(jj)+1:commas(jj+1)-1);
      colons = find(chunk==':');
      if isempty(colons)
         out = [out ; eval(chunk)];
      elseif length(colons) == 1
         tmp = eval(chunk(1:colons(1)-1)):eval(chunk(colons(1)+1:length(chunk)));
         out = [out ; tmp'];
      else
         err = 2;
      end
   end
end

function [iloc,err] = LOCALfindsys(names,sysname)
iloc = find(strcmp(sysname,names));
err = 0;
if length(iloc)~=1
   err = 1;
end

function ff = LOCALgaino(out,gain,ff,startloc,sysloc)
for i=1:length(out)
   ff(i+startloc,sysloc+out(i)) = ff(i+startloc,sysloc+out(i)) + gain;
end

function [b,flag] = LOCALmat2ss(a)
% flag = 0; % no conversion from a mutools object
if ~isequal(1,mat2lti(1))
   error('MAT2LTI code has changed');
end
switch class(a)
   case{'pss' 'ss' 'tf' 'zpk' 'frd' 'genss' 'genfrd' 'genmat' 'realp'}
      b = a;
      flag = -2;
   case{'umat' 'ufrd' 'uss'}
      b = a;
      flag = -1;
   case{'ultidyn' 'ureal' 'ucomplex' 'ucomplexm' 'udyn' 'umargin'}
      b = a;
      flag = -1;
   case{'double'}
      b = mat2lti(a);
      flag = ~isequal(a,b);
   otherwise
      ctrlMsgUtils.error('Robust:umodel:IllegalSystemClass',class(a));
end

function out = LOCALchstr(in)
for i=0:9
   if ~isempty(in)
      tmp = int2str(i);
      in = in(in ~= tmp);
   end
end
if isempty(in)
   out = 0;
elseif strcmp(in,'.')
   out = 0;
elseif strcmp(in,'e+')
   out = 0;
elseif strcmp(in,'e-')
   out = 0;
elseif strcmp(in,'.e+')
   out = 0;
elseif strcmp(in,'.e-')
   out = 0;
else
   out = 1;
end

