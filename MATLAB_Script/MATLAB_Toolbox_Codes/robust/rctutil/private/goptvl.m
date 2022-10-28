% function val = goptvl(opt,tag,defval,nfval)

%   Copyright 1991-2006 MUSYN Inc. and The MathWorks, Inc.

function val = goptvl(opt,tag,defval,nfval)

val = nfval;
if any(opt==tag(1))
    loc = find(opt==tag(1));
    if length(opt)>loc
        %val = str2double(opt(loc+1));
                
        % PJS:  mod so that val can be more than one digit
        tmp = [(opt(loc+1:end)>=48) & (opt(loc+1:end)<=57) 0];
        idx = min(find( tmp==0 ) )-1;
        val = str2double( opt(loc+1:loc+idx) );        
        if isnan(val)
            val = defval;
        end
    else
        val = defval;
    end
end
