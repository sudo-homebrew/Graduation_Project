function slashedName = addLeadingSlash(name)
%addLeadingSlash Add a leading slash if the name does not have one

%   Copyright 2020 The MathWorks, Inc.

    if ~strncmp(name, '/', 1)
        slashedName = ['/' name];
    else
        slashedName = name;
    end
end
