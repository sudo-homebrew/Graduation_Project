function libraryName = validateAndGetLibraryName
%This function is for internal use only. It may be removed in the future.

%validateAndGetLibraryName return shared library name.
%Further, it checks whether shared library is already loaded
%in the MATLAB memory

%   Copyright 2019-2020 The MathWorks, Inc.

% shared library name
    libraryName = 'msgproto';

    % check shared library is loaded in the MATLAB memory
    if libisloaded( ['lib',libraryName] )
        error(message('robotics:robotgazebo:gazebogenmsg:SharedLibraryIsLoaded'))
    end

end
