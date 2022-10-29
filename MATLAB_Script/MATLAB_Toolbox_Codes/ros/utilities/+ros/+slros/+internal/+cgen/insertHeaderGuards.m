function insertHeaderGuards(buf, headerName)
%This function is for internal use only. It may be removed in the future.

%   insertHeaderGuards(BUF, HDR) takes a StringWriter object BUF (assumed
%   to hold contents of a C/C++ header file) and inserts preprocessor
%   guards.
%
%   For example, if HDR is 'foo.h', then the guards look like this:
%     #ifndef _FOO_H_
%     #define _FOO_H_
%     /* original contents of BUF */
%     #end _FOO_H_

%   Copyright 2014-2018 The MathWorks, Inc.

% Replace periods with underscores, remove all non-alphanumeric
% non-underscore characters, and put underscores around the tag
headerTag = ['_' upper(regexprep(headerName, {'\.', '\W'}, {'_', ''})) '_'];

newbuf= StringWriter;
newbuf.addcr('#ifndef %s', headerTag);
newbuf.addcr('#define %s', headerTag);
newbuf.addcr;

buf.insert(1,newbuf);
buf.craddcr('#endif');
