function rcthelp(topicid) 
% RCTHELP Show help for the Robust Control Toolbox based on the input topic
% id.
 
% Author(s): John W. Glass 17-Dec-2008
% Copyright 2008 The MathWorks, Inc.

mapfile_location = fullfile(docroot,'toolbox','robust','helptargets.map');
helpview(mapfile_location, topicid);