function helpview(topicId)
%This function is for internal use only. It may be removed in the future.

%helpview - Invoke Doc Browser for specified topicId

%   Copyright 2014-2018 The MathWorks, Inc.

validateattributes(topicId, {'char'}, {'nonempty'});
helpview(fullfile(docroot, 'ros', 'helptargets.map'), topicId);
