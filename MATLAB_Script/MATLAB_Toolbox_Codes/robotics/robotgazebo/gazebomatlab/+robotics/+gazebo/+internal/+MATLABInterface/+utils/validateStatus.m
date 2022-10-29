function validateStatus(status, modelname, linkjointname, axisindex)
%This function is for internal use only. It may be removed in the future.
%
% This function throw error based on input status. It uses modelname,
% linkname, jointname and axis index to generate valid error message.

%   Copyright 2020 The MathWorks, Inc.

    switch(status)
      case 1
        error(message('robotics:robotgazebo:gzsupport:InvalidModelName',...
                      modelname,'gzmodel("list")'));
      case 2
        instMsg = [ 'gzlink("list","' modelname,'")'];
        error(message('robotics:robotgazebo:gzsupport:InvalidLinkName',...
                      linkjointname, modelname, instMsg));
      case 3
        instMsg = [ 'gzjoint("list","' modelname,'")'];
        error(message('robotics:robotgazebo:gzsupport:InvalidJointName',...
                      linkjointname, modelname, instMsg));
      case 4
        error(message('robotics:robotgazebo:gzsupport:JointAxisNone',...
                      linkjointname, modelname));
      case 5
        error(message('robotics:robotgazebo:gzsupport:InvalidAxisIndex',...
                      axisindex, linkjointname, modelname));
    end

end
