function handleGetPrintOutput( modelname, linkJointName, ...
                               resultString, validParamName, getType)
    %This function is for internal use only. It may be removed in the future.
    %
    % This function print message based on inputs such as modelname,
    % linkname/jointname, output string and parameter name.

    %   Copyright 2020 The MathWorks, Inc.

    switch(getType)
      case 'getModel'
        % print list of model name
        robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
            modelname,'h','MODEL:');
        % print requested parameter name and value
        for pIdx = 1:numel(validParamName)
            robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                resultString(validParamName{pIdx}),'h',upper([validParamName{pIdx},':']));
        end

      case 'getLink'
        % print list of model name
        robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
            modelname,'h','MODEL:');
        robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
            linkJointName,'h','LINK:');
        % print requested parameter name and value
        for pIdx = 1:numel(validParamName)
            robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                resultString(validParamName{pIdx}),'h',upper([validParamName{pIdx},':']));
        end

      case 'getJoint'
        % print model name
        robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
            modelname,'h','MODEL:');
        robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
            linkJointName,'h','JOINT:');
        % print requested parameter name and value
        axisOptions = {'Axis0','Axis1'};
        for pIdx = 1:numel(validParamName)
            if( ~contains(validParamName{pIdx},axisOptions))
                if(contains(validParamName{pIdx},{'Damping','Angle','Friction','XYZ'}) )
                    axisIndex = contains(axisOptions,validParamName);
                    reqAxisNames = axisOptions(axisIndex);
                    for axIdx = 1:numel(reqAxisNames)
                        axisParamName = [reqAxisNames{axIdx},' ',validParamName{pIdx}];
                        axisParamValue = resultString(axisParamName);
                        robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                            axisParamValue,'h',upper([axisParamName,':']))
                    end
                else
                    robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                        resultString(validParamName{pIdx}),'h',upper([validParamName{pIdx},':']));
                end
            end
        end
    end
end
