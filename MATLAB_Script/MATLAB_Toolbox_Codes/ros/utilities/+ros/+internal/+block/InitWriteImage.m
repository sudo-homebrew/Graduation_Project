function InitWriteImage(block)
%This function is for internal use only. It may be removed in the future.

%%InitWriteImage - intilaization function for the Write Image block
% Does not support bayer encodings

%   Copyright 2021 The MathWorks, Inc.

    allEncodings = ros.msg.sensor_msgs.internal.ImageEncoding.NonBayerEncodings;
    allAlphaEncodings = ros.msg.sensor_msgs.internal.ImageEncoding.AlphaEncodings;
    % get mask and ensure self modification is enabled
    msk = Simulink.Mask.get(block);
    msk.SelfModifiable = 'on';
    params = msk.Parameters;
    portDisplay = "port_label('input',1,'Image');port_label('output',1,'Msg');"; % required ports
    for iParam = 1:length(params)
        switch params(iParam).Name
          case 'ImageEncoding'
            set(params(iParam), 'TypeOptions', allEncodings);
            selectedEncoding = params(iParam).Value;
          case 'EncodingStruct'
            set(params(iParam), 'Value', ['ros.msg.sensor_msgs.internal.ImageEncoding.info(coder.const(''' selectedEncoding '''))']);
          case 'HasAlpha'
            if any(strcmp(selectedEncoding, allAlphaEncodings))
                params(iParam).Value = 'on';
            else
                params(iParam).Value = 'off';
            end
            if strcmp(params(iParam).Value,'on')
                replace_block([block '/Alpha'],'Ground', 'Inport','noprompt');
                portDisplay = portDisplay + "port_label('input',2,'Alpha');";
            else
                replace_block([ block '/Alpha'], 'Inport','Ground', 'noprompt');
            end
        end
    end
    msk.Display = portDisplay;

end
