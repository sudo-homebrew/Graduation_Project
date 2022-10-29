function InitWritePointCloud(block)
%This function is for internal use only. It may be removed in the future.

%%InitWritePointCloud - intilaization function for the Write Point Cloud block

%   Copyright 2021 The MathWorks, Inc.

    msk = Simulink.Mask.get(block);
    msk.SelfModifiable = 'on';
    params = msk.Parameters;
    portDisplay = "port_label('input',1,'XYZ');port_label('output',1,'Msg');";
    InportPort = 2;
    for iParam = 1:length(params)
        switch params(iParam).Name
          case 'Encoding'
            set(params(iParam), 'TypeOptions', {'none', 'rgb', 'rgba'});
            SelectedEncoding = params(iParam).Value;
          case 'FieldNamesStruct'
            if HasRGBValue && ~HasAlphaValue
                set(params(iParam), 'Value', ['struct(''x'', ''single'', ''y'', ''single'', ''z'', ''single'', ''rgb'', ''uint32'')']);
            elseif HasRGBValue && HasAlphaValue
                set(params(iParam), 'Value', ['struct(''x'', ''single'', ''y'', ''single'', ''z'', ''single'', ''rgba'', ''uint32'')']);
            else
                set(params(iParam), 'Value', ['struct(''x'', ''single'', ''y'', ''single'', ''z'', ''single'')']);
            end
          case 'HasRGB'
            HasRGBValue = any(strcmp(SelectedEncoding,{'rgb', 'rgba'}) );
            params(iParam).Value = string(HasRGBValue);
            if HasRGBValue
                replace_block([block '/RGB'],'Ground', 'Inport','noprompt')
                portDisplay = portDisplay +  ...
                    convertCharsToStrings(['port_label(''input'',' num2str(InportPort) ',''RGB'');']);
                InportPort = InportPort + 1;
            else
                replace_block([block '/RGB'], 'Inport', 'Ground', 'noprompt');
            end
          case 'HasAlpha'
            HasAlphaValue = strcmp(SelectedEncoding, 'rgba');
            params(iParam).Value = string(HasAlphaValue);
            if HasAlphaValue
                replace_block([block '/Alpha'], 'Ground', 'Inport','noprompt');
                portDisplay = portDisplay +  ...
                    convertCharsToStrings(['port_label(''input'',' num2str(InportPort) ',''Alpha'');']);
                InportPort = InportPort + 1;
            else
                replace_block([ block '/Alpha'], 'Inport','Ground', 'noprompt');
            end
        end
    end
    msk.Display = portDisplay;
end
