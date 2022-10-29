classdef ImageLicense
%This class is for internal use only. It may be removed in the future.

%ImageLicense Helper class for determining license status
%   In particular, the license status for the Image Processing Toolbox
%   (IPT) can be retrieved through this class. If IPT is installed and
%   accessible, more advanced functionality is available in reading and
%   displaying of images.

%   Copyright 2014-2020 The MathWorks, Inc.

    methods (Static, Access = {?ros.msg.sensor_msgs.internal.ImageReader, ...
                            ?ros.msg.sensor_msgs.internal.ImageDisplay, ...
                            ?matlab.unittest.TestCase})

        function [isLicensed, errStr, msgObj] = isIPTLicensed
        %isIPTLicensed Returns true if IPT is installed and licensed
        %   This function checks if the Image Processing Toolbox (IPT)
        %   is installed and its license can be used. If this is the
        %   case, additional functionality like demosaicing and imshow
        %   become available.

            persistent isInstalled

            if coder.target('MATLAB') && ~isdeployed
                if isempty(isInstalled)
                    isInstalled = ~isempty(ver('images'));
                end

                % IPT is usable if it is installed and the license test returns
                % positively
                isLicensed = license('test', 'Image_Toolbox') && isInstalled;
            else
                % ver is not supported in codegen or when deploying with MATLAB
                % Compiler. Assume IPT is installed.
                isLicensed = true;
            end

            if isLicensed
                errStr = '';
                msgObj = [];
            else
                msgObj = message('ros:mlroscpp:image:NoIPTLicense');
                errStr = getString(msgObj);
            end
        end
    end

end
