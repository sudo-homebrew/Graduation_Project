classdef MessageUtil 
%This class is for internal use only. It may be removed in the future.

%   Copyright 2019 The MathWorks, Inc.

    methods (Static)
        function ret = isBoundedArray(msginfo)
            % ISBOUNDEDARRAY Returns true if the message represented by the
            % message information meta-data is a bounded array.
            % An example, message field with bounded array looks as follows:
            %
            %    string[<=5] bounded_string
            %
            % In above example, bounded_string member can have at the most 5 strings.
            %
            % Example:
            %
            %   obj = ros.slros2.internal.bus.Util;
            %   [~, messageInfo] = obj.newMessageFromSimulinkMsgType('test_msgs/BoundedArrayPrimitives');
            %   ret = obj.isBoundedArray(messageInfo);
            
            ret =  isfield(msginfo, 'MaxLen') && ...
                isfield(msginfo, 'MinLen') && ...
                isfinite(msginfo.MaxLen) && ...
                (msginfo.MaxLen > msginfo.MinLen) && ...
                (msginfo.MaxLen > 1);
        end

        function ret = isStaticArray(msginfo)
            % ISSTATICARRAY Returns true if the message represented by the
            % message information meta-data is a static array.
            % An example, message field with static-array looks as follows:
            %
            %    string[5] static_string 
            %
            % In above example, static_string must have exactly 5 entries.
            %
            % Example:
            %
            %   obj = ros.slros2.internal.bus.Util;
            %   [~, messageInfo] = obj.newMessageFromSimulinkMsgType('test_msgs/StaticArrayPrimitives');
            %   ret = obj.isStaticArray(messageInfo);
            
            ret =  isfield(msginfo, 'MaxLen') && ...
                isfield(msginfo, 'MinLen') && ...
                isfinite(msginfo.MaxLen) && ...
                (msginfo.MaxLen == msginfo.MinLen) && ...
                (msginfo.MaxLen > 1);
        end        
    end

end
