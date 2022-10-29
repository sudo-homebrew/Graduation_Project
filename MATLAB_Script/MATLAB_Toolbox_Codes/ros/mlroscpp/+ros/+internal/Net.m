classdef Net
%This class is for internal use only. It may be removed in the future.

%Net Encapsulates network-related functionality

%   Copyright 2020-2021 The MathWorks, Inc.

    methods (Static)
        function validURI = canonicalizeURI(uri)
        %ros.internal.Net.canonicalizeURI Validate URI
        %   Validate and convert the given URI into its canonical representation.
        %   URIs in ROS are XML-RPC endpoints and should use either the http
        %   or https protocol.
        %   This function does not check connectivity to the given URI.
        %
        %   validURI = ros.internal.Net.canonicalizeURI(uri)
        %      Returns the canonicalized URI address, based on input 'uri'.
        %      This function will throw an error if the URI is invalid.

            validateattributes(uri, {'char'}, ...
                               {'nonempty'}, 'canonicalizeURI', 'uri');

            try
                uriObj = matlab.net.URI(uri);
                protocol = uriObj.Scheme;
                if ~any(strcmp(protocol, {'http', 'https'}))
                    error(message( ...
                        'ros:mlros:util:ProtocolNotSupported', ...
                        protocol, uri));
                end

                % Additional checks that are not supported by the MATLAB API

                % Make sure that host name is valid
                host = char(uriObj.Host);
                if isempty(host)
                    error(message(...
                        'ros:mlros:util:URIInvalid', ...
                        uri));
                end

                % Ensure that host name can be resolved
                % Construct a new URI with only necessary parts
                validHost = ros.internal.Net.canonicalizeHost(host);
                validURIObj = matlab.net.URI;
                validURIObj.Scheme = uriObj.Scheme;
                validURIObj.Host = validHost;
                validURIObj.Port = uriObj.Port;
                validURIObj.Path = uriObj.Path;

                % Ensure that port is set and non-negative
                if isempty(validURIObj.Port)
                    error(message('ros:mlros:util:PortUnspecified'));
                end
                if validURIObj.Port < 0
                    error(message('ros:mlros:util:PortNegative', ...
                                  validURIObj.Port));
                end

                % Can return a valid URI now
                validURI = char(validURIObj);
            catch ex
                switch ex.identifier
                  case {'ros:mlros:util:ProtocolNotSupported', ...
                        'ros:mlros:util:URIInvalid', ...
                        'ros:mlros:util:PortUnspecified', ...
                        'ros:mlros:util:PortNegative', ...
                        'ros:mlros:util:HostInvalid', ...
                        'ros:mlros:util:HostnameInvalid'}
                    rethrow(ex);
                  otherwise
                    newEx = ros.internal.ROSException(message('ros:mlros:util:URIInvalid', uri));
                    throw(newEx.addCustomCause(ex));
                end
            end
        end

        function validHost = canonicalizeHost(host)
        %ros.internal.Net.canonicalizeHost Validate a hostname or IP
        %   Validate the given hostname or IP address. If a hostname is
        %   specified it has to be resolvable by this computer,
        %   otherwise an error will be thrown.
        %
        %   Examples
        %   --------
        %   ros.internal.Net.canonicalizeHost('192.168.1.1')
        %      Returns '192.168.1.1'
        %
        %   ros.internal.Net.canonicalizeHost('ah-comp')
        %      Returns 'ah-comp'

            validateattributes(host, {'char'}, ...
                               {'nonempty'}, 'canonicalizeHost', 'host');

            % Verify that the host is valid
            % This catches problems with invalid characters
            if ~ros.internal.Net.isValidHost(host)
                error(message('ros:mlros:util:HostnameInvalid', host));
            end

            % Ensure that host name can be resolved to IP address
            try
                ros.internal.Net.getIPv4AddressByName(host);
            catch ex
                % Throw an exception if host name cannot be resolved to
                % an IP address
                newEx = ros.internal.ROSException(message( ...
                    'ros:mlros:util:HostInvalid', host));
                throw(newEx.addCustomCause(ex));
            end

            % Java implementation did not actually canonicalize either the
            % IP or the hostname, so return the input as-is if no error
            validHost = host;
        end

        function masterURI = generateMasterURI( varargin )
        %ros.internal.Net.generateMasterURI Determine the ROS Master URI
        %   This function takes optional host and port arguments, but also
        %   takes into account the ROS_MASTER_URI environment variable.
        %   There is a validation step that checks that the resulting URI
        %   is valid.
        %   It uses the following precedence to determine the ROS Master
        %   URI:
        %   (1) Given URI (in host argument)
        %   (2) Given hostname/IP address(host) and port
        %   (3) Given hostname and default port 11311
        %   (4) Environment variable ROS_MASTER_URI
        %   (5) If the global node is active, the Master URI it is
        %       connected to.
        %   (6) Default hostname 'localhost' and default port 11311
        %
        %   ros.internal.Net.generateMasterURI returns the
        %   default URI http://localhost:11311, or ROS_MASTER_URI, if specified.
        %
        %   ros.internal.Net.generateMasterURI(URI) validates
        %   the given Master URI and returns it if valid.
        %
        %   ros.internal.Net.generateMasterURI(HOST,PORT) uses
        %   the given host (hostname or IP address) and port to construct
        %   the Master URI.
        %
        %   ros.internal.Net.generateMasterURI(___,
        %   SUPPRESSOUTPUT) controls if any text outputs to the console
        %   should be suppressed during the retrieval of the Master
        %   URI. Default: false.

        % Default values for host, port, and output suppression
            defaults.host = 'localhost';
            defaults.suppressOutput = false;
            defaults.port = ros.internal.getDefaultCorePort;

            % Ignore empty inputs
            varargin = varargin(~cellfun(@isempty, varargin));

            % Parse inputs
            args = ros.internal.Net.parseArguments(defaults, varargin{:});

            % No host or URI arguments are specified.
            % Check items (4), (5), and (6) in precedence order
            if ~args.isHostSpecified
                % Check environment variable (precedence item (4))
                if ~isempty(getenv('ROS_MASTER_URI'))
                    masterURI = ros.internal.Net.validateROSMasterURI(args.suppressOutput);
                    return;
                end

                % Check master URI of global node (precedence item (5))
                if ros.internal.Global.isNodeActive
                    masterURI = ros.internal.Net.validateGlobalNodeMasterURI(args.suppressOutput);
                    return;
                end

                % Simply return the default value (precedence item (6))
                masterURI = ['http://',defaults.host,':',num2str(defaults.port)];
                return;
            end

            % Check first if host is a valid URI (precedence item (1))
            try
                masterURI = ros.internal.Net.canonicalizeURI(args.host);
                return;
            catch ex
                % Only rethrow exception if it is likely that user tried to
                % enter a URI directly
                if strncmp(args.host, 'http:', 5)
                    rethrow(ex);
                end
            end

            % Otherwise, build URI from host and port and validate
            % Precedence items (2) or (3) apply.
            validHost = ros.internal.Net.canonicalizeHost(args.host);
            uri = ['http://',validHost,':',num2str(args.port)];
            masterURI = ros.internal.Net.canonicalizeURI(uri);



        end

        function masterURI = generateMasterURIforNode( varargin )
        %ros.internal.Net.generateMasterURIforNode Determine the ROS Master URI
        %   This function takes optional host and port arguments, but also
        %   takes into account the ROS_MASTER_URI environment variable.
        %   There is a validation step that checks that the resulting URI
        %   is valid.
        %   It uses the following precedence to determine the ROS Master
        %   URI:
        %   (1) Given URI (in host argument)
        %   (2) Given hostname/IP address(host) and port
        %   (3) Given hostname and default port 11311
        %   (4) Environment variable ROS_MASTER_URI
        %   (5) If the global node is active, the Master URI it is
        %       connected to.
        %   (6) Default hostname 'localhost' and default port 11311.
        %   If ROS_DEFAULT_CORE_PORT is set to 0, ROS_DEFAULT_NODE_PORT will be used
        %
        %   ros.internal.Net.generateMasterURIforNode returns the
        %   default URI http://localhost:11311, or ROS_MASTER_URI, if specified.
        %
        %   ros.internal.Net.generateMasterURI(URI) validates
        %   the given Master URI and returns it if valid.
        %
        %   ros.internal.Net.generateMasterURIforNode(HOST,PORT) uses
        %   the given host (hostname or IP address) and port to construct
        %   the Master URI.
        %
        %   ros.internal.Net.generateMasterURIforNode(___,
        %   SUPPRESSOUTPUT) controls if any text outputs to the console
        %   should be suppressed during the retrieval of the Master
        %   URI. Default: false.

        % Default values for host, port, and output suppression
            defaults.host = 'localhost';
            defaults.suppressOutput = false;
            defaults.port = ros.internal.getDefaultNodePort;

            % Ignore empty inputs
            varargin = varargin(~cellfun(@isempty, varargin));

            % Parse inputs
            args = ros.internal.Net.parseArguments(defaults, varargin{:});

            % No host or URI arguments are specified.
            % Check items (4), (5), and (6) in precedence order
            if ~args.isHostSpecified
                % Check environment variable (precedence item (4))
                if ~isempty(getenv('ROS_MASTER_URI'))
                    masterURI = ros.internal.Net.validateROSMasterURI(args.suppressOutput);
                    return;
                end

                % Check master URI of global node (precedence item (5))
                if ros.internal.Global.isNodeActive
                    masterURI = ros.internal.Net.validateGlobalNodeMasterURI(args.suppressOutput);
                    return;
                end

                % Simply return the default value (precedence item (6))
                masterURI = ['http://',defaults.host,':',num2str(defaults.port)];
                return;
            end

            % Check first if host is a valid URI (precedence item (1))
            try
                masterURI = ros.internal.Net.canonicalizeURI(args.host);
                return;
            catch ex
                % Only rethrow exception if it is likely that user tried to
                % enter a URI directly
                if strncmp(args.host, 'http:', 5)
                    rethrow(ex);
                end
            end

            % Otherwise, build URI from host and port and validate
            % Precedence items (2) or (3) apply.
            validHost = ros.internal.Net.canonicalizeHost(args.host);
            uri = ['http://',validHost,':',num2str(args.port)];
            masterURI = ros.internal.Net.canonicalizeURI(uri);
        end

        function nodeHost = generateNodeHost( masterURI, varargin )
        %ros.internal.Net.generateNodeHost Generate the node host
        %   This could take the form of an IP address or a hostname from
        %   input and environment variables ROS_HOSTNAME and ROS_IP. A
        %   validation step is also part of this function.
        %   This function uses the following precedence to determine the
        %   node host:
        %   (1) Given hostname/IP address(host)
        %   (2) Environment variable ROS_HOSTNAME
        %   (3) Environment variable ROS_IP
        %   (4) The network interface address that is used to connect
        %       to the ROS Master.
        %   (5) The computer's hostname
        %   (6) localhost
        %
        %   ros.internal.Net.generateNodeHost(MASTERURI)
        %   returns the node host for the given ROS Master connection
        %   (based on the precedence order. If the ROS_HOSTNAME or
        %   ROS_IP environment variables are defined, their values will
        %   be used in the validation.
        %
        %   ros.internal.Net.generateNodeHost(MASTERURI, HOST) validates
        %   the given host name or IP address in HOST for suitability
        %   as a node host argument.
        %
        %   ros.internal.Net.generateNodeHost(___,
        %   SUPPRESSOUTPUT) controls if any text outputs to the console
        %   should be suppressed during the function execution.
        %   Default: false.

        % Parse inputs
            defaults.suppressOutput = false;
            defaults.host = '';
            args = parseArguments(defaults, masterURI, varargin{:});

            % If HOST input argument is not defined, evaluate environment
            % variables.
            if isempty(args.host)
                if ~isempty(getenv('ROS_HOSTNAME'))
                    % Precedence item (2)
                    args.host = ros.internal.Net.validateROSHostname(args.suppressOutput);
                elseif ~isempty(getenv('ROS_IP'))
                    % Precedence item (3)
                    args.host = ros.internal.Net.validateROSIP(args.suppressOutput);
                else
                    % Precedence item (4), (5), or (6)
                    args.host = ros.internal.Net.generateDefaultNodeHost(masterURI);
                end
            end

            % Final validation steps
            nodeHost = args.host;

            % Check if host can be resolved
            try
                inetAd = ros.internal.Net.getIPv4AddressByName(args.host);
            catch
                % Issue a warning
                warning(message('ros:mlros:util:NodeHostResolutionWarning', args.host));
                return;
            end

            inetStr = char(string(inetAd));
            if ~strcmp(inetStr, args.host)
                % A valid host name was specified, not an IP address
                return;
            end

            % If IP address was specified, make sure that it corresponds to
            % an existing network interface
            nodeIP = char(string(inetAd));
            intfInfo = ros.internal.Net.getAllIPv4Addresses;
            allIPs = {intfInfo.ipAddress};
            if ~ismember(nodeIP, allIPs)
                warning(message('ros:mlros:util:NodeHostIPWarning', nodeIP));
            end

            nodeHost = nodeIP;

            function args = parseArguments(defaults, varargin)
            %parseArguments Parse the arguments to the generateNodeHost function

                args = defaults;

                % Parse the suppress output argument (if it exists)
                if islogical(varargin{end})
                    % Function called with syntax generateNodeHost(__, suppressOutput)
                    args.suppressOutput = varargin{end};
                    validateattributes(args.suppressOutput, {'logical'}, {'nonempty'}, ...
                                       'generateNodeHost', 'suppressOutput');

                    % Remove this last argument to ease subsequent parsing
                    varargin(end) = [];
                end

                parser = inputParser;
                addRequired(parser, 'masterURI', @(x) validateattributes(x, ...
                                                                         {'char'}, {'nonempty'}, 'generateNodeHost', 'masterURI'));
                addOptional(parser, 'host', defaults.host, @(x) validateattributes(x, ...
                                                                                   {'char'}, {}, 'generateNodeHost', 'host'));
                parse(parser, varargin{:});
                args.masterURI = parser.Results.masterURI;
                args.host = parser.Results.host;
            end

        end

        function ip = validateROSIP(suppressOutput)
        %validateROSIP Validate the ROS_IP environment variable
        %   If it can be parsed without errors, it will be returned.

        % Be default, do not suppress any output
            if nargin == 0
                suppressOutput = false;
            end

            ip = getenv('ROS_IP');
            try
                ip = ros.internal.Net.canonicalizeHost(ip);
            catch ex
                newEx = ros.internal.ROSException(message(...
                    'ros:mlros:util:InvalidRosIPEnv', ip));
                throw(newEx.addCustomCause(ex));
            end
            if ~suppressOutput
                disp(getString(message('ros:mlros:util:UsingRosIPEnv', ip)));
            end
        end

        function hostName = validateROSHostname(suppressOutput)
        %validateROSHostname Validate the ROS_HOSTNAME environment variable
        %   If it can be parsed without errors, it will be returned.

        % Be default, do not suppress any output
            if nargin == 0
                suppressOutput = false;
            end

            hostName = getenv('ROS_HOSTNAME');
            try
                hostName = ros.internal.Net.canonicalizeHost(hostName);
            catch ex
                newEx = ros.internal.ROSException(message(...
                    'ros:mlros:util:InvalidRosHostnameEnv', hostName));
                throw(newEx.addCustomCause(ex));
            end
            if ~suppressOutput
                disp(getString(message('ros:mlros:util:UsingRosHostnameEnv', hostName)));
            end
        end

        function uri = validateROSMasterURI(suppressOutput)
        %validateROSMasterURI Validate the ROS_MASTER_URI environment variable
        %   If it can be parsed without errors, it will be returned.

        % Be default, do not suppress any output
            if nargin == 0
                suppressOutput = false;
            end

            uri = getenv('ROS_MASTER_URI');
            try
                uri = ros.internal.Net.canonicalizeURI(uri);
            catch ex
                newEx = ros.internal.ROSException(message(...
                    'ros:mlros:util:InvalidRosMasterURIEnv', uri));
                throw(newEx.addCustomCause(ex));
            end

            if ~suppressOutput
                disp(getString(message('ros:mlros:util:UsingRosMasterURIEnv', uri)));
            end
        end

        function masterURI = validateGlobalNodeMasterURI(suppressOutput)
        %validateGlobalNodeMasterURI Retrieve the Master URI from the
        %global node.

        % Be default, do not suppress any output
            if nargin == 0
                suppressOutput = false;
            end

            node = ros.internal.Global.getNodeHandle(false);
            masterURI = node.MasterURI;

            if ~suppressOutput
                disp(message('ros:mlros:node:UsingGlobalMasterURI', ...
                             masterURI).getString);
            end
        end

        function host = generateDefaultNodeHost( masterURI )
        %generateDefaultNodeHost Generate the default node host
        %   By default, this will return the host name of this
        %   computer. If the host name cannot be resolved to an IP
        %   address, return the IP address of the first non-local
        %   network interface instead.

        % Step 1: Try to find network interface address that is used to
        % connect to the ROS Master. Pick the first IPv4 address
        % registered with that host name.
            try
                % Try to find network interface address that is used to connect
                % to ROS Master.
                uri = matlab.net.URI(masterURI);
                masterHost = uri.Host;
                masterInet = ros.internal.Net.getIPv4AddressByName(masterHost);

                % Get the local address on the same subnet
                masterIP = char(string(masterInet));
                host = ros.internal.Net.getAssociatedInterfaceAddress(masterIP);
                return
            catch
                % Cannot retrieve IP address from Master host name or the
                % address is local
            end

            % Step 2: Use the host name of the computer (if it can be
            % resolved)
            try
                % Try to get local host name and validate it.
                % To stay consistent with the behavior of roscore in standard
                % ROS, we are using the local host name (and not the fully
                % qualified domain name (FQDN)).
                host = char(matlab.net.internal.InetAddress.getLocalHost.getHostName);
                assert(ros.internal.Net.isValidHost(host));
                return
            catch
                % Host name could not be resolved to an IP address
                % or contains invalid characters
            end

            % Step 3: Use IP address of first non-local network interface
            intfInfo = ros.internal.Net.getAllIPv4Addresses;
            if ~isempty(intfInfo)
                host = intfInfo(1).ipAddress;
                return
            end

            % Step 4: Give up and return 'localhost'
            host = 'localhost';
        end

        function validHost = isValidHost(host)
        %isValidHost Ensure that host name contains only legal characters
        %   Supporting host names according to RFC 2396 and RFC 2732

            validHost = false;

            try
                % Use MATLAB's URI object for very basic checking
                uri = matlab.net.URI(['http://' host ':999']);
                assert(~isempty(uri.Host));
                assert(~isempty(uri.Port));
            catch
                % Return while validHost is false
                return
            end

            % Host can be a name, IPv4 address, or IPv6 address
            % Name can only contain alphanumeric, dashes, and dots, needing
            % alphanumeric around any dashes and dots, and if there are any
            % dots, the last label must start with a letter
            % From original java.net.URI code comments:
            % hostname      = domainlabel [ "." ] | 1*( domainlabel "." ) toplabel [ "." ]
            % domainlabel   = alphanum | alphanum *( alphanum | "-" ) alphanum
            % toplabel      = alpha | alpha *( alphanum | "-" ) alphanum
            domainLabelPat = alphanumericsPattern + optionalPattern(asManyOfPattern(alphanumericsPattern | "-") + alphanumericsPattern);
            topLabelPat = lettersPattern + optionalPattern(asManyOfPattern(alphanumericsPattern | "-") + alphanumericsPattern);
            hostnamePat = (domainLabelPat + optionalPattern(".")) | (asManyOfPattern(domainLabelPat + ".", 1) + topLabelPat + optionalPattern("."));
            if matches(host, hostnamePat)
                validHost = true;
                return
            end

            % IPv4 can only contain four sets of digits separated by dots
            % Each set of digits can have no more than three
            % Each set must fit in a byte (255 is max value), which will
            % need further verification
            bytePat = digitsPattern(1, 3);
            IPv4Pat = bytePat + "." + bytePat + "." + bytePat + "." + bytePat;
            if matches(host, IPv4Pat) && ...
                    all(255 >= str2double(extract(host, bytePat)))
                validHost = true;
                return
            end

            % IPv6 is 16 bytes of address - typically brackets surrounding
            % eight hexadecimal sections separated by colons
            % Sections of only zeros may be compressed with double-colons
            % An IPv4 address may substitute for the last 2 sections
            % From the original java.net.URI code comments:
            % IPv6address = hexseq [ ":" IPv4address ]
            %              | hexseq [ "::" [ hexpost ] ]
            %              | "::" [ hexpost ]
            % hexpost     = hexseq | hexseq ":" IPv4address | IPv4address
            % hexseq      = hex4 *( ":" hex4)
            % hex4        = 1*4HEXDIG
            % Additionally we constrain the IPv6 address as follows:
            % i.  IPv6 addresses without compressed zeros should contain
            %     exactly 16 bytes.
            % ii. IPv6 addresses with compressed zeros should contain
            %     less than 16 bytes.
            hexDigPat = characterListPattern("0123456789ABCDEFabcdef");
            hex4Pat = asManyOfPattern(hexDigPat, 1, 4);
            hexSeqPat = hex4Pat + asManyOfPattern(":" + hex4Pat);
            hexPostPat = hexSeqPat | ...
                optionalPattern(hexSeqPat + ":") + IPv4Pat;
            IPv6Pat = ("[" + hexSeqPat + optionalPattern(hexSeqPat + ":") + IPv4Pat + "]") | ...
                      ("[" + hexSeqPat + optionalPattern("::" + optionalPattern(hexPostPat)) + "]") | ...
                      ("[::" + optionalPattern(hexPostPat) + "]");
            if matches(host, IPv6Pat)
                IPv4Portion = extract(host, IPv4Pat);
                if ~isempty(IPv4Portion)
                    % Check for valid IPv4 address (4 bytes total)
                    if ~all(255 >= str2double(extract(IPv4Portion, bytePat)))
                        return  % While validHost is false
                    end
                    nBytes = 4;
                    IPv6Portion = extractBefore(host, IPv4Portion);
                else
                    nBytes = 0;
                    IPv6Portion = host;
                end
                % Get number of bytes in IPv6 portion (2 bytes per hex4)
                nBytes = nBytes + 2*numel(extract(IPv6Portion, hex4Pat));
                if contains(host, '::')
                    % If zero-compression is used must be less than 16 bytes
                    validHost = nBytes < 16;
                else
                    % Otherwise must be exactly 16 bytes
                    validHost = nBytes == 16;
                end
            end
        end

        function inSubnet = isAddressInSubnet(ipv4Address, intfAddress, subnetLength)
        %isAddressInSubnet Determine if a given address is in a subnet
        %
        %   INSUBNET = isAddressInSubnet(IPV4ADDRESS, INTFADDRESS,
        %   SUBNETLENGTH) determines if IPV4ADDRESS is in the subnet
        %   that is defined by the network interface address
        %   INTFADDRESS and its associated number of subnet bits
        %   SUBNETLENGTH.
        %
        %   Example:
        %       % This will return 'true'
        %       inSubnet = isAddressInSubnet('192.168.70.154',
        %          '192.168.70.1', 24)

        % Convert subnet length into actual address
        % in form of uint8 array, for bitwise operations
        % Loop over values in hex format for easy fixed length
            subnetHexChar = dec2hex(intmax('uint32') - (2^(32-subnetLength)-1));
            subnetMask = ones(1, 4, 'uint8');
            for k = 1:4
                subnetMask(k) = uint8(hex2dec(subnetHexChar(2*k-1:2*k)));
            end

            % Determine if addresses in same subnet by comparing the
            % "bitwise and" operation of each IP on the subnet mask
            ipObj = matlab.net.internal.InetAddress(ipv4Address);
            intfObj = matlab.net.internal.InetAddress(intfAddress);
            ip = ipObj.toBytes;
            ipNetwork = bitand(ip, subnetMask);
            intfNetwork = bitand(intfObj.toBytes, subnetMask);
            inSubnet = isequal(ipNetwork, intfNetwork);

            % Also check that the provided IP does not match the "network"
            % or "broadcast" (low and high ends of the subnet) to match the
            % org.apache.commons.net.util.SubnetUtils implementation
            if inSubnet
                network = ipNetwork;  % Same operation
                broadcast = bitor(ip, bitcmp(subnetMask));
                inSubnet = ~isequal(ip, network) && ~isequal(ip, broadcast);
            end
        end

        function intfAddr = getAssociatedInterfaceAddress(ipv4Address)
        %getAssociatedInterfaceAddress Retrieve interface address
        %   The network interface's IP address that is used to resolve
        %   the input address IPV4ADDRESS is returned by this function.
        %   It can be used as a default for the 'NodeHost' setting.
        %   If the network interface address cannot be determined,
        %   an error will be thrown.

        % Get all network interface addresses
            intfInfoAll = ros.internal.Net.getAllIPv4Addresses;

            % Check for which one the input address is in the subnet
            inSubnet = arrayfun(@(x) ros.internal.Net.isAddressInSubnet(ipv4Address, ...
                                                                        x.ipAddress, x.subnetLength), intfInfoAll);

            if ~any(inSubnet)
                % Address cannot be resolved through any network
                % interface. The default gateway will be used, but Java has
                % no way of determining what that is.
                error(message('ros:mlros:util:NotInAnySubnet', ipv4Address));
            end

            if sum(inSubnet) > 1
                % Address can be resolved through multiple network
                % interfaces. Check network configuration.
                error(message('ros:mlros:util:InMultipleSubnets', ipv4Address));
            end

            % Retrieve the one unique network interface that resolves the
            % address
            intfInfo = intfInfoAll(inSubnet);
            intfAddr = intfInfo.ipAddress;
        end

        function intfInfo = getAllIPv4Addresses(resolveHostName)
        %getAllIPAddresses Return all IPv4 addresses for this host
        %   This function uses information from the network interface,
        %   so it is not dependent on a correct host resolution.
        %   Only non-local addresses are returned, since local ones would not
        %   be reachable by other nodes in the ROS network.
        %
        %   INTFINFO = getAllIPv4Addresses returns information about all
        %   N IPv4 addresses that are assigned to local network
        %   interfaces. INTFINFO is a structure array of size Nx1 with
        %   the following elements:
        %   - ipAddress - IPv4 address (char)
        %   - hostName - Will be empty ''. A reverse host name lookup
        %     is not performed.
        %   - interfaceName - Name of the associated network interface
        %   (char)
        %   - subnetLength - The length of the subnet mask (double)
        %
        %   INTFINFO = getAllIPv4Addresses(true) also performs a
        %   reverse host name lookup and populates the corresponding
        %   field in INTFINFO:
        %   - hostName - Associated hostname, if any (char)

            if nargin == 0
                % By default, do not resolve IP addresses to host names
                resolveHostName = false;
            end

            % All address information will be returned in a structure array
            intfInfo = struct.empty(0,1);

            try
                interfaces = matlab.net.internal.NetworkInterface.list;
                for iIntf = 1:numel(interfaces)
                    % Check each network interface
                    intf = interfaces(iIntf);
                    intfAddr = intf.InterfaceAddresses;
                    for iAddr = 1:numel(intfAddr)
                        % Check each associated IP address
                        addr = intfAddr(iAddr);
                        ip = addr.InetAddress;

                        % Don't take local addresses
                        if ip.IsLoopback
                            continue;
                        end

                        % Only look at IPv4 addresses
                        if ~isequal(ip.Version, uint16(4))
                            continue;
                        end

                        endIdx = numel(intfInfo)+1;
                        intfInfo(endIdx,1).ipAddress = char(ip.string);

                        % Only do reverse DNS host name lookup if specified
                        % by caller. This can be a slow operation.
                        if resolveHostName
                            intfInfo(endIdx,1).hostName = char(ip.getHostName);
                        else
                            intfInfo(endIdx,1).hostName = '';
                        end

                        intfInfo(endIdx,1).interfaceName = [char(intf.Description), ...
                                                            ' (' char(intf.FriendlyName) ')'];

                        % Check for invalid subnets and choose a
                        % default value of 24 bits
                        subnetLength = double(addr.NetworkPrefixLength);
                        if subnetLength < 0
                            subnetLength = 24;
                        end
                        intfInfo(endIdx,1).subnetLength = subnetLength;
                    end
                end
            catch ex
                newEx = ros.internal.ROSException(message('ros:mlros:util:NetworkInterfaceError'));
                throw(newEx.addCustomCause(ex));
            end

        end

    end
    methods(Static, Access = private)
        function inetObj = getIPv4AddressByName(hostName)
        %getIPv4AddressByName Return the first IPv4 address with the given hostname
        %   Convenience function to enable replacement of Java net
        %   utilities with MATLAB, since Java returns IPv4 address but
        %   MATLAB net utilities returns IPv6 by default. Search through
        %   all addresses that match a given name and return the IP of the
        %   first IPv4 address.
        %
        %   If the provided host name has no IPv4 addresses or does not
        %   match any interface, this function will error.

        % List of all addresses corresponding to host
            addresses = matlab.net.internal.InetAddress.getAllByName(hostName);

            % Pick first IPv4 address
            addressesV4 = addresses([addresses.Version] == 4);
            if isempty(addressesV4)
                error(message('ros:mlros:util:HostInvalid', hostName));
            end
            inetObj = addressesV4(1);
        end

        function args = parseArguments(defaults, varargin)
        %parseArguments Parse the arguments to the
        %   generateMasterURI and the generateMasterURIforNode
        %   functions

            args = defaults;
            args.isHostSpecified = false;

            if isempty(varargin)
                % Function called with syntax generateMasterURI()
                return;
            end

            % Parse the suppress output argument (if it exists)
            if islogical(varargin{end})
                % Function called with syntax generateMasterURI(__, suppressOutput)
                args.suppressOutput = varargin{end};
                validateattributes(args.suppressOutput, {'logical'}, {'nonempty'}, ...
                                   'generateMasterURI', 'suppressOutput');

                % Remove this last argument to ease subsequent parsing
                varargin(end) = [];
            end

            % Check if there are any other arguments
            if isempty(varargin)
                % Function called with syntax generateMasterURI(suppressOutput)
                return;
            end

            % Otherwise, host, port, or URI was specified. Parse the
            % remaining arguments.
            args.isHostSpecified = true;

            parser = inputParser;
            addOptional(parser, 'host', defaults.host, @(x) validateattributes(x, ...
                                                                               {'char'}, {'nonempty'}, 'generateMasterURI', 'host'));
            addOptional(parser, 'port', defaults.port, @(x) validateattributes(x, ...
                                                                               {'numeric'}, {'scalar','nonempty','integer','nonnegative'}, ...
                                                                               'generateMasterURI', 'port'));

            parse(parser, varargin{:});
            args.host = parser.Results.host;
            args.port = parser.Results.port;
        end
    end
end
