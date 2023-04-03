classdef ParameterRequester
    %PARAMETERREQUESTER Gives access to the Lab Control Center's parameters
    %   Uses ParameterRequests to access the ParameterServer.
    %   Especially useful to get active_vehicle_ids and middleware_period
    %   Since this uses it's own participant and is probably only needed
    %   during setup, the created object should be deleted after use.

    properties (Access = private)
        % Because we use just one reader and one writer in here,
        % we can get away with using just the participant and adding
        % our subscribers/publishers/readers/writers to this object.
        % Using participant.write/participant.take then uses the last
        % added reader/writer, which always is the correct one in this
        % case.
        participant
    end

    methods

        function obj = ParameterRequester()
            %PARAMETERREQUESTER Construct an instance of this class
            %   Detailed explanation goes here

            % Hard-coded ~/dev/software paths for now

            % Import IDL files from cpm library
            dds_idl_matlab = fullfile('~/dev/software/cpm_lib/dds_idl_matlab/');
            assert(isfolder(dds_idl_matlab), ...
                'Missing directory "%s".', dds_idl_matlab);
            assert(~isempty(dir([dds_idl_matlab, '*.m'])), ...
                'No MATLAB IDL-files found in %s', dds_idl_matlab);
            addpath(dds_idl_matlab)

            % Read LCC's domain ID
            % This parameter should be set when we get called from the LCC;
            % if started manually alongside the LCC, we default to 0
            dds_domain = getenv("DDS_DOMAIN");
            % Check if environment variable returned something
            if any(size(dds_domain) == 0)
                default_dds_domain = 21;
                fprintf("ParameterRequester: DDS_DOMAIN environment variable not set; defaulting to %i\n", ...
                    default_dds_domain);
                dds_domain = default_dds_domain;
            else
                dds_domain = str2double(dds_domain); % Convert from char array to int
            end

            obj.participant = DDS.DomainParticipant('default', dds_domain);

            % This creates and adds a default subscriber/publisher
            obj.participant.addSubscriber();
            obj.participant.addPublisher();

            % Using these DDS types assumes that the MATLAB init_script.m
            % was already run. Otherwise these have to manually be added to
            % the NDDS_QOS_PROFILES environment variable
            parameter_request_topic = 'parameterRequest';
            parameter_request_type = 'ParameterRequest';

            obj.participant.addWriter(parameter_request_type, parameter_request_topic);

            parameter_topic = 'parameter';
            parameter_type = 'Parameter';

            obj.participant.addReader(parameter_type, parameter_topic);
        end

        function parameter = requestParameter(obj, parameter_name)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            request = ParameterRequest();
            request.name = convertStringsToChars(parameter_name);

            % Potentially wait for an answer here?
            % This would be a lot more efficient with a waitset and a
            % DDS filter for this specific parameter name.
            % However, it only gets called during setup, so performance is
            % not too important.
            parameter_received = false;
            start_time = tic;

            while ~parameter_received

                wait_time = toc(start_time);

                if wait_time > 5
                    fprintf("ParameterRequester: Waiting for parameter %s for %f seconds now; did you click deploy in the LCC before starting the HLC?\n", parameter_name, wait_time);
                end

                obj.participant.write(request);
                pause(1);
                [samples, ~, ~, ~] = obj.participant.take();

                for sample = samples

                    if strcmp(sample.name, parameter_name)
                        parameter_received = true;
                        parameter = sample;
                    end

                end

            end

        end

    end

end
