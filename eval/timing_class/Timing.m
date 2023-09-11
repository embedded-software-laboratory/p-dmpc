classdef Timing < Singleton
    %   Template written by Bobby Nedelkovski
    %   The MathWorks Australia Pty Ltd
    %   Copyright 2009, The MathWorks, Inc.

    properties % Public Access
        timers %TODO restrict type
        myData (1, 1)
        name (1, 1)
    end

    methods (Access = private)
        % Guard the constructor against external invocation.  We only want
        % to allow a single instance of this class.  See description in
        % Singleton superclass.
        function newObj = Timing()
            % Initialise your custom properties.
            newObj.myData = 1;
        end

    end

    methods (Static)

        function obj = instance()
            persistent uniqueInstance

            if isempty(uniqueInstance)
                obj = Timing();
                obj.myData = 42;
                uniqueInstance = obj;
            else
                obj = uniqueInstance;
            end

        end

        function create_timer(name)
            % Just assign the input value to singletonData.  See Singleton
            % superclass.
            obj = Timing.instance();
            obj.name = name;
            obj.myData = 13;
            obj.set_singleton_data(obj);
            fprintf("create_timer called with name %s \n", name);
        end

        function create_and_start_timer(name)
            % create and start timer
            fprintf("create_and_start_timer called with name %s \n", name);
        end

        function time = stop_timer(name)
            obj = Timing.instance().get_singleton_data();
            fprintf("stop_timer called with name %s \n", name);
            fprintf("have obj.name=%s, obj.myData=%d \n", obj.name, obj.myData);
            time = 0;
        end

    end

end
