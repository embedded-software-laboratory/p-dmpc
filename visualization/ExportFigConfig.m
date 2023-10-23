classdef ExportFigConfig

    properties
        fontsize = 8;
        units = 'centimeters';
        paperwidth = 8; % figure width
        paperheight = 4; % figure height
        linewidth = 0.5;
        fontname = 'Times';
        markersize = 3;
    end

    methods

        function obj = ExportFigConfig()
        end

    end

    methods (Static)

        function obj = paper(options)

            arguments
                options.fontsize
                options.units
                options.paperwidth
                options.paperheight
                options.linewidth
                options.fontname
                options.markersize
            end

            obj = ExportFigConfig();
            obj.fontsize = 8;
            obj.units = 'centimeters';
            obj.paperwidth = 8;
            obj.paperheight = 4;
            obj.linewidth = 0.5;
            obj.fontname = 'Times';
            obj = ExportFigConfig.set_optional_properties(obj, options);
        end

        function obj = document(options)

            arguments
                options.fontsize
                options.units
                options.paperwidth
                options.paperheight
                options.linewidth
                options.fontname
                options.markersize
            end

            obj = ExportFigConfig();
            obj.fontsize = 9;
            obj.units = 'centimeters';
            obj.paperwidth = 15.7;
            obj.paperheight = 7.85;
            obj.linewidth = 0.5;
            obj.fontname = 'Times';
            obj = ExportFigConfig.set_optional_properties(obj, options);
        end

        function obj = presentation(options)

            arguments
                options.fontsize
                options.units
                options.paperwidth
                options.paperheight
                options.linewidth
                options.fontname
                options.markersize
            end

            obj = ExportFigConfig();
            obj.fontsize = 18;
            obj.units = 'centimeters';
            obj.paperwidth = 31.77;
            obj.paperheight = 14.01;
            obj.linewidth = 1;
            obj.fontname = 'Arial';
            obj = ExportFigConfig.set_optional_properties(obj, options);
        end

        function obj = video(options)

            arguments
                options.fontsize
                options.units
                options.paperwidth
                options.paperheight
                options.linewidth
                options.fontname
                options.markersize
            end

            obj = ExportFigConfig();
            obj.fontsize = 20;
            obj.paperwidth = 1920;
            obj.paperheight = 1080;
            obj.linewidth = 1;
            obj.fontname = 'Arial';
            obj.units = 'pixels';
            obj = ExportFigConfig.set_optional_properties(obj, options);
        end

        function obj = set_optional_properties(obj, options)

            arguments
                obj
                options
            end

            fn = fieldnames(options);

            if isempty(fn)
                return
            end

            for field = fn

                if ~isempty(options.(field{1}))
                    obj.(field{1}) = options.(field{1});
                end

            end

        end

    end

end
