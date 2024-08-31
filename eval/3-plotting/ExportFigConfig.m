classdef ExportFigConfig

    properties
        fontsize = 8;
        units = 'centimeters';
        paperwidth = 8; % figure width
        paperheight = 4; % figure height
        linewidth = 0.5;
        linewidth_timing_boxes = 5; % boxes in runtime plots (e.g., plot_runtime_for_step) are drawn as thick lines
        fontname = 'Times';
        markersize = 3;
    end

    methods

        function obj = ExportFigConfig()
        end

    end

    methods (Static)

        function obj = paper(optional)

            arguments
                optional.fontsize
                optional.units
                optional.paperwidth
                optional.paperheight
                optional.linewidth
                optional.linewidth_timing_boxes
                optional.fontname
                optional.markersize
            end

            obj = ExportFigConfig();
            obj.fontsize = 8;
            obj.units = 'centimeters';
            obj.paperwidth = 8;
            obj.paperheight = 4;
            obj.linewidth = 0.5;
            obj.linewidth_timing_boxes = 5;
            obj.fontname = 'Times';
            obj = ExportFigConfig.set_optional_properties(obj, optional);
        end

        function obj = document(optional)

            arguments
                optional.fontsize
                optional.units
                optional.paperwidth
                optional.paperheight
                optional.linewidth
                optional.linewidth_timing_boxes
                optional.fontname
                optional.markersize
            end

            obj = ExportFigConfig();
            obj.fontsize = 9;
            obj.units = 'centimeters';
            obj.paperwidth = 15.7;
            obj.paperheight = 7.85;
            obj.linewidth = 0.5;
            obj.linewidth_timing_boxes = 5;
            obj.fontname = 'Times';
            obj = ExportFigConfig.set_optional_properties(obj, optional);
        end

        function obj = presentation(optional)

            arguments
                optional.fontsize
                optional.units
                optional.paperwidth
                optional.paperheight
                optional.linewidth
                optional.linewidth_timing_boxes
                optional.fontname
                optional.markersize
            end

            obj = ExportFigConfig();
            obj.fontsize = 18;
            obj.units = 'centimeters';
            obj.paperwidth = 31.77;
            obj.paperheight = 14.01;
            obj.linewidth = 1;
            obj.linewidth_timing_boxes = 10;
            obj.fontname = 'Arial';
            obj = ExportFigConfig.set_optional_properties(obj, optional);
        end

        function obj = video(optional)

            arguments
                optional.fontsize
                optional.units
                optional.paperwidth
                optional.paperheight
                optional.linewidth
                optional.linewidth_timing_boxes
                optional.fontname
                optional.markersize
            end

            obj = ExportFigConfig();
            obj.fontsize = 14;
            obj.paperwidth = 1920;
            obj.paperheight = 1080;
            obj.linewidth = 1;
            obj.linewidth_timing_boxes = 10;
            obj.fontname = 'Arial';
            obj.units = 'pixels';
            obj = ExportFigConfig.set_optional_properties(obj, optional);
        end

        function obj = set_optional_properties(obj, optional)

            arguments
                obj
                optional
            end


            for field = fieldnames(optional)'

                if ~isempty(optional.(field{1}))
                    obj.(field{1}) = optional.(field{1});
                end

            end

        end

    end

end
