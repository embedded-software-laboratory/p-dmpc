classdef (Abstract) Cutter
    % CUTTER    Abstract class for parallelizing couplings

    methods (Abstract)
        cut(obj, M, max_num_CLs)
    end

    methods (Static, Access = public)

        function cutter = get_cutter(strategy)
            % GET_CUTTER  Given a cutting strategy this function returns the corresponding cutter.

            switch strategy
                case CutStrategies.greedy_cut
                    cutter = GreedyCutter();
            end

        end

    end

end
