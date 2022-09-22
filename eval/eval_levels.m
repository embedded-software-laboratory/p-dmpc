function [ res ] = eval_levels(res)
    [ nVeh, nPri, nSce ] = size(res);
    nLevels_by_veh_pri = cell(nVeh, nPri);
    nLevels_by_pri = cell(nPri);
    for iVeh = 1:nVeh
        for iPri = 1:nPri
            for iSce = 1:nSce
                result = res{iVeh,iPri,iSce};
                nSteps = result.nSteps-1;
                result.nLevels = max(result.priority);
                res{iVeh,iPri,iSce} = result;
                nLevels_by_veh_pri{iVeh,iPri} = [nLevels_by_veh_pri{iVeh,iPri} result.nLevels];
                nLevels_by_pri{iPri} = [nLevels_by_pri{iPri} result.nLevels];
            end
        end
    end
    histogram(nLevels_by_pri{1})
end

