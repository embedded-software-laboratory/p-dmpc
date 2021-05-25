function [ subcontroller_runtime ] = get_subcontroller_runtime(info)

    if isstruct(info)
        info = {info};
    end

    % Array Of Structs
    info_aos = [info{:}];
    
    subcontroller_runtime = [info_aos(:).subcontroller_runtime]';
    
end
