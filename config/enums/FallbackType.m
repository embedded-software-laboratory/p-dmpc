classdef FallbackType

    enumeration
        no_fallback
        local_fallback
        global_fallback
    end

    % no_fallback
    %   for disable fallback
    % local_fallback
    %   means once a vehicle triggers fallback,
    %   only vehicles with directed or undirected couplings must also take fallback
    % global_fallback
    %   means once a vehicle triggers fallback,
    %   all other vehicles must also take fallback
end
