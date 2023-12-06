classdef ConstraintFromSuccessor

    enumeration
        % Higher-priority vehicles do not consider lower-priority vehicles
        none
        % Higher-priority vehicles consider the occupied area of lower-priority
        % vehicles if they are in standstill
        area_of_standstill
        % Higher-priority vehicles consider the previous trajectory area of
        % lower-priority vehicles
        area_of_previous_trajectory
    end

end
