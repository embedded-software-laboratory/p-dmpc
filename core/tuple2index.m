function entry = tuple2index(trim)
%TUPLE2INDEX Calculate index in trim list according to trim tuple

    dim = length(trim);

    entry = trim(1);
    
    if dim == 1
        
        return;
        
    end

    for i = 2:dim

        entry = entry + (trim(i)-1) * 10^(i-1);

    end

end
