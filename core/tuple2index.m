function entry = tuple2index(trim)
%TUPLE2INDEX Calculate index in trim list according to trim tuple

    dim = length(trim);

    entry = 0;

    for i = 1:dim

        entry = entry + trim(i) * 10 ^ i;

    end

end



