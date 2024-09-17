function result = rwth_color_order(color_index)

    arguments
        color_index (1, :) uint8 = [1:11]
    end

    result = [ ...
                  0 84 159 %RWTH blau     RGB
              246 168 0 %RWTH orange   RGB
              0 152 161 %RWTH türkis   RGB
              122 111 172 %RWTH Lila     RGB
              204 7 30 %RWTH rot      RGB
              0 97 101 %RWTH petrol   RGB
              161 16 53 %RWTH bordeaux RGB
              87 171 39 %RWTH grün     RGB
              97 33 88 %RWTH violett  RGB
              255 237 0 %RWTH gelb     RGB
              189 205 0 %RWTH maigrün  RGB
              ];
    result = result ./ 255;
    result = result(mod(color_index - 1, 11) + 1, :);
end
