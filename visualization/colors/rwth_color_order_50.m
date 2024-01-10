function result = rwth_color_order_50(color_index)

    arguments
        color_index (1, :) uint8 = [1:11]
    end

    result = [ ...
                  142 186 229 %RWTH blau RGB
              253 212 143 %RWTH orange RGB
              137 204 207 %RWTH türkis RGB
              188 181 215 %RWTH Lila RGB
              230 150 121 %RWTH rot RGB
              224 230 154 %RWTH maigrün RGB
              125 164 167 %RWTH petrol RGB
              205 139 135 %RWTH bordeaux RGB
              184 214 152 %RWTH grün RGB
              168 133 158 %RWTH violett
              255 245 155 %RWTH gelb RGB
              ];
    result = result ./ 255;
    result = result(color_index, :);
end
