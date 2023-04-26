function result = rwth_color_order
    result = [ ...
                  0 84 159 %RWTH blau     RGB
              246 168 0 %RWTH orange   RGB
              0 152 161 %RWTH türkis   RGB
              122 111 172 %RWTH Lila     RGB
              204 7 30 %RWTH rot      RGB
              189 205 0 %RWTH maigrün  RGB
              0 97 101 %RWTH petrol   RGB
              161 16 53 %RWTH bordeaux RGB
              87 171 39 %RWTH grün     RGB
              97 33 88 %RWTH violett  RGB
              255 237 0 %RWTH gelb     RGB
              ];
    result = result ./ 255;
end
