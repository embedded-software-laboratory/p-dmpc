yaws = [0.5, 1];
for i = 1:length(yaws)
    [value, index] = min(abs(situation_costs.angle - yaws(i)));
    yaws(i) = situation_costs.angle(index);
end