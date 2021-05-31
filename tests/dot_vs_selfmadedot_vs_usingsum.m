n = 1e7;

c1 = [1;2;3;4;5]*10;
c2 = [5;4;3;2;1]*10;

for i=1:n
    a1 = dot(c1,c2);
end
for i=1:n
    a2 = c1'*c2;
end
for i=1:n
    a3 = sum(c1.*c2);
end

a1
a2
a3