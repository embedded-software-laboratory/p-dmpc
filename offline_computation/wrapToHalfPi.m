function out = wrapToHalfPi(lambda)
    tmp = mod(lambda+pi/2,pi);
    out = abs(tmp+pi*(lambda>0&tmp==0)-pi/2);
end