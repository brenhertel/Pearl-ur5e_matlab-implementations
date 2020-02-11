function out = constrain(in, lower, upper)
if (in < lower)
    out = lower;
elseif (in > upper)
    out = upper;
else
    out = in;
end
end