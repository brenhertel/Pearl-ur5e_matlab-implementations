x = ones(1, length(pos_x_data));
n = 2;
while(n <= length(pos_x_data) && x(n) >= 0)
    x(n) = x(n-1) - (1/(length(pos_x_data)-1));
    n = n + 1;
end
x = x/tau;