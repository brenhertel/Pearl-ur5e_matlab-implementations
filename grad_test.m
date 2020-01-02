x = 1:100;
y = power(x, 2);
slope = gradient(y(:)) ./ gradient(x(:));
for i=1:length(x)
    new_y(i) = trapz(slope(1:i), x(1:i));
end
figure;
plot(x, y, 'k');
hold on;
plot(x, new_y, 'r--');
hold on;