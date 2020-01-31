%%% matlab draw lines %%%
clear all;
figure;
index = 1;
xlim([0 1]);
ylim([0 1]);
hold on;
a_last = 0;
b_last = 0;
[a, b] = ginput(1);
while a_last ~= a || b_last ~= b
    fprintf('a: %f, b: %f\n', a, b);
    plot(a, b, 'k.');
    hold on;
    x(index) = a;
    y(index) = b;
    index = index + 1;
    a_last = a;
    b_last = b;
    [a, b] = ginput(1);
end
n = round(1000 / (length(x) - 1));
x_new = [];
y_new = [];
for i = 1:length(x) - 1
    x_new = horzcat(x_new, linspace(x(i), x(i+1), n));
    y_new = horzcat(y_new, linspace(y(i), y(i+1), n));
end