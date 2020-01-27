%%% draw in matlab %%%
clear all;
global x y index;
index = 1;
figure;
xlim([0 1]);
ylim([0 1]);
hold on;
a = 0;
b = 0;
while (a == 0 && b == 0)
    [a, b] = ginput(1);
    c = a;
    d = b;
end
set(gcf, 'WindowButtonMotionFcn', @mouseMove);


function mouseMove(object, eventdata)
global x y index;
C = get(gca, 'CurrentPoint');
title(gca, ['(x, y) = (' num2str(C(1, 1)) ', ' num2str(C(1, 2)) ')']);
plot(C(1,1), C(1, 2), 'k.');
x(index) = C(1, 1);
y(index) = C(1, 2);
index = index + 1;
end