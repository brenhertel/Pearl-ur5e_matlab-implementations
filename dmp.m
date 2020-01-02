function new_pos = dmp(pos, t, initial, goal, given_alpha_z, given_tau, given_num_basis)
global tau alpha_z alpha_x beta_z num_basis;
tau = given_tau;
alpha_z = given_alpha_z;
beta_z = alpha_z/4;
alpha_x = 1;
num_basis = given_num_basis;
% vel = gradient(pos(:)) ./ gradient(t(:));
% %acc = perform_dmp(pos, vel, alpha_z, tau, num_basis, initial, goal);
% acc = gradient(vel(:)) ./ gradient(t(:));
% figure;
% new_pos = zeros(1, length(pos));
% new_vel = zeros(1, length(pos));
% new_pos(1) = initial;
% for i = 2:length(pos)
%     %not sure if this is the correct way to integrate the returned
%     %acceleration
%     new_vel(i) = trapz(acc(1:i), t(1:i));
% end
% for i = 2:length(pos)
%     new_pos(i) = trapz(new_vel(1:i), t(1:i)) + initial;
% end
% figure;
% plot(t, pos, 'k');
% hold on;
% plot(t, new_pos, 'r--');
% hold off;
end

%%% dmp deformation of 1d dataset %%%
%tau*y.. = alpha_z(beta_z(g-y)-y.)+f
%tau - time constant
%alpha_z, beta_z - positive constants (beta_z = alpha_z/4)
%g - goal
%f - forcing term
%y - position
%y. - velocity
%y.. - acceleration

function acc = perform_dmp(pos, vel, alpha_z, tau, num_basis, initial, goal)
%beta_z = 2*sqrt(alpha_z);
beta_z = alpha_z/4;
f = get_forcing_term(pos, tau, num_basis, initial, goal);
acc = zeros(1, length(pos));
for i = 1:length(pos)
    acc(i) = alpha_z*(beta_z*(goal-pos(i))-vel(i))+f(i);
end
acc = acc/tau;
end

%this equation gets improved later, don't use this version
%f(t) = (sum(1=1:n)(psi_i(t)*w_i))/(sum(i=1:n)(psi_i(t)))
%psi_i - fixed basis functions
%w_i - weights of those functions

%tau*x. = -alpha_x*x
%alpha_x - constant
%x - goes from 1 to 0
%DE solution: x = e^-(alpha_x*t/tau)

%although the final value of x isn't exactly 0, it's far more than close
%enough
function x = canonical_system()
global alpha_x tau cs_runtime dt;
dt = .01;
cs_runtime = 1;
x = ones(1, cs_runtime/dt);
for i = 1:length(x)
    x(i) = exp(-(alpha_x*i)/tau);
end
end

%f(x) = (sum(1=1:n)(psi_i(x)*w_i))/(sum(i=1:n)(psi_i(x)))*(x*(g-y_0))
%psi_i - fixed basis functions
%psi_i(x) = exp(-((x-c_i)^2)/(2*(sigma_i^2)))
%sigma_i - constant, width of basis function
%c_i - constant, center of basis function
%y_0 = initial state y(t=0)

function f = get_forcing_term(pos, initial, goal)
global num_basis;
%get canonical system
x = canonical_system();
%preallocate space
c = get_centers();
sigma = ones(1, num_basis);
sigma = sigma * num_basis / c;
psi = zeros(num_basis, length(pos));
f = zeros(1, length(pos));
numerator = ones(1, num_basis);
denominator = ones(1, num_basis);
%assume all weights are 1 for now?
%w = ones(1, length(pos));
%random weights??
w = zeros(1, num_basis);
%create the basis functions
for i = 1:num_basis
    %assume these constants for now
    for j = 1:length(pos)
        psi(i, j) = exp(-(power((x(j)-c(i)), 2))/(2*power(sigma(i), 2)));
    end
end
for i = 1:length(pos)
    for j = 1:num_basis
        numerator(j) = psi(j)*w(j);
        denominator(j) = psi(j);
    end
    f(i) = (sum(numerator)/sum(denominator))*x(i)*(goal-initial);
end
end

function c = get_centers()
global alpha_x num_basis cs_runtime;
des_c = linspace(0, cs_runtime, num_basis);
c = ones(length(des_c));
for i=1:length(des_c)
    c(i) = exp(-alpha_x*des_c(i));
end
end