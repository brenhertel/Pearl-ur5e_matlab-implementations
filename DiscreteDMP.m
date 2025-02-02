classdef DiscreteDMP < handle
    properties
        goal; %goal poisition
        y_0; %initial position
        c; %height of basis function
        h; %width of basis function
        % psi; %basis functions
        n_bfs; %number of basis functions
        dt = 0.001; %timestep value
        w; %weights of basis function
        alpha_y = 25; %assumed default constant
        beta_y; %constant
        cs = CanonicalSystem; %canonical system
        timesteps; %number of timesteps
        y; %position
        dy; %velocity
        ddy; %acceleration
    end
    methods
        function generate_DMP(obj, num_basis)
            obj.cs.change_params(obj.alpha_y, .0001);
            obj.n_bfs = num_basis;
            %obj.y_0 = given_y0;
            %obj.goal = given_goal;
            obj.w = zeros(1, obj.n_bfs);
            obj.beta_y = obj.alpha_y / 4; %critical damping
            %obj.beta_y = sqrt(obj.alpha_y) / 2;
            obj.timesteps = round(obj.cs.run_time / obj.dt);
            obj.reset_state();
            %setup center and variance of basis functions
            obj.gen_centers();
            %dcps(ID).psi = exp(-0.5*((dcps(ID).x-dcps(ID).c).^2).*dcps(ID).D);
            obj.h = ones(1, obj.n_bfs);
            obj.h = (obj.h * power(obj.n_bfs, 1.5)) / obj.c / obj.cs.alpha_x; %guess?
            %obj.h       = (diff(obj.c)*0.55).^2;
            %obj.h       = 1./[obj.h ;obj.h(end)];
            obj.check_offset();
        end
        function reset_state(obj)
            %set path back to start
            obj.y = obj.y_0;
            obj.dy = 0;
            obj.ddy = 0;
            obj.cs.reset_state();
        end
        function check_offset(obj)
            %Make sure goal != initial otherwise path won't generate
            if obj.y_0 == obj.goal
                obj.goal = obj.goal + 1e-4;
            end
        end
        function path = imitate_path(obj, y_des)
            %Imitate given path to generate weights for basis functions
            obj.reset_state();
            obj.y_0 = y_des(1);
            obj.goal = obj.gen_goal(y_des);
            obj.check_offset();
            path = zeros(1, obj.timesteps);
            x = linspace(0, obj.cs.run_time, length(y_des));
            %Use spline interpolation of path
            path_gen = interp1(x, y_des, x, 'spline');
            for t = 1:obj.timesteps
                path(t) = path_gen(t) * obj.dt;
            end
            y_des = path;
            %Use derivitaves to find velocity and acceleration, starting at
            %0
            dy_des = diff(y_des) / obj.dt;
            zero_arr = zeros(1);
            dy_des = horzcat(zero_arr, dy_des);
            ddy_des = diff(dy_des) / obj.dt;
            ddy_des = horzcat(zero_arr, ddy_des);
            %f_target = zeros(1, length(y_des));
            %use the pos, vel, acc to get forcing term
            f_target = (ddy_des - obj.alpha_y * (obj.beta_y * (obj.goal - y_des) - dy_des));
            %Use generated forcing term to get weights
            obj.gen_weights(f_target);
            obj.reset_state();
        end
        function y_track = rollout(obj, given_tau)
            %Create a new rollout using the already generated weights
            obj.reset_state();
            if nargin == 2
                tau = given_tau;
            else
                tau = 1;
            end
            n_timesteps = round(obj.timesteps / tau);
            y_track = zeros(1, n_timesteps);
            dy_track = zeros(1, n_timesteps);
            ddy_track = zeros(1, n_timesteps);
            for t = 1: n_timesteps
                [y_track(t), dy_track(t), ddy_track(t)] = obj.step(tau);
            end
        end
        function [pos, vel, acc] = step(obj, tau)
            x = obj.cs.step(tau);
            %generate psi
            psi = obj.gen_psi(x);
            %Get forcing term for reproduction
            f = obj.gen_front_term(x) * (dot(psi, obj.w) / sum(psi));
            %Use the forcing term to get acceleration
            obj.ddy = (obj.alpha_y * (obj.beta_y * (obj.goal - obj.y) - (obj.dy / tau)) + f) * tau;
            %Integrate to get velocity and acceleration
            obj.dy = obj.dy + obj.ddy * tau * obj.dt;
            obj.y = obj.y + obj.dy * obj.dt;
            pos = obj.y;
            vel = obj.dy;
            acc = obj.ddy;
        end
        function gen_centers(obj)
            des_c = linspace(0, obj.cs.run_time, obj.n_bfs);
            obj.c = ones(1, length(des_c));
            for i = 1:length(des_c)
                obj.c(i) = exp(-obj.cs.alpha_x * des_c(i)); %guess?
            end
        end
        function front_term = gen_front_term(obj, x)
            front_term = x * (obj.goal - obj.y_0);
        end
        function goal = gen_goal(~, y_des)
            goal = y_des(length(y_des));
        end
        function psi = gen_psi(obj, x)
            for i = 1:length(obj.c)
                for j = 1:length(x)
                    %generate basis functions using following formula
                    psi(i, j) = exp(-obj.h * power(x(j) - obj.c(i), 2));
                end
            end
        end
        function gen_weights(obj, f_target)
            %Generate weights for the given forcing term
            x_track = obj.cs.rollout;
            psi_track = obj.gen_psi(x_track);
            k = obj.goal - obj.y_0;
            for i = 1:obj.n_bfs
                numer = sum(x_track * (psi_track(i,:))' * f_target);
                denom = sum(power(x_track, 2) * (psi_track(i,:))');
                obj.w(i) = numer / (k * denom);
            end
            %Covnert any NaN's to 0's
            obj.w(isnan(obj.w)) = 0;
            %disp(obj.w);
        end
    end
end
