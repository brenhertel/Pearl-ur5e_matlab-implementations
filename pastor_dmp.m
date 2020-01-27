classdef pastor_dmp < handle
    properties
        goal; %goal poisition
        y_0; %initial position
        c; %height of basis function
        h; %width of basis function
        % psi; %basis functions
        n_bfs; %number of basis functions
        dt; %timestep value
        w; %weights of basis function
        D; %constant
        K; %constant
        cs = CanonicalSystem; %canonical system
        timesteps; %number of timesteps
        y; %position
        dy; %velocity
        ddy; %acceleration
        tau; %temporal scaling
        f;
    end
    methods
        function generate_DMP(obj, num_basis, D, K, cs_alpha, dt, tau)
            % for finding best params %
            obj.n_bfs = num_basis;
            obj.D = D;
            obj.K = K;
            obj.dt = dt;
            obj.cs.change_params(cs_alpha, dt)
            obj.tau = tau;
            %
            obj.w = ones(1, obj.n_bfs) - 0.99;
            obj.timesteps = round(obj.cs.run_time / obj.dt);
            obj.reset_state();
            %setup center and variance of basis functions
            obj.gen_centers();
            %
            obj.h = ones(1, obj.n_bfs);
            obj.h = obj.h * power(obj.n_bfs, 1.5) / obj.c / obj.cs.alpha_x; %guess?
            %
            %obj.check_offset();
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
            
            %use the pos, vel, acc to get forcing term
            x_track = obj.cs.rollout;
            for i = 1:length(x_track)
                f_target(i) = (((obj.tau * ddy_des(i)) + (obj.D * dy_des(i))) / obj.K)...
                    - (obj.goal - y_des(i)) + ((obj.goal - obj.y_0) * x_track(i));
            end
            
            %Use generated forcing term to get weights
            obj.find_weights(f_target);
            obj.reset_state();
        end
        function y_track = rollout(obj)
            %Create a new rollout using the already generated weights
            obj.reset_state();
            n_timesteps = round(obj.timesteps / obj.tau);
            y_track = zeros(1, n_timesteps);
            dy_track = zeros(1, n_timesteps);
            ddy_track = zeros(1, n_timesteps);
            %disp(obj.f);
            for t = 1: n_timesteps
                [y_track(t), dy_track(t), ddy_track(t)] = obj.step(t);
            end
        end
        function [pos, vel, acc] = step(obj, num_step)
            s = obj.cs.step(obj.tau);
            %generate psi?
            %psi = obj.gen_psi(x);
            %disp(obj.f(num_step));
            obj.ddy = (obj.K * (obj.goal - obj.y)) - (obj.D * obj.dy) - (obj.K * (obj.goal - obj.y_0) * s) + (obj.K * obj.f(num_step));
            
            %Integrate to get velocity and acceleration
            obj.dy = obj.dy + obj.ddy * obj.tau * obj.dt;
            obj.y = obj.y + obj.dy * obj.dt;
            pos = obj.y;
            vel = obj.dy;
            acc = obj.ddy;
        end
        function gen_centers(obj)
            des_c = linspace(0, obj.cs.run_time, obj.n_bfs);
            obj.c = ones(1, length(des_c));
            %
            for i = 1:length(des_c)
                obj.c(i) = exp(-obj.cs.alpha_x * des_c(i)); %guess?
            end
            %
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
        function gen_f(obj, x_track, psi_track)
            obj.f = zeros(1, length(x_track));
            for s = 1:length(x_track)
                num = 0;
                denom = 0;
                for i = 1:length(obj.n_bfs)
                    num = num + (obj.w(i) * psi_track(i, s) * x_track(s));
                    denom = denom + psi_track(i, s);
                end
                %fprintf('Numerator: %f, Denominator: %f\n', num, denom);
                obj.f(s) = num / denom;
            end
        end
        function find_weights(obj, f_target)
            x_track = obj.cs.rollout;
            psi_track = obj.gen_psi(x_track);
            obj.gen_f(x_track, psi_track);
            base_J = 0;
            for s = 1:length(x_track)
                base_J = base_J + power(f_target(s) - obj.f(s), 2);
            end
            prev_J = base_J;
            %             J = prev_J - 1;
            %             while J < prev_J
            %                 old_w = obj.w;
            %                 obj.w = obj.w + 1000;
            %                 obj.gen_f(x_track, psi_track);
            %                 J = 0;
            %                 for s = 1:length(x_track)
            %                     J = J + power(f_target(s) - obj.f(s), 2);
            %                 end
            %             end
            %             obj.w = old_w;
            %             prev_J = J;
            %             J = prev_J - 1;
            %             while J < prev_J
            %                 old_w = obj.w;
            %                 obj.w = obj.w + 100;
            %                 obj.gen_f(x_track, psi_track);
            %                 J = 0;
            %                 for s = 1:length(x_track)
            %                     J = J + power(f_target(s) - obj.f(s), 2);
            %                 end
            %             end
            %             obj.w = old_w;
            %             prev_J = J;
            for j = 1:2
                for i = 1:obj.n_bfs
                    fprintf('PrevJ: %f\n', prev_J);
                    J = prev_J - 1;
                    while J < prev_J
                        old_w = obj.w(i);
                        obj.w(i) = obj.w(i) * 1.1;
                        obj.gen_f(x_track, psi_track);
                        J = 0;
                        for s = 1:length(x_track)
                            J = J + power(f_target(s) - obj.f(s), 2);
                        end
                        fprintf('J: %f, i: %f, w(i): %f\n', J, i, obj.w(i));
                        prev_J = J;
                    end
                    obj.w(i) = old_w;
                end
            end
            disp(obj.w);
        end
    end
end
