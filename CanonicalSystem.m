%%% Canonical System %%%

classdef CanonicalSystem < handle
    properties
        alpha_x = 1; %maybe change to 5?
        dt = .05
        run_time = 1
        x = 1;
    end
    methods
        function change_params(obj, varargin)
            if (length(varargin) > 0)
                obj.alpha_x = varargin{1};
            end
            if (length(varargin) > 1)
                obj.dt = varargin{2};
            end
        end
        function reset_state(obj)
            obj.x = 1;
        end
        function out_x = step(obj, tau)
            obj.x = obj.x + ((-obj.alpha_x * obj.x) / tau)  * obj.dt;
            out_x = obj.x;
        end
        function x_track = rollout(obj, tau)
            if nargin == 2
                n_timesteps = round((obj.run_time / obj.dt) / tau);
            else
                n_timesteps = round(obj.run_time / obj.dt);
            end
            x_track = zeros(1, n_timesteps);
            for i = 1: n_timesteps
                x_track(i) = obj.x;
                if nargin == 2
                    obj.step(tau);
                else
                    obj.step(1);
                end
            end
            obj.reset_state();
        end
    end
end
