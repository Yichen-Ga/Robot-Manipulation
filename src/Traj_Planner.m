classdef Traj_Planner
    %TRAJ_PLANNER Summary of this class goes here
    %  This class is used to plan the trajectory of our robot.
    
    properties
        via_points;
    end
    
    methods
        function obj = Traj_Planner(via_points)
            %TRAJ_PLANNER Construct an instance of this class
            %   The construtor takes some via_points as
            %   input. via_points are n by 3 array. Each row  of  the
            %   array  represents  the  x, y,  and z coordinates of  a
            %   specific  position of  the  end-effector, and n is the
            %   number of via-points on the trajectory. 
            obj.via_points = via_points;
        end
        
        %solves  for  a  cubic  (3rd  order) polynomial trajectory between
        %two via-points. It should take in desired starting and ending
        %times t0 and tf(in seconds), starting and ending velocities, and
        %starting and ending positions. It should outputa 4-by-1 array
        %containing the coefficients of polynomial.
        function a = cubic_traj(self, t0, tf, v0, vf, p0, pf)
            % Set up the function
            F = [p0 v0 pf vf]';
            M = [1 t0 t0^2 t0^3; 0 1 2*t0 3*t0^2; 1 tf tf^2 tf^3; 0 1 2*tf 3*tf^2];
            %return the coefficients of polynomial
            a = inv(M)*F;
        end
        
        %helper function that use the output trajectory coefficient from
        %cubic_traj to calculate the current postion at current time.
        function q = cubic_traj_app(self, v, t)
            q = v(1) + v(2)*t + v(3)*t^2 + v(4)*t^3;
        end
        
        %solves for a quintic polynomial trajectory(5thorder) between two
        %via-points. It should take in desired starting and ending times
        %t0and tf(in seconds), starting and ending velocities, and starting
        %and ending positions. It should output a 6-by-1 array containing
        %the coefficients of the polynimial
        function a = quintic_traj(self, t0, tf, v0, vf, p0, pf, alpha0, alphaf)
            % Set up the function
            F = [p0 v0 alpha0 pf vf alphaf]';
            M = [1 t0 t0^2 t0^3 t0^4 t0^5; 0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 0 0 2 6*t0 12*t0 20*t0; 1 tf tf^2 tf^3 tf^4 tf^5; 0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; 0 0 2 6*tf 12*tf 20*tf];
            
            %return the coefficients of polynomial
            a = inv(M)*F;
        end
        
        %helper function that use the output trajectory coefficient from
        %cubic_traj to calculate the current postion at current time.
        function q = quintic_traj_app(self, v, t)
            q = v(1) + v(2)*t + v(3)*t^2 + v(4)*t^3 + v(5)*t^4 + v(6)*t^5;
        end
        
        %takes  in  start  and  ending positions, velocity, acceleration, and time
        %of the end-effector and  outputs  planned  trajectory  in  the  task  space.
        %There is a boolean variable to choose for using either cubic
        %(True) or quantic (False) trajectory. It should  generate
        %interpolated points along the trajectory in task space as your via-points. 
        function a = linear_traj(self, traj_mod, start_p, end_p, start_v, end_v, start_a, end_a, start_t, end_t)
            %Set up the trajectory function depending on the mode selected.
            if traj_mod
                ax = self.cubic_traj(start_t(1), end_t(1), start_v(1), end_v(1), start_p(1), end_p(1));
                ay = self.cubic_traj(start_t(2), end_t(2), start_v(2), end_v(2), start_p(2), end_p(2));
                az = self.cubic_traj(start_t(3), end_t(3), start_v(3), end_v(3), start_p(3), end_p(3));
            else
                ax = self.quintic_traj(start_t(1), end_t(1), start_v(1), end_v(1), start_p(1), end_p(1), start_a(1), end_a(1));
                ay = self.quintic_traj(start_t(2), end_t(2), start_v(2), end_v(2), start_p(2), end_p(2), start_a(2), end_a(2));
                az = self.quintic_traj(start_t(3), end_t(3), start_v(3), end_v(3), start_p(3), end_p(3), start_a(3), end_a(3));
            end
            a = [ax ay az];
        end       
        
    end
end

