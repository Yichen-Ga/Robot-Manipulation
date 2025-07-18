classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962
        goal; % the last or current destination position
    end
    
    methods
        
        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(packet)
	    %Close the device
            packet.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function packet = Robot(dev)
             packet.myHIDSimplePacketComs=dev; 
            packet.pol = java.lang.Boolean(false);
        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(packet, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function com = read(packet, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	packet.myHIDSimplePacketComs.readFloats(intid);
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function  write(packet, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds,packet.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(packet, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(packet.GRIPPER_ID);
                packet.myHIDSimplePacketComs.writeBytes(intid, ds, packet.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(packet)
            packet.writeGripper(180);
        end
        
        % Closes the gripper
        function closeGripper(packet)
            packet.writeGripper(0);
        end
        
        % Which takes a 1x3 array of joint values in degrees to be sent directly to 
        % the actuators and bypasses interpolation
        function servo_jp(self,v)
            SERV_ID = 1848; % ID of the actuators
            
            packet = zeros(15, 1, 'single'); % create an empty packet to be sent to actuators
            packet(1) = 0; % bypasses interpolation
            packet(2) = 0; %linear interpolation, which does not matter in this function
            packet(3) = v(1); % First link
            packet(4) = v(2); % Second link 
            packet(5) = v(3); % Third link 
            
            self.goal = v; % record the final position
            self.write(SERV_ID, packet); % send the joint values directly to the actuators
        end
        

        % Which takes a 1x3 array of joint values and an interpolation time
        % in ms to get there, then send them to actuators directly.
        function interpolate_jp(self,v, t)
            SERV_ID = 1848; % ID of the actuators
            
            packet = zeros(15, 1, 'single'); % create an empty packet to be sent to actuators
            packet(1) = t; % assign interpolation time
            packet(2) = 0; %linear interpolation.
            packet(3) = v(1); % First link
            packet(4) = v(2); % Second link 
            packet(5) = v(3); % Third link 
            
            self.goal = v; % record the final position
            self.write(SERV_ID, packet); % send the joint values directly to the actuators
        end
        
        % Which takes two boolean values, named GETPOS and GETVEL. 
        % Only return the results for the requested data, and set the rest to zero.
        % returns a 2x3 array thatcontains current joint positions in degrees (1st row)
        % and/or current joint velocities (2ndrow).
        function pos_vel = measure_js (self, GETPOS, GETVEL)
            pos_vel = []; %set up the final returning matrix
           
            if GETPOS && ~GETVEL %return only position vector
                SERV_ID = 1910; %ID of the sensors
                curr_pos = self.read(SERV_ID);
                pos = [curr_pos(3), curr_pos(5), curr_pos(7)];
                pos_vel (1,:) = pos;
                pos_vel (2,:) = [0, 0, 0];
                %replace the first row of matrix to be position vector
            end
            if GETVEL && ~GETPOS
                SERV_ID = 1822; %ID of the sensors
                curr_vel = self.read(SERV_ID);
                vel = [curr_vel(3), curr_vel(6), curr_vel(9)];
                pos_vel (1,:) = [0, 0, 0];
                pos_vel (2,:) = vel; %replace the second row of matrix to be velocity vector
            end 
            if GETVEL && GETPOS
                curr_pos = self.read(1910);
                pos = [curr_pos(3), curr_pos(5), curr_pos(7)];
                curr_vel = self.read(1822);
                vel = [curr_vel(3), curr_vel(6), curr_vel(9)];
                pos_vel (1,:) = pos;
                pos_vel (2,:) = vel;
            end    
        end
       % ==============================================================
        %Which returns a 1x3 array that contains current joint set point positions 
        %in degrees. If interpolation is being used and you request this during motion,
        %it  will  return  the  current intermediate set point.
        function curr_set_point = setpoint_js(self)
            SERV_ID = 1910; %ID of the sensors
            
            curr_pos = self.read(SERV_ID); % Get Positions and Setpoint
            curr_set_point = [curr_pos(3), curr_pos(5), curr_pos(7)]; % create setpoint array
        end
        
        % Which returns a 1x3 array thatcontains the end-of-motion joint 
        % setpoint positions in degrees. 
        function eom = goal_js(self)
            eom = self.goal; % return the previous stored end of motion setpoint vector
        end
      
        %This method takes in a 1x4 array corresponding to a row of the DH parameter table for a given link.
        %It then generates the associated intermediate transformation and returns a corresponding symbolic
        %4x4 homogeneous transformation matrix.
        function HT_m = dh2mat(self, v)
            % Apply DH parameters in the 4x4 homogeneous transformation matrix.
            HT_m = [cosd(v(1)) -sind(v(1))*cosd(v(4)) sind(v(1))*sind(v(4)) v(3)*cosd(v(1)); sind(v(1)) cosd(v(1))*cosd(v(4)) -cosd(v(1))*sind(v(4)) v(3)*sind(v(1)); 0 sind(v(4)) cosd(v(4)) v(2); 0 0 0 1];
        end
        
        function HT_m = dh2mat_sym(self, v)
            % Apply DH parameters in the 4x4 homogeneous transformation matrix.
            HT_m = [cos(v(1)) -sin(v(1))*cos(v(4)) sin(v(1))*sin(v(4)) v(3)*cos(v(1)); sin(v(1)) cos(v(1))*cos(v(4)) -cos(v(1))*sin(v(4)) v(3)*sin(v(1)); 0 sin(v(4)) cos(v(4)) v(2); 0 0 0 1];
        end
        % Which takes in an nx4 array corresponding to the n rows of  the  
        % full  DH  parameter  table.  
        % It  then  generates  a  corresponding  symbolic  4x4 
        % homogeneous  transformation  matrix  for  the  composite  transformation.
        function comp_trans_matrix = dh2fk(self, v)
            % count the number of rows of v.
            num_rows = size(v,1);
            % set the original transformational matrix as a 4x4 identical
            % matrix.
            final_trans = eye(4,4);
            for n = 1:num_rows
                %the current transformation matrix on n rows
                curr_trans = self.dh2mat(v(n,:));
                %multiply the current transfomation matrix by the previous
                %cumulative transformation matrix.
                final_trans = final_trans * curr_trans;
            end
            comp_trans_matrix = final_trans;
        end
        
        
        %This method takes n joint configurations as inputs in the form of 
        %an nx1 vector. It should return a 4x4 homogeneous transformation 
        %matrix representing the position and orientation of the 
        %tip frame with respect to the base frame
        function final_trans_matrix = fk3001(self,v)
            L0 = 55; %mm
            L1 = 40; %mm
            L2 = 100; %mm
            L3 = 100; %mm
            %create empty matrix for DH table
            dh_table = zeros(4,4);
            %assign the first row of DH table.
            dh_table(1,:) = [0,L0,0,0];
            %assign the second row of DH table using 
            %the first row of joint configuration matrix.
            dh_table(2,:) = [v(1),L1,0,-90];
            %assign the third row of DH table using 
            %the second row of joint configuration matrix.
            dh_table(3,:) = [v(2)-90,0,L2,0];
            %assign the fourth row of DH table using 
            %the third row of joint configuration matrix.
            dh_table(4,:) = [v(3)+90,0,L3,0];
            %use dh2fk function to calculate the transformation matrix.
            final_trans_matrix = self.dh2fk(dh_table);
        end
            
        %Which takes data from measured_js() and returns a 4x4 homogeneous transformation matrix
        %based upon the current joint positions in degrees.
        function HT_m = measured_cp(self)
            %get the current joint positions
            cur_pos = self.measure_js(1, 0);
            %generate 4x4 homogeneous transformation matrix based upon the
            %current joint positions in degrees.
            HT_m = self.fk3001(cur_pos(1,:));
        end
        
        %Which takes data from setpoint_js()and returns a 4x4 homogeneous transformation matrix
        %based upon the current joint set point positions in degrees. Note that if interpolation
        %is being used and you request this during motion it will return the current intermediate
        %set point. 
        function HT_m = setpoint_cp(self)
            %get the current setpoint
            cur_setpoint = self.setpoint_js();
            %generate 4x4 homogeneous transformation matrix based upon the
            %current setpoint in degrees.
            HT_m = self.fk3001(cur_setpoint);
        end
        
        %Which  takes  data  from goal_js()and returns a 4x4 homogeneous transformation matrix 
        %based upon the commanded end of motion joint set point positions in degrees.
        function HT_m = goal_cp(self)
            %get the final position
            final_pos = self.goal_js();
            %generate 4x4 homogeneous transformation matrix based upon the
            %final position in degrees.
            HT_m = self.fk3001(final_pos);
        end
        
        % This method takes a 3x1 task space position vector as the input
        % and returns a set of corresponding
        % joint angles (i.e. q1, q2, q3) that would make the robot’s
        % end-effector move to that target position with elbow up path. The
        % position vectors are in mm, joint angles are in degrees.
        function q_v = ik3001(self,p_v)
            x = p_v(1);
            y = p_v(2);
            z = p_v(3);
            %through the error if the task space is unreachable.
            if (p_v(1)>=200 | p_v(2)>=200 | p_v(3)>=295)
                error('end effector position is out of workspace')
            end
             % Calculated formular.
            %D_q1 = x/sqrt(x^2+y^2);
            D_alpha = (x^2+y^2+(z-95)^2)/(200*sqrt(x^2+y^2+(z-95)^2)); 
            D_q3 =(20000-x^2-y^2-(z-95)^2)/20000;
            % calculate each joint angle using atan2.
            q1 = atan2d(p_v(2),p_v(1));
            %q1 = atan2d(sqrt(1-(D_q1)^2), D_q1);
            alpha = atan2d(sqrt(1-(D_alpha)^2), D_alpha);
            beta = atan2d((z-95),sqrt(x^2 + y^2));
            q2 = 90 - (alpha + beta);
            q3 = 90 - atan2d(sqrt(1-(D_q3^2)), D_q3);
            %In case we need it, here is the elbow down path.
%             q1 = atan2(-sqrt(1-D_q1^2), D_q1);
%             q2 = atan2(-sqrt(1-D_q2^2), D_q2);
%             q3 = atan2(-sqrt(1-D_q3^2), D_q3);
            q_v = [q1; q2; q3];
        end
        
        %This method takes your configuration q, and returns the
        %corresponding numeric 6 by 3 Jacobian matrix.
        function jacob = jacob3001(self,q)
            %assigne three joint variable
            theta1 = q(1)/180*pi;
            theta2 = q(2)/180*pi;
            theta3 = q(3)/180*pi;
            
            %calculate upper half jacobion matrix and assgine lower half.
            %Then combine them to get the full jacobian matrix.
jp1 = [100*(sin(theta1))*(sin(theta2))*(sin(theta3))-100*(sin(theta1))*(cos(theta2))*(cos(theta3))+100*(sin(theta1))*(sin(theta2));
       -100*(cos(theta1))*(sin(theta2))*(sin(theta3))+100*(cos(theta1))*(cos(theta2))*(cos(theta3))-100*(cos(theta1))*(sin(theta2));
       0];
   
jp2 = [-100*cos(theta1)*cos(theta2)*sin(theta3)-100*cos(theta1)*sin(theta2)*cos(theta3)-100*cos(theta1)*cos(theta2);
       -100*sin(theta1)*cos(theta2)*sin(theta3)-100*sin(theta1)*sin(theta2)*cos(theta3)-100*sin(theta1)*cos(theta2);
       100*cos(theta2)*cos(theta3)-100*sin(theta2)*sin(theta3)-100*sin(theta2)];
   
jp3 = [-100*cos(theta1)*sin(theta2)*cos(theta3)-100*cos(theta1)*cos(theta2)*sin(theta3);
       -100*sin(theta1)*sin(theta2)*cos(theta3)-100*sin(theta1)*cos(theta2)*sin(theta3);
       -100*sin(theta2)*sin(theta3)+100*cos(theta2)*cos(theta3)];
   
jo1 = [0;0;1];

jo2 = [-sin(theta1);cos(theta1);0];

jo3 = [-sin(theta1);cos(theta1);0];

            jacob = [jp1 jp2 jp3; jo1 jo2 jo3]; 
        end
       
        %This method takes your configuration q, and the vector of
        %instantaneous joint velocities inputs. It should return the
        %6x1 vector including the task-space linear velocities ̇and
        %angular velocities.
        function p_vel = fdk3001(self, q, q_v)
            %calculate jacobian
            jacob = self.jacob3001(q);
            %calculate position linear velocities
            p_vel = jacob * q_v';
        end
        
        
        %The numerical inverse kinematics algorithm. you should determine
        %where the robot is, determine the desired target (input), and gradually
        %move the stick model of the arm in the direction pointing from the
        %current position of the end-effector to the final position of the end-effector.
        function curr_joint = ik_3001_numerical(self, target)
            curr_joint = [0,0,0];
            curr_pos = [0,0,0];
            while (sqrt((target(1)-curr_pos(1))^2 + (target(3)-curr_pos(3))^2) > 5)
                curr_pos_matrix = self.fk3001(curr_joint);
                curr_pos = [curr_pos_matrix(1,4),curr_pos_matrix(2,4),curr_pos_matrix(3,4)]
                pos_vector = (target-curr_pos)./norm(target-curr_pos);
                pos_vector = pos_vector * 500
                Jacob = self.jacob3001(curr_joint)
                Upper_Jacob = [Jacob(1:3,1),Jacob(1:3,2),Jacob(1:3,3)];
                inv_Jacob = inv(Upper_Jacob)
                joint_velocity = inv_Jacob*(pos_vector')
                t = 0.01;
                joint_increment = t*joint_velocity
                curr_joint = curr_joint - joint_increment
            end
        end
       end
end
