%%
% RBE3001 - Laboratory 4
% 

clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs); 
m = Model(pp);
traj5 = Traj_Planner(pp);

%%
% % %plan trajectory
% %     %find corresponding joint value:
% %     point_1 = pp.ik3001([51,-26,169]); %  -27.0127 -24.3843 34.2185
% %     point_2 = pp.ik3001([-2,2,273]); %  135.0000 -26.2023 -35.7747
% %     point_3 = pp.ik3001([74,-18,230]);%  -13.6713 -9.7663 -11.6101
% %     point_4 = [0,0,-90];
% %     curr_position = [];
% 
%     %movement 1
%         t = 0;
%         %calculate the unit vector between the current position to target position
%         u_v_1=([-2,2,273]-[51,-26,169])./norm([-2,2,273]-[51,-26,169]);
%         %set a arbitrary_velocity
%         arbitrary_v = 0.4;
%         %3x1 task space instantaneous velocity vector
%         instant_v_1 = arbitrary_v*u_v_1;
%         ee_1_joint1 = traj5.cubic_traj(0,4,instant_v_1(1),instant_v_1(1),-27.0127,135.0000);
%         ee_1_joint2 = traj5.cubic_traj(0,4,instant_v_1(2),instant_v_1(2),-24.3843,-26.2023);
%         ee_1_joint3 = traj5.cubic_traj(0,4,instant_v_1(3),instant_v_1(3),34.2185,-35.7747);
%         
% %     %movement 2
%         %calculate the unit vector between the current position to target position
%         u_v_2=([74,-18,230]-[-2,2,273])./norm([74,-18,230]-[-2,2,273]);
%         %set a arbitrary_velocity
%         arbitrary_v = 0.4;
%         %3x1 task space instantaneous velocity vector
%         instant_v_2 = arbitrary_v*u_v_2;
%         ee_2_joint1 = traj5.cubic_traj(0,4,instant_v_2(1),instant_v_2(1),135.0000,-13.6713);
%         ee_2_joint2 = traj5.cubic_traj(0,4,instant_v_2(2),instant_v_2(2),-26.2023,-9.7663);
%         ee_2_joint3 = traj5.cubic_traj(0,4,instant_v_2(3),instant_v_2(3),-35.7747,-11.6101);
%         
% %     %movement 3
%         %calculate the unit vector between the current position to target position
%         u_v_3=([51,-26,169]-[74,-18,230])./norm([51,-26,169]-[74,-18,230]);
%         %set a arbitrary_velocity
%         arbitrary_v = 0.4;
%         %3x1 task space instantaneous velocity vector
%         instant_v_3 = arbitrary_v*u_v_3;
%         ee_3_joint1 = traj5.cubic_traj(0,4,instant_v_3(1),instant_v_3(1),-13.6713,-27.0127);
%         ee_3_joint2 = traj5.cubic_traj(0,4,instant_v_3(2),instant_v_3(2),-9.7663,-24.3843);
%         ee_3_joint3 = traj5.cubic_traj(0,4,instant_v_3(3),instant_v_3(3),-11.6101,34.2185);
% 
% %assign empty matrix to the matrix that will
% %change through the loop
%        tip_position_data = [];
%        toc_data= [];
%        pos_data = [];
%        curr_joint_angle = [];
%        curr_all_angle = [];
%        t0_all = [];
%        v0_all = [];
%        p0_all = [];
%        tf_all = [];
%        vf_all = [];
%        pf_all = [];
%        all_q = [];
%        curr_position = [];
%        inv_jacob_data = [];
%     %make the robot arm move to three point
%     pp.servo_jp([-27.0127, -24.3843, 34.2185]);
%     pause(3);
%     tic
%     while toc < 12.5
%        %each time the ii is different and it assign different position
%        %value
%        if (toc <= 4)
%            t = toc;
%            position = [traj5.cubic_traj_app(ee_1_joint1,t) traj5.cubic_traj_app(ee_1_joint2,t) traj5.cubic_traj_app(ee_1_joint3,t)];
%        end
%        if (4 < toc)&&(toc <= 8)
%            t = toc - 4;
%            position = [traj5.cubic_traj_app(ee_2_joint1,t) traj5.cubic_traj_app(ee_2_joint2,t) traj5.cubic_traj_app(ee_2_joint3,t)];
%        end
%        if (8 < toc)&&(toc <= 12)
%            t = toc - 8;
%            position = [traj5.cubic_traj_app(ee_3_joint1,t) traj5.cubic_traj_app(ee_3_joint2,t) traj5.cubic_traj_app(ee_3_joint3,t)];
%        end
%        curr_position = [curr_position; position];
%        %make the robot arm move to v1 angle
%        pp.servo_jp(position);
%        %store the current time as curr_toc
%        cur_toc = toc;
%        %use the overall toc time minus the current time, when it's larger
%        %than 1, exit the while loop.
%        %measure the current joint position
%         curr_pos_vel = pp.measure_js (1, 1);
%         %store the current joint angle and joint velocity;
%         new_pos = curr_pos_vel (1, :);
%         new_vel = curr_pos_vel (2, :);
%         % Store the joint angles in an nx3 array.
%         pos_data = [pos_data; new_pos];
%         %generate the current transformation matrix
%         new_trans_matrix = pp.fk3001(new_pos);
%         % Store the current tip position in an 1x3 array.
%         curr_tip_position = [new_trans_matrix(1,4),new_trans_matrix(2,4),new_trans_matrix(3,4)];
%         %accumulate all tip position data
%         tip_position_data = [tip_position_data; curr_tip_position];
%         %measure time stamp
%         cur_time = toc;
%         % Store the timestamps (in seconds) in an nx1 array
%         toc_data = [toc_data; cur_time];
%         %plot arm live
%         linear_angular_vel = pp.fdk3001(new_pos,new_vel);
%         quiver3(new_trans_matrix(1,4),new_trans_matrix(2,4),new_trans_matrix(3,4),linear_angular_vel(1),linear_angular_vel(2),linear_angular_vel(3),10);
%         axis equal
%         hold on
%         m.plot_arm(position');
%         drawnow;
%         %calculate 6x1 vector including the task-space linear velocities ̇and angular velocities
%         %using current position and current velocity.
%         curr_q = pp.fdk3001(position,new_vel);
%         %accumulate all the velocities vector
%         all_q = [all_q, curr_q];
%         
%        %inverse of the top 3x3 portion of the Jacobian for the current configuration
%        curr_jacob = pp.jacob3001(position);
%        curr_inv_jacob = inv([curr_jacob(1:3,1) curr_jacob(1:3,2) curr_jacob(1:3,3)]);
%        inv_jacob_data = [inv_jacob_data; curr_inv_jacob];
%         
%     end
%        
%  % Graph for sigh-off (same as part 3)
%  %xyz position(mm) vs time(s)
%     figure
%     hold on
%     plot(tip_position_data(:,1));
%     plot(tip_position_data(:,2));
%     plot(tip_position_data(:,3));
%     hold off
%     xlabel('time(s)');
%     ylabel('position(mm)');
%     legend({'x position','y position','z position'})
%     title('xyz position(mm) vs time(s)');
%    
% %joint angle (degree) vs time (s)
%     figure
%     hold on
%     plot(pos_data(:,1));
%     plot(pos_data(:,2));
%     plot(pos_data(:,3));
%     hold off
%     xlabel('time(s)');
%     ylabel('position(degree)');
%     legend({'x position','y position','z position'})
%     title('joint angle (degree) vs time (s)');
%    
% %Graph for report
% %xyz velocity
%     figure
%     hold on
%     v1 = diff(tip_position_data(:,1));
%     v2 = diff(tip_position_data(:,2));
%     v3 = diff(tip_position_data(:,3));
%     plot(v1);
%     plot(v2);
%     plot(v3);
%     hold off
%     xlabel('time(s)');
%     ylabel('velocity(mm/s)');
%     legend({'x velocity','y velocity','z velocity'})
%     title('xyz velocity');
%    
% %xyz acceleration
%     figure
%     hold on
%     a1 = diff(v1);
%     a2 = diff(v2);
%     a3 = diff(v3);
%     plot(a1);
%     plot(a2);
%     plot(a3);
%     hold off
%     xlabel('time(s)');
%     ylabel('acceleration(mm/s^2)');
%     legend({'x acceleration','y acceleration','z acceleration'})
%     title('xyz acceleration');

%% Extra Credit 2
% [x,z] = ginput(1);
% new_x = x*200
% new_z = z*200
% target = [new_x,0,new_z];
% pause(2);
% curr_joint = pp.ik_3001_numerical(target);
% m.plot_arm([curr_joint(1,1);curr_joint(2,1);curr_joint(3,1)]);
% view(0,0);

%% Extra Credit 3
% curr_pos = [0,0,0];
% [x,z] = ginput(1);
% new_x = x*200;
% new_z = z*200;
% target = [new_x,0,new_z];
% % pp.servo_jp([0,0,0]);
% pause(2);
% curr_joint = [0,0,0];
% %while ((abs(target(1)-curr_pos(1)) > 5)&&(abs(target(3)-curr_pos(3)) > 5))
% while (sqrt((target(1)-curr_pos(1))^2 + (target(3)-curr_pos(3))^2) > 5)
%     curr_pos_matrix = pp.fk3001(curr_joint);
%     curr_pos = [curr_pos_matrix(1,4),curr_pos_matrix(2,4),curr_pos_matrix(3,4)];
%     pos_vector = (target-curr_pos)./norm(target-curr_pos);
%     pos_vector = pos_vector * 8000;
%     %J-1*pos we get joint vel
%     Jacob = pp.jacob3001(curr_joint);
%     Upper_Jacob = [Jacob(1:3,1),Jacob(1:3,2),Jacob(1:3,3)];
%     inv_Jacob = inv(Upper_Jacob);
%     joint_velocity = inv_Jacob*(pos_vector');
%     %joint vel * time +original
%     t = 0.01;
%     joint_increment = t*joint_velocity;
%     curr_joint = curr_joint - joint_increment;
%     new_joint = [(curr_joint(1)+joint_increment(1)),(curr_joint(2)+joint_increment(2)),(curr_joint(3)+joint_increment(3))];
% %     pp.servo_jp([curr_joint(1,1),curr_joint(2,1),curr_joint(3,1)]);
%     hold on;
%     m.plot_arm([curr_joint(1,1);curr_joint(2,1);curr_joint(3,1)]);
%     view(0,0);
%     drawnow;
% end
% 
%     figure
%     m.plot_arm([curr_joint(1,1);curr_joint(2,1);curr_joint(3,1)]);
%     view(0,0);
%     
% new_x = x*200
% new_z = z*200
% joint_value = [curr_joint(1,1) curr_joint(2,1) curr_joint(3,1)]
% fk_output = pp.fk3001([curr_joint(1,1);curr_joint(2,1);curr_joint(3,1)])
% current_position = [fk_output(1,4) fk_output(2,4) fk_output(3,4)]

point1_ik_solution = pp.ik3001([117.0034  0  162.5608])
point2_ik_solution = pp.ik3001([66.6147   0  152.1514])
point3_ik_solution = pp.ik3001([142.7959  0  131.9142])


%    66.6147         0  152.1514
%117.0034         0  162.5608
























