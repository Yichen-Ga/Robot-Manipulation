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
%==========================================================================
% syms theta1 theta2 theta3
% trans01 = pp.dh2mat_sym([0,55,0,0]);
% trans12 = pp.dh2mat_sym([theta1,40,0,-90]);
% trans23 = pp.dh2mat_sym([theta2-90,0,100,0]);
% trans34 = pp.dh2mat_sym([theta3+90,0,100,0]);
% 
% trans02 = trans01*trans12;
% trans03 = trans02*trans23;
% trans04 = trans03*trans34;
% 
% pe = [trans04(1,4),trans04(2,4),trans04(3,4)];
% variable = [theta1, theta2, theta3];
% j_upper = jacobian(pe, variable);
% J_lower = [trans01(1:3,3) trans02(1:3,3) trans03(1:3,3)];
% jacob = [j_upper; J_lower];
% %% Signoff 1
% %==========================================================================
% %joint variables that would take the manipulator to an overhead configuration
% q1 = [0,0,-90];
% %Retuns a javobian with all 0 in the first column
% Jq_1 = pp.jacob3001(q1)
% Jp_1 = [Jq_1(1,:);Jq_1(2,:);Jq_1(3,:)];
% d_Jp1 = det(Jp_1)
% 
% q = [90 90 -90]; %[90 90 -90]
% %Retuns a javobian with all 0 in the first column
% Jq_2 = pp.jacob3001(q)
% Jp_2 = [Jq_2(1,:);Jq_2(2,:);Jq_2(3,:)];
% d_Jp2 = det(Jp_2)
%% Signoff 2
% % %==========================================================================
% %plan trajectory
%     %find corresponding joint value:
%     point_1 = pp.ik3001([51,-26,169]); %  -27.0127 -24.3843 34.2185
%     point_2 = pp.ik3001([-2,2,273]); %  135.0000 -26.2023 -35.7747
%     point_3 = pp.ik3001([74,-18,230]);%  -13.6713 -9.7663 -11.6101
%     point_4 = [0,0,-90];
%     curr_position = [];
%     %We visualize the vertercies in task space and feed them into cubic
%     %movement 1
%         t = 0;
%         ee_1_joint1 = traj5.cubic_traj(0,2,0,0,-27.0127,135.0000);
%         ee_1_joint2 = traj5.cubic_traj(0,2,0,0,-24.3843,-26.2023);
%         ee_1_joint3 = traj5.cubic_traj(0,2,0,0,34.2185,-35.7747);
%     %movement 2
%         ee_2_joint1 = traj5.cubic_traj(0,2,0,0,135.0000,0);
%         ee_2_joint2 = traj5.cubic_traj(0,2,0,0,-26.2023,0);
%         ee_2_joint3 = traj5.cubic_traj(0,2,0,0,-35.7747,-90);
%     %movement 3
%         ee_3_joint1 = traj5.cubic_traj(0,2,0,0,0,-27.0127);
%         ee_3_joint2 = traj5.cubic_traj(0,2,0,0,0,-24.3843);
%         ee_3_joint3 = traj5.cubic_traj(0,2,0,0,-90,34.2185);
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
%        all_mag = [];
%        all_ang_vel = [];
%        all_toc = [];
%        all_det = [];
%     %make the robot arm move to three point
%     pp.servo_jp([-27.0127, -24.3843, 34.2185]);
%     pause(3);
%     tic
%     while toc < 6.5
%        %each time the ii is different and it assign different position
%        %value
%        if (toc <= 2)
%            t = toc;
%            position = [traj5.cubic_traj_app(ee_1_joint1,t) traj5.cubic_traj_app(ee_1_joint2,t) traj5.cubic_traj_app(ee_1_joint3,t)];
%        end
%        if (2 < toc)&&(toc <= 4)
%            t = toc - 2;
%            position = [traj5.cubic_traj_app(ee_2_joint1,t) traj5.cubic_traj_app(ee_2_joint2,t) traj5.cubic_traj_app(ee_2_joint3,t)];
%        end
%        if (4 < toc)&&(toc <= 6)
%            t = toc - 4;
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
%         quiver3(new_trans_matrix(1,4),new_trans_matrix(2,4),new_trans_matrix(3,4),linear_angular_vel(1),linear_angular_vel(2),linear_angular_vel(3),0.008);
%         axis equal
%         hold on
%         m.plot_arm(position');
%         drawnow;
%         %calculate 6x1 vector including the task-space linear velocities ̇and angular velocities
%         %using current position and current velocity.
%         curr_q = pp.fdk3001(position,new_vel);
%         %accumulate all the velocities vector
%         all_q = [all_q, curr_q];
%         %calculate the magnitude of the vector
%         mag = sqrt(((linear_angular_vel(1)-new_trans_matrix(1,4))^2)+((linear_angular_vel(2)-new_trans_matrix(2,4))^2)+((linear_angular_vel(3)-new_trans_matrix(3,4))^2));
%         all_mag = [all_mag;mag];
%         curr_jacob = pp.jacob3001(position);
%         upper_jacob = [curr_jacob(1,:);curr_jacob(2,:);curr_jacob(3,:)]
%         det_1 = det(upper_jacob);
%         %current angular velocity
%         curr_ang_vel = [linear_angular_vel(4),linear_angular_vel(5),linear_angular_vel(6)];
%         all_ang_vel = [all_ang_vel;curr_ang_vel];
%         determinant = det(upper_jacob);
%         all_det = [all_det;determinant];
% %         if(det_1 < 100)
% %             f = errordlg('A sigularity is encountered at the current position of the motion','Motion Error');
% %             error('A sigularity is encountered at the current position of the motion');
% %         end
%     end

% % %Graph for sigh-off (same as part 3)
% % %Graph for report
% 
%     %xyz velocity
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
%     %xyz angular velocity
%     figure
%     hold on
%     v1 = all_ang_vel(:,1);
%     v2 = all_ang_vel(:,2);
%     v3 = all_ang_vel(:,3);
%     plot(toc_data,v1);
%     plot(toc_data,v2);
%     plot(toc_data,v3);
%     hold off
%     xlabel('time(s)');
%     ylabel('angular velocity(mm/s)');
%     legend({'1 joint velocity','2 joint velocity','3 joint velocity'})
%     title('angular velocity');
%     
%     % magnitude
%     figure
%     plot(toc_data,all_mag);
%     xlabel('time(s)');
%     ylabel('vector magnitude(mm/s)');
%     legend({'scalar speed'})
%     title('vector magnitude');
    
%     %3d path
%     figure
%     plot3(tip_position_data(:,1),tip_position_data(:,2),tip_position_data(:,3)),grid on;
%     title('3d path');
%     xlabel('x position(mm)');
%     ylabel('y position(mm)');
%     zlabel('z position(mm)');
%     title('3d path for tip position');
%     
%     %determinant
%     figure
%     plot(toc_data,all_det);
%     xlabel('time(s)');
%     ylabel('det');
%     legend({'determinant'})
%     title('det vs time');