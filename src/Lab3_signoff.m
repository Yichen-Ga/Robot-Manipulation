%%
% RBE3001 - Laboratory 3
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
%% This is for sign-off 1
% 
% %return a set of position for[100;0;195]in 1x3 vector;
% v0 = pp.ik3001([100;0;195])
% %return a set of position for [-2;2;273] in 1x3 vector;
% v1 = pp.ik3001([-2;2;273])
% %return a set of position in rad for [-74;-18;265] in 1x3 vector;
% v2 = pp.ik3001([-74;-18;265])
% 
% fk0 = pp.fk3001(v0');
% fk1 = pp.fk3001(v1');
% fk2 = pp.fk3001(v2');
% 
% p0 = [fk0(1,4),fk0(2,4),fk0(3,4)]
% p1 = [fk1(1,4),fk1(2,4),fk1(3,4)]
% p2 = [fk2(1,4),fk2(2,4),fk2(3,4)]
%==========================================================================
%% This is for sign-off 2
% % assign empty matrix to the matrix that will 
% % change through the loop
%        tip_position_data = [];
%        toc_data= [];
%        pos_data = [];
%        joint_angle = [];
%        curr_joint_angle = [];
%        curr_all_angle = [];
%        vertices_x = [];
%        vertices_z = [];
%        vertices_y = [];
%        t0_all = [];
%        v0_all = [];
%        p0_all = [];
%        tf_all = [];
%        vf_all = [];
%        pf_all = [];
%     %make the robot arm move to three point
%     v0_pos = [-74 -18 230];
%     v0 = pp.ik3001(v0_pos);
%     pp.servo_jp(v0);
%     pause(2);
%     tic
%     for ii = [1,2,3]
%        %each time the ii is different and it assign different position
%        %value
%        if (ii == 1)
%            position = [80; -26; 169];
%        end
%        if (ii == 2)
%            position = [100;0;195];
%        end
%        if (ii == 3)
%            position = [-74;-18;230];
%        end
%        v1 = pp.ik3001(position);
%        %make the robot arm move to v1 angle
%        pp.servo_jp(v1);
%        %store the current time as curr_toc
%        cur_toc = toc;
%        %use the overall toc time minus the current time, when it's larger
%        %than 1, exit the while loo.
%        while (toc - cur_toc < 0.5)
%         %measure the current joint angle position
%         curr_joint_angle = pp.setpoint_js();
%         %Store the joint angles in an nx3 array.
%         joint_angle = [joint_angle;curr_joint_angle];        
%         %measure the current tip position
%         curr_tans_matrix = pp.measured_cp();
%         % Store the current tip position in an 1x3 array.
%         curr_tip_position = [curr_tans_matrix(1,4),curr_tans_matrix(2,4),curr_tans_matrix(3,4)];
%         %accumulate all tip position data
%         tip_position_data = [tip_position_data; curr_tip_position];
%         %measure time stamp
%         cur_time = toc;
%         % Store the timestamps (in seconds) in an nx1 array
%         toc_data = [toc_data; cur_time];
%        end
%        %pause(0.5);
%        %accumulate current joint angle in a matrix ;
%        curr_all_angle = [curr_all_angle v1];
%     end
%         
% %sign-off 2 graph   
% %3d stick plot x3
%     figure
%     m.plot_arm(curr_all_angle(:,1));
%     
%     figure
%     m.plot_arm(curr_all_angle(:,2));
%     
%     figure
%     m.plot_arm(curr_all_angle(:,3));
%     
% %xyz position(mm) vs time(s)
%     figure
%     hold on
%     plot(tip_position_data(:,1));
%     plot(tip_position_data(:,2));
%     plot(tip_position_data(:,3));
%     hold off
%     xlabel('time(s)');
%     ylabel('position(mm)');
%     legend({'x position(mm)','y position(mm)','z position(mm)'})
%     title('xyz position(mm) vs time(s)');
%     
% %3d path
%     figure
%     plot3(tip_position_data(:,1),tip_position_data(:,2),tip_position_data(:,3)),grid on;
%     title('3d path');
%     xlabel('x position(mm)');
%     ylabel('y position(mm)');
%     zlabel('z position(mm)');
%     title('3d path for tip position');
%     
% %joint angle (degree) vs time (s)
%     figure
%     hold on
%     plot(joint_angle(:,1));
%     plot(joint_angle(:,2));
%     plot(joint_angle(:,3));
%     hold off
%     xlabel('time(ms)');
%     ylabel('position(degree)');
%     legend({'x position(degree)','y position(degree)','z position(degree)'})
%     title('joint angle (degree) vs time (s)');

% %==========================================================================
%% sign off 3
%plan trajectory
    %find corresponding joint value:
    point_1 = pp.ik3001([51,-26,169]); %  -27.0127 -24.3843 34.2185
    point_2 = pp.ik3001([-2,2,273]); %  135.0000 -26.2023 -35.7747
    point_3 = pp.ik3001([74,-18,230]);%  -13.6713 -9.7663 -11.6101
    curr_position = [];
    %We visualize the vertercies in task space and feed them into cubic
    %movement 1
        t = 0;
        ee_1_joint1 = traj5.cubic_traj(0,2,0,0,-27.0127,135.0000);
        ee_1_joint2 = traj5.cubic_traj(0,2,0,0,-24.3843,-26.2023);
        ee_1_joint3 = traj5.cubic_traj(0,2,0,0,34.2185,-35.7747);
    %movement 2
        ee_2_joint1 = traj5.cubic_traj(0,2,0,0,135.0000,-13.6713);
        ee_2_joint2 = traj5.cubic_traj(0,2,0,0,-26.2023,-9.7663);
        ee_2_joint3 = traj5.cubic_traj(0,2,0,0,-35.7747,-11.6101);
    %movement 3
        ee_3_joint1 = traj5.cubic_traj(0,2,0,0,-13.6713,-27.0127);
        ee_3_joint2 = traj5.cubic_traj(0,2,0,0,-9.7663,-24.3843);
        ee_3_joint3 = traj5.cubic_traj(0,2,0,0,-11.6101,34.2185);

%assign empty matrix to the matrix that will
%change through the loop
       tip_position_data = [];
       toc_data= [];
       pos_data = [];
       curr_joint_angle = [];
       curr_all_angle = [];
       t0_all = [];
       v0_all = [];
       p0_all = [];
       tf_all = [];
       vf_all = [];
       pf_all = [];
    %make the robot arm move to three point
    pp.servo_jp([-27.0127, -24.3843, 34.2185]);
    pause(3);
    tic
    while toc < 6.2
       %==========================================
       %signoff 3 part
       %measure the start time for each joint.
       t0 = toc;
       %store the start time in a matrix
       t0_all = [t0_all;t0];
       %measure the start velocity for each joint.
       pos_vel_matrix = pp.measure_js (0,1);
       v0 = pos_vel_matrix(2,:);
       %store the start velocity for each joint.
       v0_all = [v0_all;v0];
%        %measure the start position for each joint.
%        curr_pos_angle = pp.measure_js (1, 0);
%        p0 = [curr_pos_angle(1,1);curr_pos_angle(1,2);curr_pos_angle(1,3)];
%        %store the start position for each joint
%        p0_all = [p0_all;p0];
      %=============================================
       %each time the ii is different and it assign different position
       %value
       if (toc <= 2)
           t = toc;
           position = [traj5.cubic_traj_app(ee_1_joint1,t) traj5.cubic_traj_app(ee_1_joint2,t) traj5.cubic_traj_app(ee_1_joint3,t)];
       end
       if (2 < toc)&&(toc <= 4)
           t = toc - 2;
           position = [traj5.cubic_traj_app(ee_2_joint1,t) traj5.cubic_traj_app(ee_2_joint2,t) traj5.cubic_traj_app(ee_2_joint3,t)];
       end
       if (4 < toc)&&(toc <= 6)
           t = toc - 4;
           position = [traj5.cubic_traj_app(ee_3_joint1,t) traj5.cubic_traj_app(ee_3_joint2,t) traj5.cubic_traj_app(ee_3_joint3,t)];
       end
       curr_position = [curr_position; position];
       %make the robot arm move to v1 angle
       pp.servo_jp(position);
       %store the current time as curr_toc
       cur_toc = toc;
       %use the overall toc time minus the current time, when it's larger
       %than 1, exit the while loo.
       %measure the current joint position
        curr_pos = pp.measure_js (1, 0);
        %store the current joint angle
        new_pos = curr_pos (1, :);
        % Store the joint angles in an nx3 array.
        pos_data = [pos_data; new_pos];
        %measure the current tip position
        curr_tans_matrix = pp.measured_cp();
        % Store the current tip position in an 1x3 array.
        curr_tip_position = [curr_tans_matrix(1,4),curr_tans_matrix(2,4),curr_tans_matrix(3,4)];
        %accumulate all tip position data
        tip_position_data = [tip_position_data; curr_tip_position];
        %measure time stamp
        cur_time = toc;
        % Store the timestamps (in seconds) in an nx1 array
        toc_data = [toc_data; cur_time];
       %signoff 3 part=======================================
       %measure the final time for each joint.
       tf = toc;
       %store the final time in a matrix
       tf_all = [tf_all;tf];
       %measure the final velocity for each joint.
       pos_vel_matrix = pp.measure_js (0,1);
       vf = pos_vel_matrix(2,:);
       %store the final velocity for each joint.
       vf_all = [vf_all;vf];
       %=====================================================   
    end


        
 % Graph for sigh-off (same as part 3)

 %xyz position(mm) vs time(s)
    figure
    hold on
    plot(tip_position_data(:,1));
    plot(tip_position_data(:,2));
    plot(tip_position_data(:,3));
    hold off
    xlabel('time(s)');
    ylabel('position(mm)');
    legend({'x position','y position','z position'})
    title('xyz tip position(mm) vs time(s)');
    
%3d path
    figure
    plot3(tip_position_data(:,1),tip_position_data(:,2),tip_position_data(:,3)),grid on;
    xlabel('x position(mm)');
    ylabel('y position(mm)');
    zlabel('z position(mm)');
    title('3d path');
    
%joint angle (degree) vs time (s)
    figure
    hold on
    plot(pos_data(:,1));
    plot(pos_data(:,2));
    plot(pos_data(:,3));
    hold off
    xlabel('time(s)');
    ylabel('position(degree)');
    legend({'first joint','second joint','third joint'});
    title('joint position(degree) vs time(s)');
    
%Graph for report
%xyz velocity
    figure
    hold on
    v1 = diff(tip_position_data(:,1));
    v2 = diff(tip_position_data(:,2));
    v3 = diff(tip_position_data(:,3));
    plot(v1);
    plot(v2);
    plot(v3);
    hold off
    xlabel('time(s)');
    ylabel('velocity(mm/s)');
    legend({'x velocity','y velocity','z velocity'})
    title('xyz velocity');
    
%xyz acceleration
    figure
    hold on
    a1 = diff(v1);
    a2 = diff(v2);
    a3 = diff(v3);
    plot(a1);
    plot(a2);
    plot(a3);
    hold off
    xlabel('time(s)');
    ylabel('acceleration(mm/s^2)');
    legend({'x acceleration','y acceleration','z acceleration'})
    title('xyz acceleration');

    %movement 1
        %ee x
        t = 0;
        ee_1x = traj5.cubic_traj(0,2,0,0,-27.0127,135.0000);
        traj_1x = [];
        for t =0:(2/15):2
        q_1x = ee_1x(1) + ee_1x(2)*t + ee_1x(3)*t^2 + ee_1x(4)*t^3;
             traj_1x = [traj_1x;q_1x];
        end
        
        %ee y
        ee_1y = traj5.cubic_traj(0,2,0,0,-24.3843,-26.2023);
        traj_1y = [];
        for t = 0:(2/15):2
             q_1y = ee_1y(1) + ee_1y(2)*t + ee_1y(3)*t^2 + ee_1y(4)*t^3;
             traj_1y = [traj_1y;q_1y];
        end

        %ee z
        ee_1z = traj5.cubic_traj(0,2,0,0,34.2185,-35.7747);
        traj_1z = [];
        for t = 0:(2/15):2
             q_1z = ee_1z(1) + ee_1z(2)*t + ee_1z(3)*t^2 + ee_1z(4)*t^3;
             traj_1z = [traj_1z;q_1z];
        end
        
    %movement 2
        %ee x
        ee_2x = traj5.cubic_traj(0,2,0,0,135.0000,-13.6713);
        traj_2x = [];
        for t = 0:(2/15):2
             q_2x = ee_2x(1) + ee_2x(2)*t + ee_2x(3)*t^2 + ee_2x(4)*t^3;
             traj_2x = [traj_2x;q_2x];
        end
        
        %ee y
        ee_2y = traj5.cubic_traj(0,2,0,0,-26.2023,-9.7663);
        traj_2y = [];
        for t = 0:(2/15):2
             q_2y = ee_2y(1) + ee_2y(2)*t + ee_2y(3)*t^2 + ee_2y(4)*t^3;
             traj_2y = [traj_2y;q_2y];
        end
        
        %ee z
        ee_2z = traj5.cubic_traj(0,2,0,0,-35.7747,-11.6101);
        traj_2z = [];
        for t = 0:(2/15):2
             q_2z = ee_2z(1) + ee_2z(2)*t + ee_2z(3)*t^2 + ee_2z(4)*t^3;
             traj_2z = [traj_2z;q_2z];
        end
        
    %movement 3
        %ee x
        ee_3x = traj5.cubic_traj(0,2,0,0,-13.6713,-27.0127);
        traj_3x = [];
        for t = 0:(2/15):2
             q_3x = ee_3x(1) + ee_3x(2)*t + ee_3x(3)*t^2 + ee_3x(4)*t^3;
             traj_3x = [traj_3x;q_3x];
        end

        %ee y
        ee_3y = traj5.cubic_traj(0,2,0,0,-9.7663,-24.3843);
        traj_3y = [];
        for t = 0:(2/15):2
             q_3y = ee_3y(1) + ee_3y(2)*t + ee_3y(3)*t^2 + ee_3y(4)*t^3;
             traj_3y = [traj_3y;q_3y];
        end

        %ee z
        ee_3z = traj5.cubic_traj(0,2,0,0,-11.6101,34.2185);
        traj_3z = [];
        for t = 0:(2/15):2
             q_3z = ee_3z(1) + ee_3z(2)*t + ee_3z(3)*t^2 + ee_3z(4)*t^3;
             traj_3z = [traj_3z;q_3z];
        end
%==========================================================================
%% sign off 4
% %plan trajectory
%     %movement 1
%         t0_1 = [0 0 0];
%         tf_1 = [2 2 2];
%         p0_1 = [51 -26 169];
%         pf_1 = [-2 2 273];
%         v0_1 = [0 0 0];
%         vf_1 = [0 0 0];
%         a0_1 = [0 0 0];
%         af_1 = [0 0 0];
%         ee_1 = traj5.linear_traj(1,p0_1,pf_1,v0_1,vf_1,a0_1,af_1,t0_1,tf_1);
%     %movement 2
%         t0_2 = [0 0 0];
%         tf_2 = [2 2 2];
%         p0_2 = [-2 2 273];
%         pf_2 = [74 -18 230];
%         v0_2 = [0 0 0];
%         vf_2 = [0 0 0];
%         a0_2 = [0 0 0];
%         af_2 = [0 0 0];
%         ee_2 = traj5.linear_traj(1,p0_2,pf_2,v0_2,vf_2,a0_2,af_2,t0_2,tf_2);
%     %movement 3
%         t0_3 = [0 0 0];
%         tf_3 = [2 2 2];
%         p0_3 = [74 -18 230];
%         pf_3 = [51 -26 169];
%         v0_3 = [0 0 0];
%         vf_3 = [0 0 0];
%         a0_3 = [0 0 0];
%         af_3 = [0 0 0];
%         ee_3 = traj5.linear_traj(1,p0_3,pf_3,v0_3,vf_3,a0_3,af_3,t0_3,tf_3);
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
%     %make the robot arm move to three point
%     v0 = pp.ik3001([51,-26,169]);
%     pp.servo_jp(v0);
%     pause(3);
%     tic
%     while toc < 6.2
%        %==========================================
%        if (toc <= 2)
%            t = toc;
%            position = [traj5.cubic_traj_app(ee_1(:,1),t) traj5.cubic_traj_app(ee_1(:,2),t) traj5.cubic_traj_app(ee_1(:,3),t)];
%            v1 = pp.ik3001(position);
%        end
%        if (2 < toc)&&(toc <= 4)
%            t = toc - 2;
%            position = [traj5.cubic_traj_app(ee_2(:,1),t) traj5.cubic_traj_app(ee_2(:,2),t) traj5.cubic_traj_app(ee_2(:,3),t)];
%            v1 = pp.ik3001(position);
%        end
%        if (4 < toc)&&(toc <= 6)
%            t = toc - 4;
%            position = [traj5.cubic_traj_app(ee_3(:,1),t) traj5.cubic_traj_app(ee_3(:,2),t) traj5.cubic_traj_app(ee_3(:,3),t)];
%            v1 = pp.ik3001(position);
%        end
%        %make the robot arm move to v1 angle
%        pp.servo_jp(v1);
%        %store the current time as curr_toc
%        cur_toc = toc;
%        %use the overall toc time minus the current time, when it's larger
%        %than 1, exit the while loo.
%        %measure the current joint position
%         curr_pos = pp.measure_js (1, 0);
%         %store the current joint angle
%         new_pos = curr_pos (1, :);
%         % Store the joint angles in an nx3 array.
%         pos_data = [pos_data; new_pos];
%         %measure the current tip position
%         curr_tans_matrix = pp.measured_cp();
%         % Store the current tip position in an 1x3 array.
%         curr_tip_position = [curr_tans_matrix(1,4),curr_tans_matrix(2,4),curr_tans_matrix(3,4)];
%         %accumulate all tip position data
%         tip_position_data = [tip_position_data; curr_tip_position];
%         %measure time stamp
%         cur_time = toc;
%         % Store the timestamps (in seconds) in an nx1 array
%         toc_data = [toc_data; cur_time];
%        %measure the final time for each joint.
%        tf = toc;
%        %store the final time in a matrix
%        tf_all = [tf_all;tf];
%        %measure the final velocity for each joint.
%        pos_vel_matrix = pp.measure_js (0,1);
%        vf = pos_vel_matrix(2,:);
%        %store the final velocity for each joint.
%        vf_all = [vf_all;vf];
%        %=====================================================   
%     end
%     
%     %movement 1
%         %ee x
%         t = 0;
%         ee_1x = traj5.cubic_traj(0,2,0,0,51,-2);
%         traj_1x = [];
%         for t =0:(2/15):2
%         q_1x = ee_1x(1) + ee_1x(2)*t + ee_1x(3)*t^2 + ee_1x(4)*t^3;
%              traj_1x = [traj_1x;q_1x];
%         end
%         
%         %ee y
%         ee_1y = traj5.cubic_traj(0,2,0,0,-26,2);
%         traj_1y = [];
%         for t = 0:(2/15):2
%              q_1y = ee_1y(1) + ee_1y(2)*t + ee_1y(3)*t^2 + ee_1y(4)*t^3;
%              traj_1y = [traj_1y;q_1y];
%         end
% 
%         %ee z
%         ee_1z = traj5.cubic_traj(0,2,0,0,169,273);
%         traj_1z = [];
%         for t = 0:(2/15):2
%              q_1z = ee_1z(1) + ee_1z(2)*t + ee_1z(3)*t^2 + ee_1z(4)*t^3;
%              traj_1z = [traj_1z;q_1z];
%         end
%         
%     %movement 2
%         %ee x
%         ee_2x = traj5.cubic_traj(0,2,0,0,-2,74);
%         traj_2x = [];
%         for t = 0:(2/15):2
%              q_2x = ee_2x(1) + ee_2x(2)*t + ee_2x(3)*t^2 + ee_2x(4)*t^3;
%              traj_2x = [traj_2x;q_2x];
%         end
%         
%         %ee y
%         ee_2y = traj5.cubic_traj(0,2,0,0,2,-18);
%         traj_2y = [];
%         for t = 0:(2/15):2
%              q_2y = ee_2y(1) + ee_2y(2)*t + ee_2y(3)*t^2 + ee_2y(4)*t^3;
%              traj_2y = [traj_2y;q_2y];
%         end
%         
%         %ee z
%         ee_2z = traj5.cubic_traj(0,2,0,0,273,230);
%         traj_2z = [];
%         for t = 0:(2/15):2
%              q_2z = ee_2z(1) + ee_2z(2)*t + ee_2z(3)*t^2 + ee_2z(4)*t^3;
%              traj_2z = [traj_2z;q_2z];
%         end
%         
%     %movement 3
%         %ee x
%         ee_3x = traj5.cubic_traj(0,2,0,0,74,51);
%         traj_3x = [];
%         for t = 0:(2/15):2
%              q_3x = ee_3x(1) + ee_3x(2)*t + ee_3x(3)*t^2 + ee_3x(4)*t^3;
%              traj_3x = [traj_3x;q_3x];
%         end
% 
%         %ee y
%         ee_3y = traj5.cubic_traj(0,2,0,0,-18,-26);
%         traj_3y = [];
%         for t = 0:(2/15):2
%              q_3y = ee_3y(1) + ee_3y(2)*t + ee_3y(3)*t^2 + ee_3y(4)*t^3;
%              traj_3y = [traj_3y;q_3y];
%         end
% 
%         %ee z
%         ee_3z = traj5.cubic_traj(0,2,0,0,230,169);
%         traj_3z = [];
%         for t = 0:(2/15):2
%              q_3z = ee_3z(1) + ee_3z(2)*t + ee_3z(3)*t^2 + ee_3z(4)*t^3;
%              traj_3z = [traj_3z;q_3z];
%         end
%         
%     %movement 1
%         m1 = [traj_1x,traj_1y,traj_1z];
%     %movement 2
%         m2 = [traj_2x,traj_2y,traj_2z];
%     %movement 3
%         m3 = [traj_3x,traj_3y,traj_3z];
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
% %3d path
%     figure
%     pos2 = [0.1 0.1 1.4 0.8];
%     subplot('Position', pos2);
%     plot3(tip_position_data(:,1),tip_position_data(:,2),tip_position_data(:,3)),grid on;
%     hold all
%     scatter3(m1(:,1),m1(:,2),m1(:,3));
%      scatter3(m2(:,1),m2(:,2),m2(:,3));
%      scatter3(m3(:,1),m3(:,2),m3(:,3));
%     grid on
%     hold off
%     xlabel('x position(mm)');
%     ylabel('y position(mm)');
%     zlabel('z position(mm)');
%     legend({'tip position for planned trajectory'});
%     title('3d path');
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
% %==========================================================================
%% sign off 5
% %plan trajectory
%     %movement 1
%         t0_1 = [0 0 0];
%         tf_1 = [2 2 2];
%         p0_1 = [51 -26 169];
%         pf_1 = [-2 2 273];
%         v0_1 = [0 0 0];
%         vf_1 = [0 0 0];
%         a0_1 = [0 0 0];
%         af_1 = [0 0 0];
%         ee_1 = traj5.linear_traj(0,p0_1,pf_1,v0_1,vf_1,a0_1,af_1,t0_1,tf_1);
%     %movement 2
%         t0_2 = [0 0 0];
%         tf_2 = [2 2 2];
%         p0_2 = [-2 2 273];
%         pf_2 = [74 -18 230];
%         v0_2 = [0 0 0];
%         vf_2 = [0 0 0];
%         a0_2 = [0 0 0];
%         af_2 = [0 0 0];
%         ee_2 = traj5.linear_traj(0,p0_2,pf_2,v0_2,vf_2,a0_2,af_2,t0_2,tf_2);
%     %movement 3
%         t0_3 = [0 0 0];
%         tf_3 = [2 2 2];
%         p0_3 = [74 -18 230];
%         pf_3 = [51 -26 169];
%         v0_3 = [0 0 0];
%         vf_3 = [0 0 0];
%         a0_3 = [0 0 0];
%         af_3 = [0 0 0];
%         ee_3 = traj5.linear_traj(0,p0_3,pf_3,v0_3,vf_3,a0_3,af_3,t0_3,tf_3);
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
%     %make the robot arm move to three point
%     v0 = pp.ik3001([51,-26,169]);
%     pp.servo_jp(v0);
%     pause(5);
%     tic
%     while toc < 6.2
%        %==========================================
%        %measure the start time for each joint.
%        t0 = toc;
%        %store the start time in a matrix
%        t0_all = [t0_all;t0];
%        %measure the start velocity for each joint.
%        pos_vel_matrix = pp.measure_js (0,1);
%        v0 = pos_vel_matrix(2,:);
%        %store the start velocity for each joint.
%        v0_all = [v0_all;v0];
%        if (toc <= 2)
%            t = toc;
%            position = [traj5.quintic_traj_app(ee_1(:,1),t) traj5.quintic_traj_app(ee_1(:,2),t) traj5.quintic_traj_app(ee_1(:,3),t)];
%            v1 = pp.ik3001(position);
%        end
%        if (2 < toc)&&(toc <= 4)
%            t = toc - 2;
%            position = [traj5.quintic_traj_app(ee_2(:,1),t) traj5.quintic_traj_app(ee_2(:,2),t) traj5.quintic_traj_app(ee_2(:,3),t)];
%            v1 = pp.ik3001(position);
%        end
%        if (4 < toc)&&(toc <= 6)
%            t = toc - 4;
%            position = [traj5.quintic_traj_app(ee_3(:,1),t) traj5.quintic_traj_app(ee_3(:,2),t) traj5.quintic_traj_app(ee_3(:,3),t)];
%            v1 = pp.ik3001(position);
%        end
%        %make the robot arm move to v1 angle
%        pp.servo_jp(v1);
%        %store the current time as curr_toc
%        cur_toc = toc;
%        %use the overall toc time minus the current time, when it's larger
%        %than 1, exit the while loo.
%        %measure the current joint position
%         curr_pos = pp.measure_js (1, 0);
%         %store the current joint angle
%         new_pos = curr_pos (1, :);
%         % Store the joint angles in an nx3 array.
%         pos_data = [pos_data; new_pos];
%         %measure the current tip position
%         curr_tans_matrix = pp.measured_cp();
%         % Store the current tip position in an 1x3 array.
%         curr_tip_position = [curr_tans_matrix(1,4),curr_tans_matrix(2,4),curr_tans_matrix(3,4)];
%         %accumulate all tip position data
%         tip_position_data = [tip_position_data; curr_tip_position];
%         %measure time stamp
%         cur_time = toc;
%         % Store the timestamps (in seconds) in an nx1 array
%         toc_data = [toc_data; cur_time];
%        %measure the final time for each joint.
%        tf = toc;
%        %store the final time in a matrix
%        tf_all = [tf_all;tf];
%        %measure the final velocity for each joint.
%        pos_vel_matrix = pp.measure_js (0,1);
%        vf = pos_vel_matrix(2,:);
%        %store the final velocity for each joint.
%        vf_all = [vf_all;vf];
%        %=====================================================   
%     end
% 
% 
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
% %3d path
%     figure
%     plot3(tip_position_data(:,1),tip_position_data(:,2),tip_position_data(:,3)),grid on;
%     xlabel('x position(mm)');
%     ylabel('y position(mm)');
%     zlabel('z position(mm)');
%     title('3d path');
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
%     
%     %movement 1
%         %ee x
%         t = 0;
%         ee_1x = traj5.cubic_traj(0,2,0,0,51,-2);
%         traj_1x = [];
%         for t =0:(2/15):2
%         q_1x = ee_1x(1) + ee_1x(2)*t + ee_1x(3)*t^2 + ee_1x(4)*t^3;
%              traj_1x = [traj_1x;q_1x];
%         end
%         
%         %ee y
%         ee_1y = traj5.cubic_traj(0,2,0,0,-26,2);
%         traj_1y = [];
%         for t = 0:(2/15):2
%              q_1y = ee_1y(1) + ee_1y(2)*t + ee_1y(3)*t^2 + ee_1y(4)*t^3;
%              traj_1y = [traj_1y;q_1y];
%         end
% 
%         %ee z
%         ee_1z = traj5.cubic_traj(0,2,0,0,169,273);
%         traj_1z = [];
%         for t = 0:(2/15):2
%              q_1z = ee_1z(1) + ee_1z(2)*t + ee_1z(3)*t^2 + ee_1z(4)*t^3;
%              traj_1z = [traj_1z;q_1z];
%         end
%         
%     %movement 2
%         %ee x
%         ee_2x = traj5.cubic_traj(0,2,0,0,-2,74);
%         traj_2x = [];
%         for t = 2:(2/15):4
%              q_2x = ee_2x(1) + ee_2x(2)*t + ee_2x(3)*t^2 + ee_2x(4)*t^3;
%              traj_2x = [traj_2x;q_2x];
%         end
%         
%         %ee y
%         ee_2y = traj5.cubic_traj(0,2,0,0,2,-18);
%         traj_2y = [];
%         for t = 2:(2/15):4
%              q_2y = ee_2y(1) + ee_2y(2)*t + ee_2y(3)*t^2 + ee_2y(4)*t^3;
%              traj_2y = [traj_2y;q_2y];
%         end
%         
%         %ee z
%         ee_2z = traj5.cubic_traj(0,2,0,0,273,230);
%         traj_2z = [];
%         for t = 2:(2/15):4
%              q_2z = ee_2z(1) + ee_2z(2)*t + ee_2z(3)*t^2 + ee_2z(4)*t^3;
%              traj_2z = [traj_2z;q_2z];
%         end
%         
%     %movement 3
%         %ee x
%         ee_3x = traj5.cubic_traj(0,2,0,0,74,51);
%         traj_3x = [];
%         for t = 4:(2/15):6
%              q_3x = ee_3x(1) + ee_3x(2)*t + ee_3x(3)*t^2 + ee_3x(4)*t^3;
%              traj_3x = [traj_3x;q_3x];
%         end
% 
%         %ee y
%         ee_3y = traj5.cubic_traj(0,2,0,0,-18,-26);
%         traj_3y = [];
%         for t = 4:(2/15):6
%              q_3y = ee_3y(1) + ee_3y(2)*t + ee_3y(3)*t^2 + ee_3y(4)*t^3;
%              traj_3y = [traj_3y;q_3y];
%         end
% 
%         %ee z
%         ee_3z = traj5.cubic_traj(0,2,0,0,230,169);
%         traj_3z = [];
%         for t = 4:(2/15):6
%              q_3z = ee_3z(1) + ee_3z(2)*t + ee_3z(3)*t^2 + ee_3z(4)*t^3;
%              traj_3z = [traj_3z;q_3z];
%         end
% %==========================================================================