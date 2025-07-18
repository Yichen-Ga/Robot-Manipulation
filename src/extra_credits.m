%%
% RBE3001 - Laboratory
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

% %lab2 extra_credit 1
% %plot the workspace of our robot
% % set the x,y,z information
% x = 10;
% y = 210;
% z = 10;
% 
% %calculate the parameters
% theta = atan(y/x);
% x_y = sqrt(x^2 + y^2);
% phi = atan(x_y/z);
% R=sqrt(x_y^2 + z^2);
% Phi=linspace(0,phi);
% Theta=linspace(-theta,theta);
% %plot the workspace of our robot
% [Phi,Theta]=meshgrid(Phi,Theta);
% [X,Y,Z]=sph2cart(Theta,Phi,R);
% surf(X,Y,Z);
% 

%--------------------------------------------

%lab4 extra_credit 1
%plan trajectory
    %movement 1
        t0_1 = [0 0 0];
        tf_1 = [1 1 1];
        p0_1 = [100 0 195];
        pf_1 = [100 -30 152.5];
        v0_1 = [0 0 0];
        vf_1 = [0 0 0];
        a0_1 = [0 0 0];
        af_1 = [0 0 0];
        ee_1 = traj5.linear_traj(1,p0_1,pf_1,v0_1,vf_1,a0_1,af_1,t0_1,tf_1);
    %movement 2
        t0_2 = [0 0 0];
        tf_2 = [1 1 1];
        p0_2 = [100 -30 152.5];
        pf_2 = [100 30 152.5];
        v0_2 = [0 0 0];
        vf_2 = [0 0 0];
        a0_2 = [0 0 0];
        af_2 = [0 0 0];
        ee_2 = traj5.linear_traj(1,p0_2,pf_2,v0_2,vf_2,a0_2,af_2,t0_2,tf_2);
    %movement 3
        t0_3 = [0 0 0];
        tf_3 = [1 1 1];
        p0_3 = [100 30 152.5];
        pf_3 = [100 0 195];
        v0_3 = [0 0 0];
        vf_3 = [0 0 0];
        a0_3 = [0 0 0];
        af_3 = [0 0 0];
        ee_3 = traj5.linear_traj(1,p0_3,pf_3,v0_3,vf_3,a0_3,af_3,t0_3,tf_3);
     %movement 4
        t0_4 = [0 0 0];
        tf_4 = [1 1 1];
        p0_4 = [100 0 195];
        pf_4 = [91.5 0 172.5];
        v0_4 = [0 0 0];
        vf_4 = [0 0 0];
        a0_4 = [0 0 0];
        af_4 = [0 0 0];
        ee_4 = traj5.linear_traj(1,p0_4,pf_4,v0_4,vf_4,a0_4,af_4,t0_4,tf_4);
     %movement 5
        t0_5 = [0 0 0];
        tf_5 = [1 1 1];
        p0_5 = [91.5 0 172.5];
        pf_5 = [100 -30 152.5];
        v0_5 = [0 0 0];
        vf_5 = [0 0 0];
        a0_5 = [0 0 0];
        af_5 = [0 0 0];
        ee_5 = traj5.linear_traj(1,p0_5,pf_5,v0_5,vf_5,a0_5,af_5,t0_5,tf_5);
     %movement 6
        t0_5 = [0 0 0];
        tf_5 = [1 1 1];
        p0_5 = [100 -30 152.5];
        pf_5 = [100 30 152.5];
        v0_5 = [0 0 0];
        vf_5 = [0 0 0];
        a0_5 = [0 0 0];
        af_5 = [0 0 0];
        ee_6 = traj5.linear_traj(1,p0_5,pf_5,v0_5,vf_5,a0_5,af_5,t0_5,tf_5);
     %movement 7
        t0_5 = [0 0 0];
        tf_5 = [1 1 1];
        p0_5 = [100 30 152.5];
        pf_5 = [91.5 0 172.5];
        v0_5 = [0 0 0];
        vf_5 = [0 0 0];
        a0_5 = [0 0 0];
        af_5 = [0 0 0];
        ee_7 = traj5.linear_traj(1,p0_5,pf_5,v0_5,vf_5,a0_5,af_5,t0_5,tf_5);

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
    v0 = pp.ik3001([100 0 195]);
    pp.servo_jp(v0);
    pause(3);
    tic
    while toc < 7.2
       %==========================================
       %measure the start time for each joint.
       t0 = toc;
       %store the start time in a matrix
       t0_all = [t0_all;t0];
       %measure the start velocity for each joint.
       pos_vel_matrix = pp.measure_js (0,1);
       v0 = pos_vel_matrix(2,:);
       %store the start velocity for each joint.
       v0_all = [v0_all;v0];
       if (toc <= 1)
           t = toc;
           position = [traj5.cubic_traj_app(ee_1(:,1),t) traj5.cubic_traj_app(ee_1(:,2),t) traj5.cubic_traj_app(ee_1(:,3),t)];
           v1 = pp.ik3001(position);
       end
       if (1 < toc)&&(toc <= 2)
           t = toc - 1;
           position = [traj5.cubic_traj_app(ee_2(:,1),t) traj5.cubic_traj_app(ee_2(:,2),t) traj5.cubic_traj_app(ee_2(:,3),t)];
           v1 = pp.ik3001(position);
       end
       if (2 < toc)&&(toc <= 3)
           t = toc - 2;
           position = [traj5.cubic_traj_app(ee_3(:,1),t) traj5.cubic_traj_app(ee_3(:,2),t) traj5.cubic_traj_app(ee_3(:,3),t)];
           v1 = pp.ik3001(position);
       end
       if (3 < toc)&&(toc <= 4)
           t = toc - 3;
           position = [traj5.cubic_traj_app(ee_4(:,1),t) traj5.cubic_traj_app(ee_4(:,2),t) traj5.cubic_traj_app(ee_4(:,3),t)];
           v1 = pp.ik3001(position);
       end
       if (4 < toc)&&(toc <= 5)
           t = toc - 4;
           position = [traj5.cubic_traj_app(ee_5(:,1),t) traj5.cubic_traj_app(ee_5(:,2),t) traj5.cubic_traj_app(ee_5(:,3),t)];
           v1 = pp.ik3001(position);
       end
       if (5 < toc)&&(toc <= 6)
           t = toc - 5;
           position = [traj5.cubic_traj_app(ee_6(:,1),t) traj5.cubic_traj_app(ee_6(:,2),t) traj5.cubic_traj_app(ee_6(:,3),t)];
           v1 = pp.ik3001(position);
       end
       if (6 < toc)&&(toc <= 7)
           t = toc - 6;
           position = [traj5.cubic_traj_app(ee_7(:,1),t) traj5.cubic_traj_app(ee_7(:,2),t) traj5.cubic_traj_app(ee_7(:,3),t)];
           v1 = pp.ik3001(position);
       end
       %make the robot arm move to v1 angle
       pp.servo_jp(v1);
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
    title('xyz position(mm) vs time(s)');
    
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
    legend({'x position','y position','z position'})
    title('joint angle (degree) vs time (s)');
    
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
        ee_1x = traj5.cubic_traj(0,2,0,0,51,-2);
        traj_1x = [];
        for t =0:(2/15):2
        q_1x = ee_1x(1) + ee_1x(2)*t + ee_1x(3)*t^2 + ee_1x(4)*t^3;
             traj_1x = [traj_1x;q_1x];
        end
        
        %ee y
        ee_1y = traj5.cubic_traj(0,2,0,0,-26,2);
        traj_1y = [];
        for t = 0:(2/15):2
             q_1y = ee_1y(1) + ee_1y(2)*t + ee_1y(3)*t^2 + ee_1y(4)*t^3;
             traj_1y = [traj_1y;q_1y];
        end

        %ee z
        ee_1z = traj5.cubic_traj(0,2,0,0,169,273);
        traj_1z = [];
        for t = 0:(2/15):2
             q_1z = ee_1z(1) + ee_1z(2)*t + ee_1z(3)*t^2 + ee_1z(4)*t^3;
             traj_1z = [traj_1z;q_1z];
        end
        
    %movement 2
        %ee x
        ee_2x = traj5.cubic_traj(0,2,0,0,-2,74);
        traj_2x = [];
        for t = 2:(2/15):4
             q_2x = ee_2x(1) + ee_2x(2)*t + ee_2x(3)*t^2 + ee_2x(4)*t^3;
             traj_2x = [traj_2x;q_2x];
        end
        
        %ee y
        ee_2y = traj5.cubic_traj(0,2,0,0,2,-18);
        traj_2y = [];
        for t = 2:(2/15):4
             q_2y = ee_2y(1) + ee_2y(2)*t + ee_2y(3)*t^2 + ee_2y(4)*t^3;
             traj_2y = [traj_2y;q_2y];
        end
        
        %ee z
        ee_2z = traj5.cubic_traj(0,2,0,0,273,230);
        traj_2z = [];
        for t = 2:(2/15):4
             q_2z = ee_2z(1) + ee_2z(2)*t + ee_2z(3)*t^2 + ee_2z(4)*t^3;
             traj_2z = [traj_2z;q_2z];
        end
        
    %movement 3
        %ee x
        ee_3x = traj5.cubic_traj(0,2,0,0,74,51);
        traj_3x = [];
        for t = 4:(2/15):6
             q_3x = ee_3x(1) + ee_3x(2)*t + ee_3x(3)*t^2 + ee_3x(4)*t^3;
             traj_3x = [traj_3x;q_3x];
        end

        %ee y
        ee_3y = traj5.cubic_traj(0,2,0,0,-18,-26);
        traj_3y = [];
        for t = 4:(2/15):6
             q_3y = ee_3y(1) + ee_3y(2)*t + ee_3y(3)*t^2 + ee_3y(4)*t^3;
             traj_3y = [traj_3y;q_3y];
        end

        %ee z
        ee_3z = traj5.cubic_traj(0,2,0,0,230,169);
        traj_3z = [];
        for t = 4:(2/15):6
             q_3z = ee_3z(1) + ee_3z(2)*t + ee_3z(3)*t^2 + ee_3z(4)*t^3;
             traj_3z = [traj_3z;q_3z];
        end