% Continuously reads the current configuration of the arm as input 
% (using the methods we have previously created in our Robot class for doing so),
% and use our new Model class to visualize the stick model of the arm in real-time.

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
% % Create a model object to draw the real-time robot motion
m = Model(pp);
% send the robot arm to zero position
pp.servo_jp([0,0,0]);


% %sign_off_3
%  while true
% % store the current three joint angle
% cur_pos = pp.measure_js(1,0);
% %flip the matrix
% cur_pos = cur_pos(1,:)';
% % Pass joint angles to the plot_arm function to draw the current robot pos.
% m.plot_arm(cur_pos)
% % draw the graph live
% drawnow
%  end
%==================================================================
%sign_off_4
% assign 5 set of joint angle
v = [-50 -30 -40; 20 -70 -50; 80 -60 -20; -70 -10 -40; 50 -70 -80;];
FK = [];
setpoint = [];
%create a loop to do the same set of action 5 times
for i = 1:5
    tic
    %make the arm move
    pp.interpolate_jp(v(i,:), 2000);
    pause(1);

    while(toc <=2.005)
    %Store the current postion and flip it so plot_arm can take the 3x1 matrix
    cur_pos = pp.measure_js(1, 0);
    cur_pos = cur_pos(1,:)';
    %plot the arm live
    m.plot_arm(cur_pos)
    drawnow
    end
    pause(1);

    %calculate the set point each time and cumulate them in a matrix
    setpoint = [setpoint; pp.setpoint_js]
    %calculate the forward kinematics each time and cumulate them in a matrix
    FK = [FK; pp.setpoint_cp]
end
%==================================================================
% %sign_off_5
%        % assign empty matrix to the matrix that will change through the
%        % loop
%        tip_position_data = [];
%        toc_data= [];
%        pos_data = [];
%        vertices_x = [];
%        vertices_z = [];
%        vertices_y = [];
%        pause(1);
%     %make the robot arm move to three point
%     tic
%     for ii = [2,-2,0]
%        %each time the ii is different and it assign v1 different value
%        v1 = [0,ii*(20),ii*(-20)];
%        %make the robot arm move to v1 angle
%        pp.interpolate_jp(v1,1000);
%        %store the current time as curr_toc
%        cur_toc = toc;
%        %use the overall toc time minus the current time, when it's larger
%        %than 1, exit the while loo.
%        while (toc - cur_toc < 1)
%         %measure the current joint position
%         curr_pos = pp.measure_js (1, 0);
%         %store the current joint position
%         new_pos = curr_pos (1, :);
%         % Store the joint angles in an nx3 array.
%         pos_data = [pos_data; new_pos];
%         %measure the current tip position
%         curr_tans_matrix = pp.measured_cp();
%         % Store the current tip position in an 1x3 array.
%         curr_tip_position = [curr_tans_matrix(1,4),curr_tans_matrix(2,4),curr_tans_matrix(3,4)];
%         % Store the joint angles in an nx3 array.
%        
%         cur_time = toc*1000;
%         % Store the timestamps (in milliseconds) in an nx1 array
%         toc_data = [toc_data; cur_time];
%         tip_position_data = [tip_position_data; curr_tip_position];
%        end
%        pause(0.5)
%        %measure the current transformation matrix
%        curr_tans_matrix = pp.measured_cp();
%        %generate the current tip position
%        curr_tip_position = [curr_tans_matrix(1,4),curr_tans_matrix(2,4),curr_tans_matrix(3,4)];
%        %accumulate the x/y/z tip position in a matrix
%        vertices_x = [vertices_x curr_tip_position(1)];
%        vertices_z = [vertices_z curr_tip_position(3)];
%        vertices_y = [vertices_y curr_tip_position(2)];
%     end
%          %accumulate the time,position, and tip position data and out put
%          %as csv file
%          output_data = [toc_data pos_data tip_position_data];
%          csvwrite('lab2_signoff5_data.csv', output_data);
%          %read the csv file
%          m1 = readmatrix('lab2_signoff5_data.csv');
%          %specify which colomn is what
%          joint_angle1 = m1(:,2);
%          joint_angle2 = m1(:,3);
%          joint_angle3 = m1(:,4);
%          tip_position_x = m1(:,5);
%          tip_position_y = m1(:,6);
%          tip_position_z = m1(:,7);
%          
%          %se  the  logged  data to  create  a plot  with three  lines  
%          %representing  the joint  angles  (in  degrees)  vs time (in seconds), 
%          %using unique lines and legends.
%          hold on
%          plot(joint_angle1);
%          plot(joint_angle2);
%          plot(joint_angle3);
%          hold off
%          title('Joint Degree');
%          xlabel('time(ms)');
%          ylabel('position(degree)');
%          legend({'First joint','Second Joint','Third Joint'})
%          
%          
%          %Plot  two  lines  representing  tip  positions  in  x and  z  (in  mm)  vs time  (in  seconds).
%          figure
%          plot(tip_position_x);
%          hold on
%          plot(tip_position_z);
%          hold off
%          title('Tip Position');
%          xlabel('time(ms)');
%          ylabel('position(mm)');
%          legend({'x','z'})
%          
%          %Plot a 2D path representing the trajectory in the x-z plane followed by 
%          %the arm as it traversed all 3 setpoints, and overlay the triangle vertices 
%          %with points of a different color or style
%          figure
%          plot(tip_position_x,tip_position_z);
%          hold on
%          plot(vertices_x(1), vertices_z(1),'o','MarkerFaceColor','red');
%          plot(vertices_x(2), vertices_z(2),'o','MarkerFaceColor','red');
%          plot(vertices_x(3), vertices_z(3),'o','MarkerFaceColor','red');
%          title('Trajectory');
%          xlabel('tip position x');
%          ylabel('tip position z');
%          hold off
%          
%          %Repeat above tragectory in the x-y plane
%          figure
%          plot(tip_position_x,tip_position_y);
%          hold on
%          plot(vertices_x(1), vertices_y(1),'o','MarkerFaceColor','red');
%          plot(vertices_x(2), vertices_y(2),'o','MarkerFaceColor','red');
%          plot(vertices_x(3), vertices_y(3),'o','MarkerFaceColor','red');
%          title('Trajectory');
%          xlabel('tip position x');
%          ylabel('tip position y');
%          hold off



