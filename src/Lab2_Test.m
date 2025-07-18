%%
% RBE3001 - Laboratory 1 
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
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
try
%   This is for signoff 1
    %set the joint variable to 0,0,0
    v0 = [0,0,0];
    %calculate the trasformation matrix using the given joint configuration
    a0 = pp.fk3001(v0);
    %set the joint variable to calibration pose
    v1 = [-90.00, 86.14, 33.71];
    %calculate the trasformation matrix using the given joint configuration
    a1 = pp.fk3001(v1);
    %set the joint variable to 0,90,-90
    v2 = [0,90,-90];
    %calculate the trasformation matrix using the given joint configuration
    a2 = pp.fk3001(v2);
    %set the joint variable to 90,0,0
    v3 = [90,0,0];
    %calculate the trasformation matrix using the given joint configuration
    a3 = pp.fk3001(v3);

%generate the DH table using the given joint angle  
    q1 = [0,0,0]';
    full_dh_table_1=[0,55,0,0;
                      q1(1,1),40,0,-90;
                      q1(2,1)-90,0,100,0;
                      q1(3,1)+90,0,100,0;];

    q2 = [-90.00, 86.14, 33.71]';              
    full_dh_table_2=[0,55,0,0;
                      q2(1,1),40,0,-90;
                      q2(2,1)-90,0,100,0;
                      q2(3,1)+90,0,100,0;];  
    q3 = [0,90,-90]';            
    full_dh_table_3=[0,55,0,0;
                      q3(1,1),40,0,-90;
                      q3(2,1)-90,0,100,0;
                      q3(3,1)+90,0,100,0;];
    q4 = [90,0,0]';          
    full_dh_table_4=[0,55,0,0;
                      q4(1,1),40,0,-90;
                      q4(2,1)-90,0,100,0;
                      q4(3,1)+90,0,100,0;];

%calculate the intermediate transformation matrix using the dh table
    Q1_T01 = pp.dh2mat(full_dh_table_1(1,:))
    Q1_T12 = pp.dh2mat(full_dh_table_1(2,:))
    Q1_T23 = pp.dh2mat(full_dh_table_1(3,:))
    Q1_T34 = pp.dh2mat(full_dh_table_1(4,:))
    
    Q2_T01 = pp.dh2mat(full_dh_table_2(1,:))
    Q2_T12 = pp.dh2mat(full_dh_table_2(2,:))
    Q2_T23 = pp.dh2mat(full_dh_table_2(3,:))
    Q2_T34 = pp.dh2mat(full_dh_table_2(4,:))
    
    Q3_T01 = pp.dh2mat(full_dh_table_3(1,:))
    Q3_T12 = pp.dh2mat(full_dh_table_3(2,:))
    Q3_T23 = pp.dh2mat(full_dh_table_3(3,:))
    Q3_T34 = pp.dh2mat(full_dh_table_3(4,:))
    
    Q4_T01 = pp.dh2mat(full_dh_table_4(1,:))
    Q4_T12 = pp.dh2mat(full_dh_table_4(2,:))
    Q4_T23 = pp.dh2mat(full_dh_table_4(3,:))
    Q4_T34 = pp.dh2mat(full_dh_table_4(4,:))
    
%==========================================================================
%    This is for signoff 2
      v1 = [0,0,0];
      %make the robot arm move
      pp.servo_jp(v1);
      %test for the function measured_cp()
      a = pp.measured_cp()
      %test for the function setpoint_cp()
      b = pp.setpoint_cp()
      %test for the function goal_cp() 
      c = pp.goal_cp()
      %wait untill the move is done
      pause(2);
      %test measured_cp() again after the move is done
      d = pp.measured_cp()%%
%==============================================================
%Step 4
%move robot to zero position
pp.servo_jp([0,0,0]);
pause(1);
tip_position_data = [];
%loop 10 times
for i = 1:10
    v1 = [0,0,0];
    v2 = [-50,-30,-40];
    %make robot to the position we choose
    pp.servo_jp(v2);
    pause(0.5);
    %measure the current transformation matrix
    curr_tans_matrix = pp.measured_cp();
    %find the tip position using the position matrix in current transformation matrix
    %store the current tip position in a matrix
    curr_tip_position = [curr_tans_matrix(1,4),curr_tans_matrix(2,4),curr_tans_matrix(3,4)];
    %accumulate the tip position data in a matrix
    tip_position_data = [tip_position_data; curr_tip_position];
    pause(0.5);
    %move the robot arm to zero position
    pp.servo_jp(v1);
    pause(0.5);
end

%calculate the anticipated point position/coordinate using the joint angle
q = [-50,-30,-40]';
full_dh_table=[0,55,0,0;
                      q(1,1),40,0,-90;
                      q(2,1)-90,0,100,0;
                      q(3,1)+90,0,100,0;];
anticipated_point = pp.dh2fk(full_dh_table);

%average tip
mean = [mean(tip_position_data(:,1)),mean(tip_position_data(:,2)),mean(tip_position_data(:,3))]
%root mean square
root_mean_square = [rms(tip_position_data(:,1)),rms(tip_position_data(:,2)),rms(tip_position_data(:,3))]
%root mean square error
rms_error = abs(root_mean_square - abs(mean))

%scatter plot all the point
a = [[1 0 0];[0 1 0];[0 0 1];[0 1 1];[1 0 1];[1 1 0];[0 0 0];[0.8500 0.3250 0.0980];[0.4940 0.1840 0.5560];[0 0.4470 0.7410]];
scatter3(tip_position_data(:,1),tip_position_data(:,2),tip_position_data(:,3),[],a,'filled');
grid on
legend({'tip position'});







      
catch exception
    getReport(exception);

    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()



