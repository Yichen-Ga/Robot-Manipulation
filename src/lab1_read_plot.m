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
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 60
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
%   packet = zeros(15, 1, 'single');
%   
%   % prepare the vectors we need to turn to.
%   v1 = [0, 0, 0];
%   v2 = [45, 0, 0];
%   % let the robot go to the zero position
%   pp.servo_jp(v1);
%   pause(1);
%   % send the robot base angle from 0 to 45 dgree using three second
%   pp.interpolate_jp(v2, 3000);
%   new_pos = [0, 0, 0];
%   pos_data = [];
%   toc_data = [];
  tic
  
  % During  the  trajectory,  continuously  read  the  current  joint  positions  and  record  their timestamps.
  while (toc <= 10)
      pos_temp = pp.measure_js (1, 0);
      new_pos = pos_temp (1, :);
      % Store the joint positions in an nx3 array.
      pos_data = [pos_data; new_pos];
      cur_time = toc*1000;
      % Store the timestamps (in milliseconds) in an nx1 array
      toc_data = [toc_data; cur_time];
  end
  
  
  %Create a csv file, where each row stores a timestamp and its corresponding joint values.
%   output_data = [toc_data pos_data];
%   csvwrite('lab1_data.csv', output_data);
%   time_step = diff(toc_data, 1, 1);

 % Create a figure showing 3 subplots representing the motion profile of each of the 3 joints
% pos1 = [0.1 0.5 0.25 0.25];
% subplot('Position', pos1);
% a = pos_data(:,1);
% plot(a);
% title('First Joint Position');
% xlabel('time(ms)');
% ylabel('position(degree)');
% 
% pos2 = [0.38 0.5 0.25 0.25];
% subplot('Position', pos2);
% b = pos_data(:,2);
% plot(b);
% title('Second Joint Position');
% xlabel('time(ms)');
% ylabel('position(degree)');
% 
% pos3 = [0.7 0.5 0.25 0.25];
% subplot('Position', pos3);
% c = pos_data(:,3);
% plot(c);
% title('Third Joint Position');
% xlabel('time(ms)');
% ylabel('position(degree)');
% 
% % Create a histogram of the incremental timesteps between each reading. 
% figure
% histogram(time_step, 100, 'BinLimits',[0,1]);
% title('Packet delay');
% xlabel('time(ms)');
% ylabel('amount');
% 
% 
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

