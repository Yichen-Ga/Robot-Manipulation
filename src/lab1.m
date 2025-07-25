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
  packet = zeros(15, 1, 'single');
    
  tic
  %test the servo_jp function with a zero position array
  v1 = [0 0 0];
  v2 = [-90 -80 -80];
  pp.servo_jp(v1);
  pause(1);
  
  %test the interpolate_jp fnction with a zero position array and a few ms
  pp.interpolate_jp(v2, 2000);
  pause(0.1);

  tic
  setpoint_data = [];
  pos_data = [];
  vel_data = [];
  pos_vel_data = [];
  while (toc <= 2)  
      %test the setpoint_js function.
      new_setpoint = pp.setpoint_js();
      setpoint_data = [setpoint_data; new_setpoint];
      
      %test the measure_js function with different return information
      pos_infor = pp.measure_js(true, false);
      pos_data = [pos_data; pos_infor];
      vel_infor = pp.measure_js(false, true);
      vel_data = [vel_data; vel_infor];
      pos_vel_infor = pp.measure_js(true, true);
      pos_vel_data = [pos_vel_data;pos_vel_infor];
  end
  
  %test the setpoint_js function.
  end_of_motion = pp.goal_js();
 
  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
%   viaPts = [20,90,50];

%   for k = viaPts
%       tic
%       packet = zeros(15, 1, 'single');
%       packet(1) = 1000;%one second time
%       packet(2) = 0;%linear interpolation
%       packet(3) = k;
%       packet(4) = -80;% Second link to 0
%       packet(5) = -80;% Third link to 0
% 
%       % Send packet to the server and get the response      
%       %pp.write sends a 15 float packet to the micro controller
%        pp.write(SERV_ID, packet); 
%        %pp.read reads a returned 15 float backet from the micro controller.
%        returnPacket = pp.read(SERVER_ID_READ);
%       toc
% 
%       if DEBUG
%           disp('Sent Packet:')
%           disp(packet);
%           disp('Received Packet:');
%           disp(returnPacket);
%       end
%       
%       toc
%       pause(1) 
%       
%   end
  
  % Closes then opens the gripper
  pp.closeGripper()
  pause(1)
  pp.openGripper()
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
