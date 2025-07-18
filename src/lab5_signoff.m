%%
% RBE 3001 Lab 5 example code!
% Developed by Alex Tacescu (https://alextac.com)
%%
clc;
clear;
clear java;
format short

%% Flags
DEBUG = false;
STICKMODEL = false;
DEBUG_CAM = false;

%% Setup
vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

robot = Robot(myHIDSimplePacketComs);
traj = Traj_Planner(robot);

cam = Camera();
cam.DEBUG = DEBUG_CAM;
robot.servo_jp([0,0,-90]);
pause(0.5);
disp('5 second left')
pause();
a=10;A=a(ones(10, 10)); 
Avg_mask = A;
    rotation_matrix = [cam.cam_pose(1,1:3);cam.cam_pose(2,1:3);cam.cam_pose(3,1:3)];
    translation_vector = [cam.cam_pose(1,4);cam.cam_pose(2,4);cam.cam_pose(3,4)];
%     ptw = pointsToWorld(cam.params.Intrinsics,rotation_matrix,translation_vector,[211 152]);
%     trans_vector_checker_frame = [ptw(1);ptw(2); 0; 1];
    T_0_checker = [0,1,0,50;1,0,0,-100;0,0,-1,12;0,0,0,1];
%%
tic
while toc < 20
    
    im = snapshot(cam.cam);
    
    [greenBW,greenRGB] = greengreenMask(im);
    greenMed = medfilt2(greenBW);
    greenAvg = imfilter(greenMed,Avg_mask);
    greenEdge = edge(greenAvg);
    greenFill = imfill(greenEdge,'holes');
    greenBound = bwboundaries(greenFill);
    greenBoundary = greenBound{1,1};
    greenPoly = polyshape(greenBoundary(:,1),greenBoundary(:,2));
    [greenCenterY, greenCenterX] = centroid(greenPoly);
    
    
    ptwGreen = pointsToWorld(cam.params.Intrinsics,rotation_matrix,translation_vector,[greenCenterX, greenCenterY]);
    
    GreenYtocamera = 150 - ptwGreen(2);
    GreenXtocamera = 100 - ptwGreen(1);
    
    Green_Checker_Vector = [ptwGreen(1);ptwGreen(2); 0; 1];
    Green_coord_in_robotFrame = (T_0_checker)*(Green_Checker_Vector);
    
    
    %ik for The point straight above green ball
    Green_above = robot.ik3001([Green_coord_in_robotFrame(1);Green_coord_in_robotFrame(2);120]);
    
    robot.servo_jp([Green_above(1)-10,Green_above(2),Green_above(3)]);
end