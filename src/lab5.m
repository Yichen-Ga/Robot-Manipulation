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
%%
Green_Position = [0,125,15];
Green_Position_Above = [0,125,100];
Red_Position = [75,125,15];
Red_Position_Above = [75,125,100];
Yellow_Position = [0,-140,15];
Yellow_Position_Above = [0,-140,100];
Orange_Position = [75,-132,15];
Orange_Position_Above = [75,-132,100];
Blue_Position = [120,120,23];
Blue_Position_Above = [120,120,100];
%% Main Loop

a=10;A=a(ones(10, 10)); 
Avg_mask = A;
ballH = 12;%mm

% try
%     %Set up camera
%     if cam.params == 0
%         error("No camera parameters found!");
%     end
    rotation_matrix = [cam.cam_pose(1,1:3);cam.cam_pose(2,1:3);cam.cam_pose(3,1:3)];
    translation_vector = [cam.cam_pose(1,4);cam.cam_pose(2,4);cam.cam_pose(3,4)];
%     ptw = pointsToWorld(cam.params.Intrinsics,rotation_matrix,translation_vector,[211 152]);
%     trans_vector_checker_frame = [ptw(1);ptw(2); 0; 1];
    T_0_checker = [0,1,0,50;1,0,0,-100;0,0,-1,12;0,0,0,1];
%     coord_in_robotFrame = (T_0_checker)*(trans_vector_checker_frame);
% catch exception
%     fprintf('\n ERROR!!! \n \n');
%     disp(getReport(exception));
%     disp('Exited on error, clean shutdown');
% end




while true
    
im = snapshot(cam.cam);

[greenBW,greenRGB] = greenMask(im);
greenMed = medfilt2(greenBW);
greenAvg = imfilter(greenMed,Avg_mask);
greenEdge = edge(greenAvg);
greenFill = imfill(greenEdge,'holes');
greenBound = bwboundaries(greenFill);
greenBoundary = greenBound{1,1};
greenPoly = polyshape(greenBoundary(:,1),greenBoundary(:,2));
[greenCenterY, greenCenterX] = centroid(greenPoly);

[redBW,redRGB] = redMask(im);
redMed = medfilt2(redBW);
redAvg = imfilter(redMed,Avg_mask);
redEdge = edge(redAvg);
redFill = imfill(redEdge,'holes');
redBound = bwboundaries(redFill);
redBoundary = redBound{1,1};
redPoly = polyshape(redBoundary(:,1),redBoundary(:,2));
[redCenterY, redCenterX] = centroid(redPoly);

[yellowBW,yellowRGB] = yellowMask(im);
yellowMed = medfilt2(yellowBW);
yellowAvg = imfilter(yellowMed,Avg_mask);
yellowEdge = edge(yellowAvg);
yellowFill = imfill(yellowEdge,'holes');
yellowBound = bwboundaries(yellowFill);
yellowBoundary = yellowBound{1,1};
yellowPoly = polyshape(yellowBoundary(:,1),yellowBoundary(:,2));
[yellowCenterY, yellowCenterX] = centroid(yellowPoly);

[orangeBW,orangeRGB] = orangeMask(im);
orangeMed = medfilt2(orangeBW);
orangeAvg = imfilter(orangeMed,Avg_mask);
orangeEdge = edge(orangeAvg);
orangeFill = imfill(orangeEdge,'holes');
orangeBound = bwboundaries(orangeFill);
orangeBoundary = orangeBound{1,1};
orangePoly = polyshape(orangeBoundary(:,1),orangeBoundary(:,2));
[orangeCenterY, orangeCenterX] = centroid(orangePoly);

[BlueBW,BlueRGB] = BlueMask(im);
BlueMed = medfilt2(BlueBW);
BlueAvg = imfilter(BlueMed,Avg_mask);
BlueEdge = edge(BlueAvg);
BlueFill = imfill(BlueEdge,'holes');
BlueBound = bwboundaries(BlueFill);
BlueBoundary = BlueBound{1,1};
BluePoly = polyshape(BlueBoundary(:,1),BlueBoundary(:,2));
[BlueCenterY, BlueCenterX] = centroid(BluePoly);

%%
    ptwGreen = pointsToWorld(cam.params.Intrinsics,rotation_matrix,translation_vector,[greenCenterX, greenCenterY]);
    ptwRed = pointsToWorld(cam.params.Intrinsics,rotation_matrix,translation_vector,[redCenterX, redCenterY]);
    ptwYellow = pointsToWorld(cam.params.Intrinsics,rotation_matrix,translation_vector,[yellowCenterX, yellowCenterY]);
    ptwOrange = pointsToWorld(cam.params.Intrinsics,rotation_matrix,translation_vector,[orangeCenterX, orangeCenterY]);
    ptwBlue = pointsToWorld(cam.params.Intrinsics,rotation_matrix,translation_vector,[BlueCenterX, BlueCenterY]);
    
    GreenYtocamera = 150 - ptwGreen(2);
    GreenXtocamera = 100 - ptwGreen(1);
    RedYtocamera = 150 - ptwRed(2);
    RedXtocamera = 100 - ptwRed(1);
    YellowYtocamera = 150 - ptwYellow(2);
    YellowXtocamera = 100 - ptwYellow(1);
    OrangeYtocamera = 150 - ptwOrange(2);
    OrangeXtocamera = 100 - ptwOrange(1);
    BlueYtocamera = 150 - ptwBlue(2);
    BlueXtocamera = 100 - ptwBlue(1);   
    
    %?????????????????????????????????????????????????????????????????????????????????
    GreenRealY = ((12/317.5)*(GreenYtocamera))+ptwGreen(2);
    GreenRealX = ((12/317.5)*(GreenXtocamera))+ptwGreen(1);
    RedRealY = ((12/317.5)*(RedYtocamera))+ptwRed(2);
    RedRealX = ((12/317.5)*(RedXtocamera))+ptwRed(1);
    YellowRealY = ((12/317.5)*(YellowYtocamera))+ptwYellow(2);
    YellowRealX = ((12/317.5)*(YellowXtocamera))+ptwYellow(1);
    OrangeRealY = ((12/317.5)*(OrangeYtocamera))+ptwOrange(2);
    OrangeRealX = ((12/317.5)*(OrangeXtocamera))+ptwOrange(1);
    BlueRealY = ((23/317.5)*(GreenYtocamera))+ptwGreen(2);
    BlueRealX = ((23/317.5)*(GreenXtocamera))+ptwGreen(1);
    
    %Green
    Green_Checker_Vector = [ptwGreen(1);ptwGreen(2); 0; 1];
    Green_coord_in_robotFrame = (T_0_checker)*(Green_Checker_Vector);
    %Red
    Red_Checker_Vector = [ptwRed(1);ptwRed(2); 0; 1];
    Red_coord_in_robotFrame = (T_0_checker)*(Red_Checker_Vector);
    %Yellow
    Yellow_Checker_Vector = [ptwYellow(1);ptwYellow(2); 0; 1];
    Yellow_coord_in_robotFrame = (T_0_checker)*(Yellow_Checker_Vector);
    %Orange
    Orange_Checker_Vector = [ptwOrange(1);ptwOrange(2); 0; 1];
    Orange_coord_in_robotFrame = (T_0_checker)*(Orange_Checker_Vector);
    %Blue
    Blue_Checker_Vector = [ptwBlue(1);ptwBlue(2); 0; 1];
    Blue_coord_in_robotFrame = (T_0_checker)*(Blue_Checker_Vector);
%% step 5
    %Uses inverse kinematics from previous labs to figurcle out the joint angles
    %required for the robot’s end effector to reach the object’s location.

        %========================================================================================
        %ik for The point straight above green ball
        Green_above = robot.ik3001([Green_coord_in_robotFrame(1);Green_coord_in_robotFrame(2);100]);
        %ik for Green ball position
        Green_joint = robot.ik3001([Green_coord_in_robotFrame(1);Green_coord_in_robotFrame(2);15]);
        %===================================================================
        %ik for The point straight above Red ball
        Red_above = robot.ik3001([Red_coord_in_robotFrame(1);Red_coord_in_robotFrame(2);100]);
        %ik for Red ball position
        Red_joint = robot.ik3001([Red_coord_in_robotFrame(1);Red_coord_in_robotFrame(2);15]);
        %===================================================================
        %ik for The point straight above Yellow ball
        Yellow_above = robot.ik3001([Yellow_coord_in_robotFrame(1);Yellow_coord_in_robotFrame(2);100]);
        %ik for Yellow ball position
        Yellow_joint = robot.ik3001([Yellow_coord_in_robotFrame(1);Yellow_coord_in_robotFrame(2);15]);
        %===================================================================
        %ik for The point straight above Orange ball
        Orange_above = robot.ik3001([Orange_coord_in_robotFrame(1);Orange_coord_in_robotFrame(2);100]);
        %ik for Orange ball position
        Orange_joint = robot.ik3001([Orange_coord_in_robotFrame(1);Orange_coord_in_robotFrame(2);15]);
        %========================================================================================
        %ik for The point straight above Blue ball
        Blue_above = robot.ik3001([Blue_coord_in_robotFrame(1);Blue_coord_in_robotFrame(2);100]);
        %ik for Blue ball position
        Blue_joint = robot.ik3001([Blue_coord_in_robotFrame(1);Blue_coord_in_robotFrame(2);23]);
        
        
        
        Green_Final_joint = robot.ik3001(Green_Position);
        Green_Final_Above_joint = robot.ik3001(Green_Position_Above);
        Red_Final_joint = robot.ik3001(Red_Position);
        Red_Final_Above_joint = robot.ik3001(Red_Position_Above);
        Yellow_Final_joint = robot.ik3001(Yellow_Position);
        Yellow_Final_Above_joint = robot.ik3001(Yellow_Position_Above);
        Orange_Final_joint = robot.ik3001(Orange_Position);
        Orange_Final_Above_joint = robot.ik3001(Orange_Position_Above);
        Blue_Final_joint = robot.ik3001(Blue_Position);
        Blue_Final_Above_joint = robot.ik3001(Blue_Position_Above);

        
        if(~((50>Green_coord_in_robotFrame(1)&&Green_coord_in_robotFrame(1)>-25)&&(100<=Green_coord_in_robotFrame(2)&&Green_coord_in_robotFrame(2)<=150)))
        %if(GreenRealY<100)
        %traj plan from zero position to point above green ball
        trajZeroToGreenAboveJoint1 = traj.cubic_traj(0,2,0,0,0,Green_above(1)-8);
        trajZeroToGreenAboveJoint2 = traj.cubic_traj(0,2,0,0,0,Green_above(2));
        trajZeroToGreenAboveJoint3 = traj.cubic_traj(0,2,0,0,0,Green_above(3));
        %traj plan from point above green ball to green ball
        trajGreenAboveToGreenJoint1 = traj.cubic_traj(0,2,0,0,Green_above(1)-8,Green_joint(1)-8);
        trajGreenAboveToGreenJoint2 = traj.cubic_traj(0,2,0,0,Green_above(2),Green_joint(2));
        trajGreenAboveToGreenJoint3 = traj.cubic_traj(0,2,0,0,Green_above(3),Green_joint(3)-3);
        %traj plan from green ball to point above green ball
        trajGreenToGreenAboveJoint1 = traj.cubic_traj(0,2,0,0,Green_joint(1)-8,Green_above(1));
        trajGreenToGreenAboveJoint2 = traj.cubic_traj(0,2,0,0,Green_joint(2),Green_above(2));
        trajGreenToGreenAboveJoint3 = traj.cubic_traj(0,2,0,0,Green_joint(3)-3,Green_above(3));
        %traj plan from point above green ball to point above arbituary
        trajGreenAboveToArbiAboveJoint1 = traj.cubic_traj(0,2,0,0,Green_above(1),Green_Final_Above_joint(1));
        trajGreenAboveToArbiAboveJoint2 = traj.cubic_traj(0,2,0,0,Green_above(2),Green_Final_Above_joint(2));
        trajGreenAboveToArbiAboveJoint3 = traj.cubic_traj(0,2,0,0,Green_above(3),Green_Final_Above_joint(3));
        %traj plan from point above arbituary to arbituary point
        trajArbiAboveToGreenArbiJoint1 = traj.cubic_traj(0,2,0,0,Green_Final_Above_joint(1),Green_Final_joint(1));
        trajArbiAboveToGreenArbiJoint2 = traj.cubic_traj(0,2,0,0,Green_Final_Above_joint(2),Green_Final_joint(2));
        trajArbiAboveToGreenArbiJoint3 = traj.cubic_traj(0,2,0,0,Green_Final_Above_joint(3),Green_Final_joint(3));  
        %traj plan from arbituary above arbituary
        trajGreenArbiToArbiAbove1 = traj.cubic_traj(0,2,0,0,Green_Final_joint(1),Green_Final_Above_joint(1));
        trajGreenArbiToArbiAbove2 = traj.cubic_traj(0,2,0,0,Green_Final_joint(2),Green_Final_Above_joint(2));
        trajGreenArbiToArbiAbove3 = traj.cubic_traj(0,2,0,0,Green_Final_joint(3),Green_Final_Above_joint(3));  
        %traj plan from above arbituary to zero
        trajGreenArbiToZero1 = traj.cubic_traj(0,2,0,0,Green_Final_Above_joint(1),0);
        trajGreenArbiToZero2 = traj.cubic_traj(0,2,0,0,Green_Final_Above_joint(2),0);
        trajGreenArbiToZero3 = traj.cubic_traj(0,2,0,0,Green_Final_Above_joint(3),0);  
        
        tic
        while toc < 4.3
            if toc <= 2.1
                t = toc;
                Joint = [traj.cubic_traj_app(trajZeroToGreenAboveJoint1,t) traj.cubic_traj_app(trajZeroToGreenAboveJoint2,t) traj.cubic_traj_app(trajZeroToGreenAboveJoint3,t)];
            end
            
            if toc > 2.1 && toc <= 4.2
                t = toc - 2.1;
                Joint = [traj.cubic_traj_app(trajGreenAboveToGreenJoint1,t) traj.cubic_traj_app(trajGreenAboveToGreenJoint2,t) traj.cubic_traj_app(trajGreenAboveToGreenJoint3,t)];
            end
            
            robot.servo_jp(Joint);
        end
        robot.closeGripper();
        tic
        while toc < 6.4
            %green to green above
            if toc <= 2.1
                t = toc;
                Joint = [traj.cubic_traj_app(trajGreenToGreenAboveJoint1,t) traj.cubic_traj_app(trajGreenToGreenAboveJoint2,t) traj.cubic_traj_app(trajGreenToGreenAboveJoint3,t)];
            end
            
            %green above to arbi above
            if toc > 2.1 && toc <= 4.2
                t = toc - 2.1;
                Joint = [traj.cubic_traj_app(trajGreenAboveToArbiAboveJoint1,t) traj.cubic_traj_app(trajGreenAboveToArbiAboveJoint2,t) traj.cubic_traj_app(trajGreenAboveToArbiAboveJoint3,t)];
            end
            
            %arbi above to arbi
            if toc > 4.2 && toc <= 6.3
                t = toc - 4.2;
                Joint = [traj.cubic_traj_app(trajArbiAboveToGreenArbiJoint1,t) traj.cubic_traj_app(trajArbiAboveToGreenArbiJoint2,t) traj.cubic_traj_app(trajArbiAboveToGreenArbiJoint3,t)];
            end
            robot.servo_jp(Joint);
        end
        
        robot.openGripper();
        
        tic
        while toc < 4.2
            if toc < 2.1
                t = toc;
                Joint = [traj.cubic_traj_app(trajGreenArbiToArbiAbove1,t) traj.cubic_traj_app(trajGreenArbiToArbiAbove2,t) traj.cubic_traj_app(trajGreenArbiToArbiAbove3,t)];
            end
            if toc > 2.1 && toc <= 4.2
                t = toc - 2.1;
                Joint = [traj.cubic_traj_app(trajGreenArbiToZero1,t) traj.cubic_traj_app(trajGreenArbiToZero2,t) traj.cubic_traj_app(trajGreenArbiToZero3,t)];
            end
            robot.servo_jp(Joint);
        end

        end
        
  %===============================================================================
  
        if(~((50<Red_coord_in_robotFrame(1)&&Red_coord_in_robotFrame(1)<100) && (100<Red_coord_in_robotFrame(2)&&Red_coord_in_robotFrame(2)<150)))
        %if(RedRealY<100)
   %traj plan from zero position to point above Red ball
        trajZeroToRedAboveJoint1 = traj.cubic_traj(0,2,0,0,0,Red_above(1)-8);
        trajZeroToRedAboveJoint2 = traj.cubic_traj(0,2,0,0,0,Red_above(2));
        trajZeroToRedAboveJoint3 = traj.cubic_traj(0,2,0,0,0,Red_above(3));
        %traj plan from point above Red ball to Red ball
        trajRedAboveToRedJoint1 = traj.cubic_traj(0,2,0,0,Red_above(1)-8,Red_joint(1)-8);
        trajRedAboveToRedJoint2 = traj.cubic_traj(0,2,0,0,Red_above(2),Red_joint(2));
        trajRedAboveToRedJoint3 = traj.cubic_traj(0,2,0,0,Red_above(3),Red_joint(3)-3);
        %traj plan from Red ball to point above Red ball
        trajRedToRedAboveJoint1 = traj.cubic_traj(0,2,0,0,Red_joint(1)-8,Red_above(1));
        trajRedToRedAboveJoint2 = traj.cubic_traj(0,2,0,0,Red_joint(2),Red_above(2));
        trajRedToRedAboveJoint3 = traj.cubic_traj(0,2,0,0,Red_joint(3)-3,Red_above(3));
        %traj plan from point above Red ball to point above arbituary
        trajRedAboveToArbiAboveJoint1 = traj.cubic_traj(0,2,0,0,Red_above(1),Red_Final_Above_joint(1));
        trajRedAboveToArbiAboveJoint2 = traj.cubic_traj(0,2,0,0,Red_above(2),Red_Final_Above_joint(2));
        trajRedAboveToArbiAboveJoint3 = traj.cubic_traj(0,2,0,0,Red_above(3),Red_Final_Above_joint(3));
        %traj plan from point above arbituary to arbituary point
        trajArbiAboveToRedArbiJoint1 = traj.cubic_traj(0,2,0,0,Red_Final_Above_joint(1),Red_Final_joint(1));
        trajArbiAboveToRedArbiJoint2 = traj.cubic_traj(0,2,0,0,Red_Final_Above_joint(2),Red_Final_joint(2));
        trajArbiAboveToRedArbiJoint3 = traj.cubic_traj(0,2,0,0,Red_Final_Above_joint(3),Red_Final_joint(3));  
        %traj plan from arbituary above arbituary
        trajRedArbiToArbiAbove1 = traj.cubic_traj(0,2,0,0,Red_Final_joint(1),Red_Final_Above_joint(1));
        trajRedArbiToArbiAbove2 = traj.cubic_traj(0,2,0,0,Red_Final_joint(2),Red_Final_Above_joint(2));
        trajRedArbiToArbiAbove3 = traj.cubic_traj(0,2,0,0,Red_Final_joint(3),Red_Final_Above_joint(3));  
        %traj plan from above arbituary to zero
        trajRedArbiToZero1 = traj.cubic_traj(0,2,0,0,Red_Final_Above_joint(1),0);
        trajRedArbiToZero2 = traj.cubic_traj(0,2,0,0,Red_Final_Above_joint(2),0);
        trajRedArbiToZero3 = traj.cubic_traj(0,2,0,0,Red_Final_Above_joint(3),0);  
        
        tic
        while toc < 4.3
            if toc <= 2.1
            t = toc;
            Joint = [traj.cubic_traj_app(trajZeroToRedAboveJoint1,t) traj.cubic_traj_app(trajZeroToRedAboveJoint2,t) traj.cubic_traj_app(trajZeroToRedAboveJoint3,t)];
            end
            
            if toc > 2.1 && toc <= 4.2
            t = toc - 2.1;
            Joint = [traj.cubic_traj_app(trajRedAboveToRedJoint1,t) traj.cubic_traj_app(trajRedAboveToRedJoint2,t) traj.cubic_traj_app(trajRedAboveToRedJoint3,t)];
            end
            
            robot.servo_jp(Joint);
        end
        robot.closeGripper();
        tic
        while toc < 6.4
            %Red to Red above
            if toc <= 2.1
            t = toc;
            Joint = [traj.cubic_traj_app(trajRedToRedAboveJoint1,t) traj.cubic_traj_app(trajRedToRedAboveJoint2,t) traj.cubic_traj_app(trajRedToRedAboveJoint3,t)];
            end
            %Red above to arbi above
            if toc > 2.1 && toc <= 4.2
            t = toc - 2.1;
            Joint = [traj.cubic_traj_app(trajRedAboveToArbiAboveJoint1,t) traj.cubic_traj_app(trajRedAboveToArbiAboveJoint2,t) traj.cubic_traj_app(trajRedAboveToArbiAboveJoint3,t)];
            end
            %arbi above to arbi
            if toc > 4.2 && toc <= 6.3
            t = toc - 4.2;
            Joint = [traj.cubic_traj_app(trajArbiAboveToRedArbiJoint1,t) traj.cubic_traj_app(trajArbiAboveToRedArbiJoint2,t) traj.cubic_traj_app(trajArbiAboveToRedArbiJoint3,t)];
            end
            robot.servo_jp(Joint);
        end
        robot.openGripper();
        
        tic
        while toc < 4.2
            if toc < 2.1
                t = toc;
                Joint = [traj.cubic_traj_app(trajRedArbiToArbiAbove1,t) traj.cubic_traj_app(trajRedArbiToArbiAbove2,t) traj.cubic_traj_app(trajRedArbiToArbiAbove3,t)];
            end
            if toc > 2.1 && toc <= 4.2
                t = toc - 2.1;
                Joint = [traj.cubic_traj_app(trajRedArbiToZero1,t) traj.cubic_traj_app(trajRedArbiToZero2,t) traj.cubic_traj_app(trajRedArbiToZero3,t)];
            end
            robot.servo_jp(Joint);
        end
        end
 %===============================================================================
        if(~((-25<Yellow_coord_in_robotFrame(1)&&Yellow_coord_in_robotFrame(1)<50)&&(-85>Yellow_coord_in_robotFrame(2)&&Yellow_coord_in_robotFrame(2)>-160)))
        %if(YellowRealY>-100)
   %traj plan from zero position to point above Yellow ball
        trajZeroToYellowAboveJoint1 = traj.cubic_traj(0,2,0,0,0,Yellow_above(1)-8);
        trajZeroToYellowAboveJoint2 = traj.cubic_traj(0,2,0,0,0,Yellow_above(2));
        trajZeroToYellowAboveJoint3 = traj.cubic_traj(0,2,0,0,0,Yellow_above(3));
        %traj plan from point above Yellow ball to Yellow ball
        trajYellowAboveToYellowJoint1 = traj.cubic_traj(0,2,0,0,Yellow_above(1)-8,Yellow_joint(1)-8);
        trajYellowAboveToYellowJoint2 = traj.cubic_traj(0,2,0,0,Yellow_above(2),Yellow_joint(2));
        trajYellowAboveToYellowJoint3 = traj.cubic_traj(0,2,0,0,Yellow_above(3),Yellow_joint(3)-3);
        %traj plan from Yellow ball to point above Yellow ball
        trajYellowToYellowAboveJoint1 = traj.cubic_traj(0,2,0,0,Yellow_joint(1)-8,Yellow_above(1));
        trajYellowToYellowAboveJoint2 = traj.cubic_traj(0,2,0,0,Yellow_joint(2),Yellow_above(2));
        trajYellowToYellowAboveJoint3 = traj.cubic_traj(0,2,0,0,Yellow_joint(3)-3,Yellow_above(3));
        %traj plan from point above Yellow ball to point above arbituary
        trajYellowAboveToArbiAboveJoint1 = traj.cubic_traj(0,2,0,0,Yellow_above(1),Yellow_Final_Above_joint(1));
        trajYellowAboveToArbiAboveJoint2 = traj.cubic_traj(0,2,0,0,Yellow_above(2),Yellow_Final_Above_joint(2));
        trajYellowAboveToArbiAboveJoint3 = traj.cubic_traj(0,2,0,0,Yellow_above(3),Yellow_Final_Above_joint(3));
        %traj plan from point above arbituary to arbituary point
        trajArbiAboveToYellowArbiJoint1 = traj.cubic_traj(0,2,0,0,Yellow_Final_Above_joint(1),Yellow_Final_joint(1));
        trajArbiAboveToYellowArbiJoint2 = traj.cubic_traj(0,2,0,0,Yellow_Final_Above_joint(2),Yellow_Final_joint(2));
        trajArbiAboveToYellowArbiJoint3 = traj.cubic_traj(0,2,0,0,Yellow_Final_Above_joint(3),Yellow_Final_joint(3));  
        %traj plan from arbituary above arbituary
        trajYellowArbiToArbiAbove1 = traj.cubic_traj(0,2,0,0,Yellow_Final_joint(1),Yellow_Final_Above_joint(1));
        trajYellowArbiToArbiAbove2 = traj.cubic_traj(0,2,0,0,Yellow_Final_joint(2),Yellow_Final_Above_joint(2));
        trajYellowArbiToArbiAbove3 = traj.cubic_traj(0,2,0,0,Yellow_Final_joint(3),Yellow_Final_Above_joint(3));  
        %traj plan from above arbituary to zero
        trajYellowArbiToZero1 = traj.cubic_traj(0,2,0,0,Yellow_Final_Above_joint(1),0);
        trajYellowArbiToZero2 = traj.cubic_traj(0,2,0,0,Yellow_Final_Above_joint(2),0);
        trajYellowArbiToZero3 = traj.cubic_traj(0,2,0,0,Yellow_Final_Above_joint(3),0);  
        tic
        while toc < 4.3
            if toc <= 2.1
            t = toc;
            Joint = [traj.cubic_traj_app(trajZeroToYellowAboveJoint1,t) traj.cubic_traj_app(trajZeroToYellowAboveJoint2,t) traj.cubic_traj_app(trajZeroToYellowAboveJoint3,t)];
            end
            
            if toc > 2.1 && toc <= 4.2
            t = toc - 2.1;
            Joint = [traj.cubic_traj_app(trajYellowAboveToYellowJoint1,t) traj.cubic_traj_app(trajYellowAboveToYellowJoint2,t) traj.cubic_traj_app(trajYellowAboveToYellowJoint3,t)];
            end
            
            robot.servo_jp(Joint);
        end
        robot.closeGripper();
        tic
        while toc < 6.4
            %Yellow to Yellow above
            if toc <= 2.1
            t = toc;
            Joint = [traj.cubic_traj_app(trajYellowToYellowAboveJoint1,t) traj.cubic_traj_app(trajYellowToYellowAboveJoint2,t) traj.cubic_traj_app(trajYellowToYellowAboveJoint3,t)];
            end
            %Yellow above to arbi above
            if toc > 2.1 && toc <= 4.2
            t = toc - 2.1;
            Joint = [traj.cubic_traj_app(trajYellowAboveToArbiAboveJoint1,t) traj.cubic_traj_app(trajYellowAboveToArbiAboveJoint2,t) traj.cubic_traj_app(trajYellowAboveToArbiAboveJoint3,t)];
            end
            %arbi above to arbi
            if toc > 4.2 && toc <= 6.3
            t = toc - 4.2;
            Joint = [traj.cubic_traj_app(trajArbiAboveToYellowArbiJoint1,t) traj.cubic_traj_app(trajArbiAboveToYellowArbiJoint2,t) traj.cubic_traj_app(trajArbiAboveToYellowArbiJoint3,t)];
            end
            robot.servo_jp(Joint);
        end
        robot.openGripper();
            
        tic
        while toc < 4.2
            if toc < 2.1
                t = toc;
                Joint = [traj.cubic_traj_app(trajYellowArbiToArbiAbove1,t) traj.cubic_traj_app(trajYellowArbiToArbiAbove2,t) traj.cubic_traj_app(trajYellowArbiToArbiAbove3,t)];
            end
            if toc > 2.1 && toc <= 4.2
                t = toc - 2.1;
                Joint = [traj.cubic_traj_app(trajYellowArbiToZero1,t) traj.cubic_traj_app(trajYellowArbiToZero2,t) traj.cubic_traj_app(trajYellowArbiToZero3,t)];
            end
            robot.servo_jp(Joint);
        end
        end
   %===============================================================================
        if(~((50<Orange_coord_in_robotFrame(1)&&Orange_coord_in_robotFrame(1)<100) && (-85>Orange_coord_in_robotFrame(2)&&Orange_coord_in_robotFrame(2)>-150)))
        %if(OrangeRealY>-100)
   %traj plan from zero position to point above Orange ball
        trajZeroToOrangeAboveJoint1 = traj.cubic_traj(0,2,0,0,0,Orange_above(1)-8);
        trajZeroToOrangeAboveJoint2 = traj.cubic_traj(0,2,0,0,0,Orange_above(2));
        trajZeroToOrangeAboveJoint3 = traj.cubic_traj(0,2,0,0,0,Orange_above(3));
        %traj plan from point above Orange ball to Orange ball
        trajOrangeAboveToOrangeJoint1 = traj.cubic_traj(0,2,0,0,Orange_above(1)-8,Orange_joint(1)-8);
        trajOrangeAboveToOrangeJoint2 = traj.cubic_traj(0,2,0,0,Orange_above(2),Orange_joint(2));
        trajOrangeAboveToOrangeJoint3 = traj.cubic_traj(0,2,0,0,Orange_above(3),Orange_joint(3)-3);
        %traj plan from Orange ball to point above Orange ball
        trajOrangeToOrangeAboveJoint1 = traj.cubic_traj(0,2,0,0,Orange_joint(1)-8,Orange_above(1));
        trajOrangeToOrangeAboveJoint2 = traj.cubic_traj(0,2,0,0,Orange_joint(2),Orange_above(2));
        trajOrangeToOrangeAboveJoint3 = traj.cubic_traj(0,2,0,0,Orange_joint(3)-3,Orange_above(3));
        %traj plan from point above Orange ball to point above arbituary
        trajOrangeAboveToArbiAboveJoint1 = traj.cubic_traj(0,2,0,0,Orange_above(1),Orange_Final_Above_joint(1));
        trajOrangeAboveToArbiAboveJoint2 = traj.cubic_traj(0,2,0,0,Orange_above(2),Orange_Final_Above_joint(2));
        trajOrangeAboveToArbiAboveJoint3 = traj.cubic_traj(0,2,0,0,Orange_above(3),Orange_Final_Above_joint(3));
        %traj plan from point above arbituary to arbituary point
        trajArbiAboveToOrangeArbiJoint1 = traj.cubic_traj(0,2,0,0,Orange_Final_Above_joint(1),Orange_Final_joint(1));
        trajArbiAboveToOrangeArbiJoint2 = traj.cubic_traj(0,2,0,0,Orange_Final_Above_joint(2),Orange_Final_joint(2));
        trajArbiAboveToOrangeArbiJoint3 = traj.cubic_traj(0,2,0,0,Orange_Final_Above_joint(3),Orange_Final_joint(3));  
        %traj plan from arbituary above arbituary
        trajOrangeArbiToArbiAbove1 = traj.cubic_traj(0,2,0,0,Orange_Final_joint(1),Orange_Final_Above_joint(1));
        trajOrangeArbiToArbiAbove2 = traj.cubic_traj(0,2,0,0,Orange_Final_joint(2),Orange_Final_Above_joint(2));
        trajOrangeArbiToArbiAbove3 = traj.cubic_traj(0,2,0,0,Orange_Final_joint(3),Orange_Final_Above_joint(3));  
        %traj plan from above arbituary to zero
        trajOrangeArbiToZero1 = traj.cubic_traj(0,2,0,0,Orange_Final_Above_joint(1),0);
        trajOrangeArbiToZero2 = traj.cubic_traj(0,2,0,0,Orange_Final_Above_joint(2),0);
        trajOrangeArbiToZero3 = traj.cubic_traj(0,2,0,0,Orange_Final_Above_joint(3),0);  
        
        tic
        while toc < 4.3
            if toc <= 2.1
            t = toc;
            Joint = [traj.cubic_traj_app(trajZeroToOrangeAboveJoint1,t) traj.cubic_traj_app(trajZeroToOrangeAboveJoint2,t) traj.cubic_traj_app(trajZeroToOrangeAboveJoint3,t)];
            end
            
            if toc > 2.1 && toc <= 4.2
            t = toc - 2.1;
            Joint = [traj.cubic_traj_app(trajOrangeAboveToOrangeJoint1,t) traj.cubic_traj_app(trajOrangeAboveToOrangeJoint2,t) traj.cubic_traj_app(trajOrangeAboveToOrangeJoint3,t)];
            end
            
            robot.servo_jp(Joint);
        end
        robot.closeGripper();
        tic
        while toc < 6.4
            %Orange to Orange above
            if toc <= 2.1
            t = toc;
            Joint = [traj.cubic_traj_app(trajOrangeToOrangeAboveJoint1,t) traj.cubic_traj_app(trajOrangeToOrangeAboveJoint2,t) traj.cubic_traj_app(trajOrangeToOrangeAboveJoint3,t)];
            end
            %Orange above to arbi above
            if toc > 2.1 && toc <= 4.2
            t = toc - 2.1;
            Joint = [traj.cubic_traj_app(trajOrangeAboveToArbiAboveJoint1,t) traj.cubic_traj_app(trajOrangeAboveToArbiAboveJoint2,t) traj.cubic_traj_app(trajOrangeAboveToArbiAboveJoint3,t)];
            end
            %arbi above to arbi
            if toc > 4.2 && toc <= 6.3
            t = toc - 4.2;
            Joint = [traj.cubic_traj_app(trajArbiAboveToOrangeArbiJoint1,t) traj.cubic_traj_app(trajArbiAboveToOrangeArbiJoint2,t) traj.cubic_traj_app(trajArbiAboveToOrangeArbiJoint3,t)];
            end
            robot.servo_jp(Joint);
        end
        robot.openGripper();
        
        tic
        while toc < 4.2
            if toc < 2.1
                t = toc;
                Joint = [traj.cubic_traj_app(trajOrangeArbiToArbiAbove1,t) traj.cubic_traj_app(trajOrangeArbiToArbiAbove2,t) traj.cubic_traj_app(trajOrangeArbiToArbiAbove3,t)];
            end
            if toc > 2.1 && toc <= 4.2
                t = toc - 2.1;
                Joint = [traj.cubic_traj_app(trajOrangeArbiToZero1,t) traj.cubic_traj_app(trajOrangeArbiToZero2,t) traj.cubic_traj_app(trajOrangeArbiToZero3,t)];
            end
            robot.servo_jp(Joint);
        end
        end
        
        
        if(~((80<Blue_coord_in_robotFrame(1)&&Blue_coord_in_robotFrame(1)<150)&&(90<Blue_coord_in_robotFrame(2)&&Blue_coord_in_robotFrame(2)<150)))
        %if(BlueRealY<100)
        %traj plan from zero position to point above Blue ball
        trajZeroToBlueAboveJoint1 = traj.cubic_traj(0,2,0,0,0,Blue_above(1)-8);
        trajZeroToBlueAboveJoint2 = traj.cubic_traj(0,2,0,0,0,Blue_above(2));
        trajZeroToBlueAboveJoint3 = traj.cubic_traj(0,2,0,0,0,Blue_above(3));
        %traj plan from point above Blue ball to Blue ball
        trajBlueAboveToBlueJoint1 = traj.cubic_traj(0,2,0,0,Blue_above(1)-8,Blue_joint(1)-10);
        trajBlueAboveToBlueJoint2 = traj.cubic_traj(0,2,0,0,Blue_above(2),Blue_joint(2));
        trajBlueAboveToBlueJoint3 = traj.cubic_traj(0,2,0,0,Blue_above(3),Blue_joint(3)-3);
        %traj plan from Blue ball to point above Blue ball
        trajBlueToBlueAboveJoint1 = traj.cubic_traj(0,2,0,0,Blue_joint(1)-10,Blue_above(1));
        trajBlueToBlueAboveJoint2 = traj.cubic_traj(0,2,0,0,Blue_joint(2),Blue_above(2));
        trajBlueToBlueAboveJoint3 = traj.cubic_traj(0,2,0,0,Blue_joint(3)-3,Blue_above(3));
        %traj plan from point above Blue ball to point above arbituary
        trajBlueAboveToArbiAboveJoint1 = traj.cubic_traj(0,2,0,0,Blue_above(1),Blue_Final_Above_joint(1));
        trajBlueAboveToArbiAboveJoint2 = traj.cubic_traj(0,2,0,0,Blue_above(2),Blue_Final_Above_joint(2));
        trajBlueAboveToArbiAboveJoint3 = traj.cubic_traj(0,2,0,0,Blue_above(3),Blue_Final_Above_joint(3));
        %traj plan from point above arbituary to arbituary point
        trajArbiAboveToBlueArbiJoint1 = traj.cubic_traj(0,2,0,0,Blue_Final_Above_joint(1),Blue_Final_joint(1));
        trajArbiAboveToBlueArbiJoint2 = traj.cubic_traj(0,2,0,0,Blue_Final_Above_joint(2),Blue_Final_joint(2));
        trajArbiAboveToBlueArbiJoint3 = traj.cubic_traj(0,2,0,0,Blue_Final_Above_joint(3),Blue_Final_joint(3));  
        %traj plan from arbituary above arbituary
        trajBlueArbiToArbiAbove1 = traj.cubic_traj(0,2,0,0,Blue_Final_joint(1),Blue_Final_Above_joint(1));
        trajBlueArbiToArbiAbove2 = traj.cubic_traj(0,2,0,0,Blue_Final_joint(2),Blue_Final_Above_joint(2));
        trajBlueArbiToArbiAbove3 = traj.cubic_traj(0,2,0,0,Blue_Final_joint(3),Blue_Final_Above_joint(3));  
        %traj plan from above arbituary to zero
        trajBlueArbiToZero1 = traj.cubic_traj(0,2,0,0,Blue_Final_Above_joint(1),0);
        trajBlueArbiToZero2 = traj.cubic_traj(0,2,0,0,Blue_Final_Above_joint(2),0);
        trajBlueArbiToZero3 = traj.cubic_traj(0,2,0,0,Blue_Final_Above_joint(3),0);  
        
        tic
        while toc < 4.3
            if toc <= 2.1
                t = toc;
                Joint = [traj.cubic_traj_app(trajZeroToBlueAboveJoint1,t) traj.cubic_traj_app(trajZeroToBlueAboveJoint2,t) traj.cubic_traj_app(trajZeroToBlueAboveJoint3,t)];
            end
            
            if toc > 2.1 && toc <= 4.2
                t = toc - 2.1;
                Joint = [traj.cubic_traj_app(trajBlueAboveToBlueJoint1,t) traj.cubic_traj_app(trajBlueAboveToBlueJoint2,t) traj.cubic_traj_app(trajBlueAboveToBlueJoint3,t)];
            end
            
            robot.servo_jp(Joint);
        end
        robot.closeGripper();
        tic
        while toc < 6.4
            %Blue to Blue above
            if toc <= 2.1
                t = toc;
                Joint = [traj.cubic_traj_app(trajBlueToBlueAboveJoint1,t) traj.cubic_traj_app(trajBlueToBlueAboveJoint2,t) traj.cubic_traj_app(trajBlueToBlueAboveJoint3,t)];
            end
            
            %Blue above to arbi above
            if toc > 2.1 && toc <= 4.2
                t = toc - 2.1;
                Joint = [traj.cubic_traj_app(trajBlueAboveToArbiAboveJoint1,t) traj.cubic_traj_app(trajBlueAboveToArbiAboveJoint2,t) traj.cubic_traj_app(trajBlueAboveToArbiAboveJoint3,t)];
            end
            
            %arbi above to arbi
            if toc > 4.2 && toc <= 6.3
                t = toc - 4.2;
                Joint = [traj.cubic_traj_app(trajArbiAboveToBlueArbiJoint1,t) traj.cubic_traj_app(trajArbiAboveToBlueArbiJoint2,t) traj.cubic_traj_app(trajArbiAboveToBlueArbiJoint3,t)];
            end
            robot.servo_jp(Joint);
        end
        
        robot.openGripper();
        
        tic
        while toc < 4.2
            if toc < 2.1
                t = toc;
                Joint = [traj.cubic_traj_app(trajBlueArbiToArbiAbove1,t) traj.cubic_traj_app(trajBlueArbiToArbiAbove2,t) traj.cubic_traj_app(trajBlueArbiToArbiAbove3,t)];
            end
            if toc > 2.1 && toc <= 4.2
                t = toc - 2.1;
                Joint = [traj.cubic_traj_app(trajBlueArbiToZero1,t) traj.cubic_traj_app(trajBlueArbiToZero2,t) traj.cubic_traj_app(trajBlueArbiToZero3,t)];
            end
            robot.servo_jp(Joint);
        end

        end
        robot.servo_jp([0,0,-90]);
        pause(3);
end
%% Shutdown Procedure
robot.shutdown()
cam.shutdown()


%imshow(redAvg);