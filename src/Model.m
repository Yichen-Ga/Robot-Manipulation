classdef Model < handle
    %MODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Robot
    end
    
    methods
        function model = Model(target)
            %MODEL Construct an instance of this class
            %  The construtor takes a robot object as input
            model.Robot = target;
        end
        %take a full DH table and generate the intermediate transformation
        %matrix
        function [ Every_Step_FK ] = DHKine3001(self, full_DH_table)
            %calculate T01,T12,T23,T34
            T01 = self.Robot.dh2mat([full_DH_table(1,1),full_DH_table(1,2),full_DH_table(1,3),full_DH_table(1,4)]);
            T12 = self.Robot.dh2mat([full_DH_table(2,1),full_DH_table(2,2),full_DH_table(2,3),full_DH_table(2,4)]);
            T23 = self.Robot.dh2mat([full_DH_table(3,1),full_DH_table(3,2),full_DH_table(3,3),full_DH_table(3,4)]);
            T34 = self.Robot.dh2mat([full_DH_table(4,1),full_DH_table(4,2),full_DH_table(4,3),full_DH_table(4,4)]);
            %multiply the intermediate transformation matrix together and
            %get T01, T02, T03, T04
            T02=T01*T12;
            T03=T02*T23;
            T04=T03*T34;
            %Put them all together to form a 4x16 matrix to store the data
            Every_Step_FK = [T01 T02 T03 T04];
        end
        
        %takes in the 4x16 matrix by DHKine(), and generate every reference
        %frame axis's tip position with respect to the origins.
        function [ Every_Step_Frame ] = Frame3001(self, Every_Step_FK)
            %frame_ab means the the tip of b axis of ath frame with respect
            %to the frame a's origins.
           frame_1x = [40 0 0 1]';
           frame_1y = [0 40 0 1]';
           frame_1z = [0 0 40 1]';
           frame_2x = [40 0 0 1]';
           frame_2y = [0 40 0 1]';
           frame_2z = [0 0 40 1]';
           frame_3x = [40 0 0 1]';
           frame_3y = [0 40 0 1]';
           frame_3z = [0 0 40 1]';
           frame_4x = [40 0 0 1]';
           frame_4y = [0 40 0 1]';
           frame_4z = [0 0 40 1]';
           
           %for example, for x axis of Frame 1, we use T01 multiply by the
           %coordinate of the tip of x axis of 1st frame with respect to the frame a's origins.
           %so that we can get the coordinate of tip of x axis of 1st frame
           %with respect to the frame o's origins
           f1x = Every_Step_FK(:,(1:4)) * frame_1x;
           f1y = Every_Step_FK(:,(1:4)) * frame_1y;
           f1z = Every_Step_FK(:,(1:4)) * frame_1z;
           f2x = Every_Step_FK(:,(5:8)) * frame_2x;
           f2y = Every_Step_FK(:,(5:8)) * frame_2y;
           f2z = Every_Step_FK(:,(5:8)) * frame_2z;
           f3x = Every_Step_FK(:,(9:12)) * frame_3x;
           f3y = Every_Step_FK(:,(9:12)) * frame_3y;
           f3z = Every_Step_FK(:,(9:12)) * frame_3z;
           f4x = Every_Step_FK(:,(13:16)) * frame_4x;
           f4y = Every_Step_FK(:,(13:16)) * frame_4y;
           f4z = Every_Step_FK(:,(13:16)) * frame_4z;
           
           %store all the result in a 4*12 matrix, the first three row
           %represent the xyz coordinate of each column(f1x,f1y.....)
           %which is the tip of each frame axis.
           Every_Step_Frame = [f1x f1y f1z f2x f2y f2z f3x f3y f3z f4x f4y f4z];
        end
        
        %takes in every step of intermediate transformation function and
        %then use the position matrix generated in every intermediate
        %transformation function and
        function q_vector_matix = XYZkine3001(self, Every_Step_FK)
            Q1=[0 Every_Step_FK(1,4) Every_Step_FK(1,8) Every_Step_FK(1,12) Every_Step_FK(1,16)];
            Q2=[0 Every_Step_FK(2,4) Every_Step_FK(2,8) Every_Step_FK(2,12) Every_Step_FK(2,16)];
            Q3=[0 Every_Step_FK(3,4) Every_Step_FK(3,8) Every_Step_FK(3,12) Every_Step_FK(3,16)];
        %generate a 3*5 matrix where every column of the q_vector_matix is the X;Y;Z position of
        %each joint.
            q_vector_matix=[Q1;Q2;Q3];
        end
        
        
        
        %takes in three joint variable
        function plot_arm (self,q)
            %put the joint variable into the DH table
            full_dh_table=[0,55,0,0;
                      q(1,1),40,0,-90;
                      q(2,1)-90,0,100,0;
                      q(3,1)+90,0,100,0;];
            %generate every steps' intermediate transformation matrix
            Every_Step_FK = self.DHKine3001(full_dh_table);
            %generate the matrix for X;Y;Z position of each joint
            q_vector_matix = self.XYZkine3001(Every_Step_FK);
            %generate the matrix for every respective frame's axis tip's X;Y;Z
            %position
            frame_vector = self.Frame3001(Every_Step_FK);
            
            %the coordinate of the tip of frame 0's x/y/z axis and frame 0's origin
            point_0_zframe = [q_vector_matix(1,1) q_vector_matix(1,1);q_vector_matix(2,1) q_vector_matix(2,1);q_vector_matix(3,1) q_vector_matix(3,1)+20;];
            point_0_xframe = [q_vector_matix(1,1) q_vector_matix(1,1)+20;q_vector_matix(2,1) q_vector_matix(2,1);q_vector_matix(3,1) q_vector_matix(3,1);];
            point_0_yframe = [q_vector_matix(1,1) q_vector_matix(1,1);q_vector_matix(2,1) q_vector_matix(2,1)+20;q_vector_matix(3,1) q_vector_matix(3,1);];
            %the coordinate of the tip of frame 1's x/y/z axis and frame 1's origin
            point_1_xframe = [q_vector_matix(1,2) frame_vector(1,1);q_vector_matix(2,2) frame_vector(2,1);q_vector_matix(3,2) frame_vector(3,1);];
            point_1_yframe = [q_vector_matix(1,2) frame_vector(1,2);q_vector_matix(2,2) frame_vector(2,2);q_vector_matix(3,2) frame_vector(3,2);];
            point_1_zframe = [q_vector_matix(1,2) frame_vector(1,3);q_vector_matix(2,2) frame_vector(2,3);q_vector_matix(3,2) frame_vector(3,3);];
            %the coordinate of the tip of frame 2's x/y/z axis and frame 2's origin
            point_2_xframe = [q_vector_matix(1,3) frame_vector(1,4);q_vector_matix(2,3) frame_vector(2,4);q_vector_matix(3,3) frame_vector(3,4);];
            point_2_yframe = [q_vector_matix(1,3) frame_vector(1,5);q_vector_matix(2,3) frame_vector(2,5);q_vector_matix(3,3) frame_vector(3,5);];
            point_2_zframe = [q_vector_matix(1,3) frame_vector(1,6);q_vector_matix(2,3) frame_vector(2,6);q_vector_matix(3,3) frame_vector(3,6);];
            %the coordinate of the tip of frame 3's x/y/z axis and frame 3's origin
            point_3_xframe = [q_vector_matix(1,4) frame_vector(1,7);q_vector_matix(2,4) frame_vector(2,7);q_vector_matix(3,4) frame_vector(3,7);];
            point_3_yframe = [q_vector_matix(1,4) frame_vector(1,8);q_vector_matix(2,4) frame_vector(2,8);q_vector_matix(3,4) frame_vector(3,8);];
            point_3_zframe = [q_vector_matix(1,4) frame_vector(1,9);q_vector_matix(2,4) frame_vector(2,9);q_vector_matix(3,4) frame_vector(3,9);];
            %the coordinate of the tip of frame 4's x/y/z axis and frame 4's origin
            point_4_xframe = [q_vector_matix(1,5) frame_vector(1,10);q_vector_matix(2,5) frame_vector(2,10);q_vector_matix(3,5) frame_vector(3,10);];
            point_4_yframe = [q_vector_matix(1,5) frame_vector(1,11);q_vector_matix(2,5) frame_vector(2,11);q_vector_matix(3,5) frame_vector(3,11);];
            point_4_zframe = [q_vector_matix(1,5) frame_vector(1,12);q_vector_matix(2,5) frame_vector(2,12);q_vector_matix(3,5) frame_vector(3,12);];
            
            %plot the arm!!!!!!!!
            plot3(q_vector_matix(1,:),q_vector_matix(2,:),q_vector_matix(3,:),'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);grid on;%axis([-31,31,-31,31,0,31]);
            hold on
            
            %plot the frame 0's xyz axis
            plot3(point_0_zframe(1,:),point_0_zframe(2,:),point_0_zframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','green');grid on;%axis([-31,31,-31,31,0,31]);
            plot3(point_0_xframe(1,:),point_0_xframe(2,:),point_0_xframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','red');grid on;%axis([-31,31,-31,31,0,31]);
            plot3(point_0_yframe(1,:),point_0_yframe(2,:),point_0_yframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','cyan');grid on;%axis([-31,31,-31,31,0,31]);
            %plot the frame 1's xyz axis
            plot3(point_1_zframe(1,:),point_1_zframe(2,:),point_1_zframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','green');grid on;%axis([-31,31,-31,31,0,31]);
            plot3(point_1_xframe(1,:),point_1_xframe(2,:),point_1_xframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','red');grid on;%axis([-31,31,-31,31,0,31]);
            plot3(point_1_yframe(1,:),point_1_yframe(2,:),point_1_yframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','cyan');grid on;%axis([-31,31,-31,31,0,31]);
            %plot the frame 2's xyz axis
            plot3(point_2_zframe(1,:),point_2_zframe(2,:),point_2_zframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','green');grid on;%axis([-31,31,-31,31,0,31]);
            plot3(point_2_xframe(1,:),point_2_xframe(2,:),point_2_xframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','red');grid on;%axis([-31,31,-31,31,0,31]);
            plot3(point_2_yframe(1,:),point_2_yframe(2,:),point_2_yframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','cyan');grid on;%axis([-31,31,-31,31,0,31]);
            %plot the frame 3's xyz axis
            plot3(point_3_zframe(1,:),point_3_zframe(2,:),point_3_zframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','green');grid on;%axis([-31,31,-31,31,0,31]);
            plot3(point_3_xframe(1,:),point_3_xframe(2,:),point_3_xframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','red');grid on;%axis([-31,31,-31,31,0,31]);
            plot3(point_3_yframe(1,:),point_3_yframe(2,:),point_3_yframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','cyan');grid on;%axis([-31,31,-31,31,0,31]);
            %plot the frame 4's xyz axis
            plot3(point_4_zframe(1,:),point_4_zframe(2,:),point_4_zframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','green');grid on;%axis([-31,31,-31,31,0,31]);
            plot3(point_4_xframe(1,:),point_4_xframe(2,:),point_4_xframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','red');grid on;%axis([-31,31,-31,31,0,31]);
            plot3(point_4_yframe(1,:),point_4_yframe(2,:),point_4_yframe(3,:),'-','LineWidth',3,'MarkerSize',6,'Color','cyan');grid on;%axis([-31,31,-31,31,0,31]);
            
%             %mark the joint's coordinate
%             text(q_vector_matix(1,5),q_vector_matix(2,5),q_vector_matix(3,5),['  (', num2str(q_vector_matix(1,5)), ', ', num2str(q_vector_matix(2,5)),', ', num2str(q_vector_matix(3,5)), ')']);
%             text(q_vector_matix(1,4),q_vector_matix(2,4),q_vector_matix(3,4),['  (', num2str(q_vector_matix(1,4)), ', ', num2str(q_vector_matix(2,4)),', ', num2str(q_vector_matix(3,4)), ')']);
%             text(q_vector_matix(1,3),q_vector_matix(2,3),q_vector_matix(3,3),['  (', num2str(q_vector_matix(1,3)), ', ', num2str(q_vector_matix(2,3)),', ', num2str(q_vector_matix(3,3)), ')']);
%             text(q_vector_matix(1,2),q_vector_matix(2,2),q_vector_matix(3,2),['  (', num2str(q_vector_matix(1,2)), ', ', num2str(q_vector_matix(2,2)),', ', num2str(q_vector_matix(3,2)), ')']);
%             text(q_vector_matix(1,1),q_vector_matix(2,1),q_vector_matix(3,1),['  (', num2str(q_vector_matix(1,1)), ', ', num2str(q_vector_matix(2,1)),', ', num2str(q_vector_matix(3,1)), ')']);

            %add title and lable and assign range
            title('Visual Servo Manipulator Pick-Place Object(VSMP2O)')
            xlabel('X Axis(mm)');
            ylabel('Y Axis(mm)');
            zlabel('Z Axis(mm)');
            
            axis([-300 300 -300 300 0 300]);
            hold off
        end
        %===============================================================================
        

    end
    
end

