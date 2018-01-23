close all;clc;clear all;

% User defined parameters
angleDegrees = input('Enter angle in degrees : ');   % Angle from the verticle (for the bot's motion)
velocity = input('Enter velocity magnitude : ');       % Velocity of motion

angle = angleDegrees * pi / 180;
transformationMatrix = [
    -1 cos(deg2rad(60)) cos(deg2rad(60))
    0 -sin(deg2rad(60)) sin(deg2rad(60))
    1 1 1
    ];  %The transformation/coeficient matrix of the system of equations
inverseTransformationMatrix = inv(transformationMatrix);
RHSVelocityMatrix = [
    velocity * sin(angle)
    velocity * cos(angle)
    0
    ];
resultantThreeWheelVelocityMagnitudes = transformationMatrix \ RHSVelocityMatrix;
vel1 = resultantThreeWheelVelocityMagnitudes(1);
vel2 = resultantThreeWheelVelocityMagnitudes(2);
vel3 = resultantThreeWheelVelocityMagnitudes(3);
aimString = strcat('Aim is : Velocity = ' , num2str(velocity) , ', Angle with vertical = ' , num2str(angleDegrees) , ' degrees');
disp(aimString);
disp('Wheel 1 is on top (0 degrees with horizontal)');
disp('Wheel 2 is on bottom left (60 degrees with horizontal)');
disp('Wheel 3 is on bottom right (60 degrees with horizontal)');
disp('FOLLOW RIGHT HAND THUMB RULE');
display(resultantThreeWheelVelocityMagnitudes);
resultantVelocities = resultantThreeWheelVelocityMagnitudes;


