clear all
close all

data=csvread('cycloid.csv')
time=data(:,1)
theta=data(:,2)

plot(time,theta,'-o')
xlabel('Time [sec]')
ylabel('\psi + \theta [deg]')
xlim([0 0.05])
ylim([-60 360])
xticks(0:0.005:0.050)
yticks(-60:30:360)
title('Absolute angle of airfoil3 vs Elapsed time')
hold on

grid on

