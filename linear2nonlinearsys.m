% author: Mesih Veysi Kılınç
% date:   11-07-2019
% title:  Linear Controller Simulation of system: m(1+q*q)qddot+bq*qdot=u
clc
clear all

%model parameters
m = 1.5; b = 2;

%gains
kv = 100; kp = 100;

q = 0;  qdot = 0; qddot = 0;
yd = 5; ydot = 0; yddot = 0;
e = yd-q; edot = 0; eddot = 0; 

%for ploting
index = 1;
qp = []; qdotp = []; qddotp = [];
ep = []; edotp = []; eddotp = [];
ydp = []; ydotp = []; yddotp = [];

step = 0.01;
for t=0:step:30
  
  %desired trajectory
  yd = 5 + 2*sin(t);
  ydot = 2*cos(t);
  yddot = -2*sin(t);
  
  %errors
  e = yd - q;
  edot = ydot - qdot;
  eddot = yddot - qddot;
  
  %control signal
  u = m*yddot + kv*edot +kp*e;
  
  %model
  qddot = (u-(b*q*qdot))/(m*(1+q*q));
  
  %integrate
  qdot = qdot + qddot*step;
  q  = q  + qdot*step;
  
  %for plotting
  qp(index) = q;
  qdotp(index) = qdot;
  qddotp(index) = qddot;
  ep(index) = e;
  edotp(index) = edot;
  eddotp(index) = eddot;
  ydp(index) = yd;
  ydotp(index) = ydot;
  yddotp(index) = yddot;
  index = index+1;
endfor

figure(1)
hold on;
plot(qp);
%plot(qdotp);
%plot(qddotp);
%legend("y", "y'", "y''");
title("Model");

figure(2)
hold on;
plot(ep)
%plot(edotp)
%plot(eddotp)
%legend("e", "e'", "e''");
title("Error");

figure(3)
hold on;
plot(ydp)
plot(ydotp)
plot(yddotp)
legend("yd", "yd'", "yd''");
title("Trajectory");
