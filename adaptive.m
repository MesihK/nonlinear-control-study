% author: Mesih Veysi Kılınç
% date:   11-07-2019
% title:  Nonlinear Controller Simulation with adaptive control.
clc
clear all

%model parameters
m = 3.5; b = 2;

%estimates
be = 0; bedot = 0;
me = 0; medot = 0;
g1 = 2; g2 = 3;
r = 0;

%gains
a = 1; k = 10;

q = 0;  qdot = 0; qddot = 0;
yd = 5; ydot = 0; yddot = 0;
e = yd-q; edot = 0; eddot = 0; 

%for ploting
index = 1;
qp = []; qdotp = []; qddotp = [];
ep = []; edotp = []; eddotp = [];
ydp = []; ydotp = []; yddotp = [];
bep = []; mep = []; 

step = 0.01;
for t=0:step:50
  
  %desired trajectory
  yd = 5 + 2*sin(t);
  ydot = 2*cos(t);
  yddot = -2*sin(t);
  
  %errors
  e = yd - q;
  edot = ydot - qdot;
  eddot = yddot - qddot;
  r = edot + a*e;
  
  %estimates
  bedot = (g1*q*qdot*r)/(1+q*q);
  medot = (g2*yddot*r);
  be = be + bedot*step;
  me = me + medot*step;
  
  %control signal
  u = be*q*qdot+me*(1+q*q)*yddot + (1+q*q)*(k*(edot + a*e) + a*edot);
  
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
  bep(index) = be;
  mep(index) = me;
  index = index+1;
endfor

figure(1)
hold on;
plot(qp);
plot(qdotp);
plot(qddotp);
legend("y", "y'", "y''");
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

figure(4)
hold on;
plot(bep)
plot(mep)
legend("b", "m'");
title("Estimates");
