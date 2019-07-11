% author: Mesih Veysi Kılınç
% date:   10-07-2019
% title:  Vanderpol Oscillator Simulation.
clc
clear all

y = 1;  ydot = 0; yddot = 0;

%for plotting
index = 1;
yp = []; ydotp = []; yddotp = [];

step = 0.01;
for t=0:step:10
  %model
  yddot = -(y*y-1)*ydot -y
  
  %integrals
  ydot = ydot + yddot*step;
  y  = y  + ydot*step;
  
  %for plotting
  yp(index) = y;
  ydotp(index) = ydot;
  yddotp(index) = yddot;
  index = index+1;
endfor

figure(1)
hold on;
plot(yp);
plot(ydotp);
plot(yddotp);
legend("y", "y'", "y''");
figure(2)
plot(yp,ydotp)
title("y vs y'")
