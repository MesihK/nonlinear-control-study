% author: Mesih Veysi Kılınç
% date:   24-07-2019
% title:  Plot barrier lypunov funtions
ln = [];
trig = [];
trig2 = [];
index = 1;

length = 0.999;
step = 0.001;
for t=-length:step:length
  ln(index)   = log(1/(1-t*t));
  trig(index) = tan(pi/2*t*t);
  trig2(index) = tan(pi/2*t)^2;
  
  if(trig(index) > 10) trig(index) = 10; endif;
  if(trig2(index) > 10) trig2(index) = 10; endif;
  if(ln(index) > 10) ln(index) = 10; endif;
  index += 1;
endfor

hold on;
plot(-length:step:length, ln);
plot(-length:step:length, trig);
plot(-length:step:length, trig2);
legend("ln","tan", "tan^2");
