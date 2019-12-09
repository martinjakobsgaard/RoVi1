clc; clear;

T1 = readtable('ROBDATA1.dat');
T2 = readtable('ROBDATA2.dat');
T3 = readtable('ROBDATA3.dat');

time1avg=mean(reshape(T1.time,10,[]))';
time2avg=mean(reshape(T2.time,10,[]))';
time3avg=mean(reshape(T3.time,10,[]))';

distance1avg=mean(reshape(T1.distance,10,[]))';
distance2avg=mean(reshape(T2.distance,10,[]))';
distance3avg=mean(reshape(T3.distance,10,[]))';

steps1avg=mean(reshape(T1.steps,10,[]))';
steps2avg=mean(reshape(T2.steps,10,[]))';
steps3avg=mean(reshape(T3.steps,10,[]))';

hold on;

title('Line plot of number of steps for different epsilon values')
xlabel('Epsilon * 100 [cm]') 
ylabel('Steps') 
plot(steps1avg)
plot(steps2avg)
plot(steps3avg)

hold off;

%title('Line plot of distance travelled for different epsilon values')
%xlabel('Epsilon * 100') 
%ylabel('Distance') 
%plot(distance1avg)
%plot(distance2avg)
%plot(distance3avg)

%title('Line plot of calculation time for different epsilon values')
%xlabel('Epsilon * 100 [cm]') 
%ylabel('Time [s]') 
%plot(time1avg)
%plot(time2avg)
%plot(time3avg)

%title('Line plot of number of steps for different epsilon values')
%xlabel('Epsilon * 100') 
%ylabel('Steps') 
%plot(steps1avg)
%plot(steps2avg)
%plot(steps3avg)