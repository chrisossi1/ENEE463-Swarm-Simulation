close all;
clear all;

%Point mass robot model:
frict = .5;
A = [0 1-frict 0 0;0 0 0 0;0 0 0 1-frict;0 0 0 0];
B = [0 0;1 0;0 0;0 1];
C = [1 0 0 0;0 0 1 0];
K = place(A,B,[-4 -4.5+1i -4.5-1i -5]);

x0 = [1;-1;0;.5];

%%Open and closed loop dynamics

t = 0:.01:2;
len = size(t)-1;

%Open loop dynamics
olsys = ss(A,B,C,0);

u = zeros(len(2)+1,2);

[yo,t,x] = lsim(olsys,u,t,x0);

figure;
plot(t,yo);
xlabel('time');
ylabel('distance');
legend('x','y');
title('Single point mass open loop dynamics')
isstable(olsys)
%Not surprisingly, a free point mass in 2d space is not stable.

clsys = ss(A-B*K,B,C,0);

[yc,t,x] = lsim(clsys,u,t,x0);
figure;
plot(t,yc);
xlabel('time');
ylabel('distance');
legend('x','y');
title('Single point mass closed loop dynamics')

isstable(clsys)
%The full state feedback stabilizes the point mass to zero.

%{
res = 4;
for i = 1:res:len(2)
    hold off;
    plot(yc(i,1),yc(i,2),'or','MarkerSize',5,'MarkerFaceColor','r')
    hold on;
    plot(yo(i,1),yo(i,2),'or','MarkerSize',5,'MarkerFaceColor','b','MarkerEdgeColor','b')
    
    title(['Stabilizing point robot to the origin, t = ',num2str(i)]);
    legend('Full state feedback','Open loop')
    
    axis([-2 2 -2 2])

    pause(.01);
end

%}

t = 0:.01:10;
len = size(t)-1;

x0 = [.5,0,.5,0];

%choose 4 random coordinates
xr1 = rand;
yr1 = rand;
xr2 = rand;
yr2 = rand;
xr3 = rand;
yr3 = rand;
xr4 = rand;
yr4 = rand;

u = [
    xr1*ones(len(2)/4,1),yr1*ones(len(2)/4,1)
    xr2*ones(len(2)/4,1),yr2*ones(len(2)/4,1)
    xr3*ones(len(2)/4,1),yr3*ones(len(2)/4,1)
    xr4*ones(len(2)/4,1),yr4*ones(len(2)/4,1)   
    0,0
    ]';

Nbar=rscale(A,B,C,[0 0;0 0],K); 

uscale = [Nbar(1)*u(1,:);Nbar(2)*u(2,:)];

clsys = ss(A-B*K,B,C,0);

[yc,t,x] = lsim(clsys,uscale,t,x0);
figure;
hold on;
plot(t,yc);
plot(t,u,'--');
xlabel('time');
ylabel('distance');
legend('x','y','u_x','u_y');
title('Single point mass reference point tracking')
hold off;


%{

res = 4;

for i = 1:res:len(2)
    hold off;
    plot(yc(i,1),yc(i,2),'or','MarkerSize',5,'MarkerFaceColor','r')
    hold on;
    plot(u(1,i),u(2,i),'x','MarkerSize',5,'MarkerFaceColor','b')
        
    title(['Stabilizing point robot to the reference, t = ',num2str(i)]);
    legend('Controlled point mass','Reference position')
    
    axis([0,1,0,1])

    pause(.01);
end

%}

%We had some trouble achieving consistent input tracking.
%This is probably due to problems with the feedforward control
%scaling factor Nbar. We modified the rscale.m function by
%Yanjie Sun of the University of Michigan to support MIMO systems.
