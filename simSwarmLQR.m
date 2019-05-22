%simSwarmLQR(number of agents, velocity alignment weight, centroid weight);

function[Poles_LQR]=simSwarmLQR(n,vw,cw)

%%Defining dynamics
frict=0;
A = [cw 1-frict 0 0;0 0 0 0;0 0 cw 1-frict;0 0 0 0];
B = [0 0;1 0;0 0;0 1];
C = [1 0 0 0;0 0 1 0];

Aconnect = [-cw/n vw/n 0 0;0 0 0 0;0 0 -cw/n vw/n;0 0 0 0];
Afinal = [];
for i=0:n-1
    Arow = [];
    for j=0:n-1
        if (j==i)
            Arow = [Arow, A];
        else
            Arow = [Arow, Aconnect];
        end
    end
    Afinal = [Afinal;Arow];
end

A=Afinal;

B = repmat({B},1,n);
B = blkdiag(B{:});

C = repmat({C},1,n);
C = blkdiag(C{:});

%%initializing robots at random positions with velocity 0

x0=[];
for i=1:n
    singleAgent = [rand ;0;rand; 0];
    x0=[x0;singleAgent];
end


%%Caluclating the LQR control
origsys = ss(A,B,C,0);
Q = eye(size(C'*C));
R = eye(size(B'*B));
N=0;

K_LQR = lqr(origsys,Q,R,N);

Poles_LQR = eig(A-B*K_LQR);
clsys = ss(A-B*K_LQR,B,C,0);


%%Simulating the system

t = 0:.01:20;
len = size(t)-1;

%choose 4 coordinates close to the x=y line so that the Nbar scaling
%mismatch is minimized, also makes for clearer plotting
xr1 = rand*.25+.75;
yr1 = xr1+rand/50;
xr2 = rand*.25+.25;
yr2 = xr2+rand/50;
xr3 = rand*.25+.5;
yr3 = xr3+rand/50;
xr4 = rand*.25;
yr4 = xr4+rand/50;

u = [
    xr1*ones(len(2)/4,n),yr1*ones(len(2)/4,n)
    xr2*ones(len(2)/4,n),yr2*ones(len(2)/4,n)
    xr3*ones(len(2)/4,n),yr3*ones(len(2)/4,n)
    xr4*ones(len(2)/4,n),yr4*ones(len(2)/4,n)   
    zeros(1,n*2)
    ]';

Nbar=rscale(A,B,C,zeros(2*n),K_LQR); 

[w,h] = size(u);
Nbar1 = [];
for i=0:h-1
    Nbar1=[Nbar1 Nbar];
end
uscale = Nbar1.*u;

%%Plotting the simulation

figure;
hold on;
[yc,t,x] = lsim(clsys,uscale,t,x0);
plot3(t,yc(:,1:2:end),yc(:,2:2:end))
plot3(t,u(1,:),u(2,:))

xlabel('time');
ylabel('x');
ylabel('y');

title(['position of swarm over time, cw = ',num2str(cw),', vw = ',num2str(vw),', avg pole = ',num2str(mean(Poles_LQR))]);

hold off;
end

