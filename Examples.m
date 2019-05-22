close all;
clear all;

n = 40; %n is the number of agents. The system will have n*4 states and n*2 inputs and outputs.
%You may want to choose fewer agents depending on your system.

poles = -n-1:.25:-1.25;
poles = poles.*.3;
%Poles are generated this way because the place() command does not allow
%repeated poles. When there are many agents, the poles generated this way
%may become very far into the left half plane. The poles can be made
%smaller for practical purposes, so the robots will respond slower.
%For a small number of agents, the poles will be very small and may need
%to be multiplied larger.


%simSwarm(number of agents, velocity alignment weight, centroid
%attraction/repulsion weight,poles);

%The plot is in 3 dimensions, so you can rotate the plot if you are running
%this code.


%First, a simulation with no velocity alignment or centroid attraction.
simSwarm(n,0,0,poles);

%Swarm simulation with velocity alignment only.
simSwarm(n,.75,0,poles);
%Adding velocity aligment greatly reduces overshoot

%Swarm simulation with centroid attraction only.
simSwarm(n,0,1,poles);
%Centroid attraction keeps the swarm cohesive, but decreases response time.

%Swarm simulation with centroid repulsion only.
simSwarm(n,0,-1,poles);
%The agents can be seen dispersing more, causing large transients.
%This kind of behavior could be useful when there is no go-to-goal
%reference tracking, making the swarm evenly distribute in a given area
%(especially when only the local centroid is used, rather than global).

%Swarm simulation with mild centroid repulsion and strong velocity
%alignment. Seeing as the reference tracking control acts to keep the
%agents cohesive, slight centroid repulsion can be used to keep the swarm
%from collapsing into to small an area.
simSwarm(n,.75,-.4,poles);


%simSwarmLQR(n,0,0);

%simSwarmLQR(n,49,0);


