%This function returns an automatically generated matrix of 
%LHP poles for a system of n agents

function[poles]=genPoles(n)

poles = -n-1:.25:-1.25;
poles = poles.*.3;

%Poles are generated this way because the place() command does not allow
%repeated poles. When there are many agents, the poles generated this way
%may become very far into the left half plane. The poles can be made
%smaller for practical purposes, so the robots will respond slower.
%For a small number of agents, the poles will be very small and may need
%to be multiplied larger.

end