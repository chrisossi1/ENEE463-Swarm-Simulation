function[Nbar]=rscale(A,B,C,D,K)
% Given the single-input linear system: 
%       .
% 		x = Ax + Bu
%   	y = Cx + Du
% and the feedback matrix K,
% 
% the function rscale(A,B,C,D,K) finds the scale factor N which will 
% eliminate the steady-state error to a step reference 
% using the schematic below:
%
%                         /---------\
%      R         +     u  | .	    |
%      ---> N --->() ---->| X=Ax+Bu |--> y=Cx ---> y
%                -|       \---------/
%                 |             | 
%                 |<---- K <----|
%
%8/21/96 Yanjie Sun of the University of Michigan
%        under the supervision of Prof. D. Tilbury
%
s = size(A,1);
Z = [zeros([1,s]) ones(1,s/2)]; %This line modified to support multiple outputs
N = inv([A,B;C,D])*Z';
Nx = N(1:s);
Nu = N(1+s);
Nbar=Nu + K*Nx;