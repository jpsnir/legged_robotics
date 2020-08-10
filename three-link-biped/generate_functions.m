% This .m file is used to symbolically derive:
%           the dynamics of the 3 link biped
%           impact map
%           controller
%           zero dynamics
% and write them onto function files
%
% Biped model: 
%   D, C, G matrices are found using a defined forward position kinematics,
%   B matrix must be defined manually 
%
% Impact map:
%   De, E. are derived using extended coordinates: p_e = [p_h; p_v]
%
% Controller:
%   The L2fh and LgLfh matrices used in feedback linearization are
%   symbolically derived
%
% Zero dynamics:
%   Vectors used in zero dynamics (eta2) are also derived
%

%-------------------------------------------------------------------------%
%%%% DCG matrices

syms q1 q2 q3 p_h p_v dp_h dp_v dq1 dq2 dq3 real
syms r m Mh Mt l g real

% Define parameters in a vector
params = [r,m,Mh,Mt,l,g];

%Mh -  mass of hip, Mt - mass of torso, m - mass of legs
%l - length from hip to torso, r - length of legs

% Defining generalized coordinates:
% Angular positions:
%           q1: stance leg (absolute, w.r.t. y axis of 
%           q2: swing leg (relative to q1)
%           q3: torso (relative to q1)
% Angular velocities dq/dt:
%           dq1: stance leg 
%           dq2: swing leg 
%           dq3: torso  
q = [q1; q2; q3];
dq = [dq1; dq2; dq3];

% We hardcoded T1,T2 because using rotz(pi) and rotz(-pi/2) yielded
% annoyingly small numerical answers
% rotz(pi)
T1 = [-1 0 0 0;
     0 -1 0 0;
     0 0 1 0;
     0 0 0 1];
%  rotz(-pi/2)
T2 = [0 1 0 0;
     -1 0 0 0;
     0 0 1 0;
     0 0 0 1];
T01 = rotz(q1)*transy(r);
T12 = T1*rotz(q2)*transy(r);
T13 = T1*rotz(q3)*transy(l);

% Position Vectors
% pmh_1 is pmh in frame 1
% _Y indicates Frame Y
pmh_1 = [0;0;0];
pm1_1 = [0;-r/2;0];
pm2_2 = [0;-r/2;0];
p2_2 =  [0;0;0];
pmt_3 = [0;0;0];

% Position Vectors in World Frame
pmh_0 = simplify(extr(T01*aug(pmh_1)));
pm1_0 = simplify(extr(T01*aug(pm1_1)));
pm2_0 = simplify(extr(T01*T12*aug(pm2_2)));
pmt_0 = simplify(extr(T01*T13*aug(pmt_3)));
p2_0 =  simplify(extr(T01*T12*aug(p2_2)));

% Forward Kinematics - position of point masses
% hip
pMh = pmh_0;
% torso
pMt = pmt_0;
% stance leg
pm1 = pm1_0;
% swing leg
pm2 = pm2_0;
% center of mass
pcm = (Mh*pMh+Mt*pMt+m*pm1+m*pm2)/(Mh+Mt+m+m);
% end of swing leg
p2 = p2_0;


% Write positions to a file
% Inputs:
%       q
%       dq
%       params
%
% Outputs: Position vectors with x and y coordinates of position
%       pMh
%       pMt
%       pm1
%       pm2
%       pcm
%       P2
%
write_symbolic_term_to_mfile(q,dq,params,pMh,pMt,pm1,pm2,pcm,p2)

% Velocities - found by taking partial derivative w.r.t. q, then multiply
% by dq/dt
vMh = simplify(jacobian(pMh,q)*dq);
vMt = simplify(jacobian(pMt,q)*dq);
vm1 = simplify(jacobian(pm1,q)*dq);
vm2 = simplify(jacobian(pm2,q)*dq);
vcm = simplify(jacobian(pcm,q)*dq);

% Write velocities to a file
% Inputs:
%       q
%       dq
%       params
%
% Outputs: Velocity vectors with x and y coordinates of velocity
%       vMh
%       vMt
%       vm1
%       vm2
%       vcm
%       
write_symbolic_term_to_mfile(q,dq,params,vMh,vMt,vm1,vm2,vcm)


% Kinetic energy
%%%%%%%%%%%%%%%%%% Compute kinetic energy of each component here %%%%%%%%%%
K_Mh = simplify((1/2)*dq'*Mh*jacobian(pMh,q)'*jacobian(pMh,q)*dq);
K_Mt = simplify((1/2)*dq'*Mt*jacobian(pMt,q)'*jacobian(pMt,q)*dq);
K_m1 = simplify((1/2)*dq'*m*jacobian(pm1,q)'*jacobian(pm1,q)*dq);
K_m2 = simplify((1/2)*dq'*m*jacobian(pm2,q)'*jacobian(pm2,q)*dq);
% Total KE
K = simplify(K_Mh + K_Mt + K_m1 + K_m2);

% Potential energy
%%%%%%%%%%%%%%%%%% Compute potnetial energy of each component here %%%%%%%%
y = [0;1];
V_Mh = simplify(Mh*g*(y'*pMh));
V_Mt = simplify(Mt*g*(y'*pMt));
V_m1 = simplify(m*g*(y'*pm1));
V_m2 = simplify(m*g*(y'*pm2));
% Total PE
V = simplify((Mh+Mt+2*m)*g*(y'*pcm));

% Inertia matrix
%%%%%%%%%%%%%%%%%% Compute D matrix here %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
D = jacobian(jacobian(K,dq),dq);
% D = simplify(Mh*jacobian(pMh,q)'*jacobian(pMh,q)+...
%              Mt*jacobian(pMt,q)'*jacobian(pMt,q)+...
%              m*jacobian(pm1,q)'*jacobian(pm1,q)+...
%              m*jacobian(pm2,q)'*jacobian(pm2,q));

% Coriolis matrix
%%%%%%%%%%%%%%%%%% Coriolis matrix here %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = max(size(q));
syms C
for k = 1:N
    for j = 1:N
        C(k,j) = 0*g; %#ok
        for i = 1:N
            C(k,j) = C(k,j) + (1/2)*(...
                jacobian(D(k,j),q(i)) + ...
                jacobian(D(k,i),q(j)) - ...
                jacobian(D(i,j),q(k)) ...
                ) * dq(i);
        end
    end
end
C = simplify(C);

% Gravity matrix
%%%%%%%%%%%%%%%%%% Compute Gravity term here %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G = jacobian(V,q)';

%%%%%%%%%%%%%%%%%% What is control input matrix? %%%%%%%%%%%%%%%%%%%%%%%%%%
B = [sym(0) sym(0); sym(1) sym(0); sym(0) sym(1)];

% Write 3 link model to file
% Inputs:
%       q
%       dq
%       params
%
% Outputs: 
%       D: Inertia matrix
%       C: Coriolis matrix
%       G: Gravity matrix
%       B: 
%
write_symbolic_term_to_mfile(q,dq,params,D,C,G,B)
% 
%-------------------------------------------------------------------------%
%%%% Impact map

% Using same psotion vectors as above, but taking partial with respect to qe
% instead

% Extended configuration variables
p_e = [p_h; p_v];

qe = [q; p_h; p_v];
dqe = [dq; dp_h; dp_v];

% Extended position
pMh_e = pMh + p_e;
pMt_e = pMt + p_e;
pm1_e = pm1 + p_e;
pm2_e = pm2 + p_e;
p2_e = p2 + p_e;

% Extended inertia matrix
m_tot = Mh+Mt+2*m;
De = simplify(Mh*jacobian(pMh_e,qe)'*jacobian(pMh_e,qe)+...
             Mt*jacobian(pMt_e,qe)'*jacobian(pMt_e,qe)+...
             m*jacobian(pm1_e,qe)'*jacobian(pm1_e,qe)+...
             m*jacobian(pm2_e,qe)'*jacobian(pm2_e,qe));

% Partial of any point on biped, hip chosen in this case
dY_dq = jacobian(p2_e,qe);

E = dY_dq;

% Write impact map to a file
% Inputs:
%       q
%       dq
%       params
%
% Outputs: Matrices needed to compile impact map
%       De: Extended inertia matrix
%       E:
%       dY_dq:
%       
write_symbolic_term_to_mfile(q,dq,params,De,E,dY_dq)


%-------------------------------------------------------------------------%
%%%% For controller

% Vector fields
fx = [dq; D\(-C*dq-G)];
gx = [zeros(3,2);D\B];

% Bezier poly - needed for output function
syms s delq
% s = (q1 - q1_plus)/delq; 
% delq = q1_minus - q1_plus; 
% ds/dt = dq1/delq; ds/dq1 = 1/delq;

syms a21 a22 a23 a24 a25 
syms a31 a32 a33 a34 a35

a2 = [a21 a22 a23 a24 a25];
a3 = [a31 a32 a33 a34 a35];
M = 4;

b2 = 0; b3 = 0;
for k = 0:M
    b2 = b2 + a2(1,k+1)*(factorial(M)/(factorial(k)*factorial(M-k)))*s^k*(1-s)^(M-k);
end

for k = 0:M
    b3 = b3 + a3(1,k+1)*(factorial(M)/(factorial(k)*factorial(M-k)))*s^k*(1-s)^(M-k);
end

% Defining outputs

h = [q2 - b2; q3 - b3];

% y_dot = Lfh = dh/dx*fx - independent of gx*u since relative degree is 2
% However, h is a function of (s,q2,q3), not q1 directly, so the following
% is used:
% dh/dq1 = dh/ds*ds/dq1 = dh/ds*1/delq
%
% Temporary variable that multiples the 1st column with 1/delq
temp = sym(eye(6)); temp(1)  = 1/delq;

Lfh = jacobian(h,[s; q2; q3; dq])*temp*fx;

dLfh = jacobian(Lfh,[s; q2; q3; dq])*temp;

% Write matrix used in feedback linearization - d/dx(Lfh) to file
% Inputs:
%       s = (q1 - q1_plus)/delq: gait timing variable
%       delq = q1_minus - q1_plus: difference in cyclic variable during gait 
%       dq1
%       params: 
%       a2: bezier coefficents (1st - 5th) for q2
%       a3: bezier coefficents (1st - 5th) for q3
%
% Outputs:
%       dLfh: partial of Lfh, to be used to compute L2fh and LgLfh
%
write_symbolic_term_to_mfile([s,delq],dq1,[a2,a3],dLfh);



%-------------------------------------------------------------------------%
%%%% For Zero Dynamics


db_ds2 = 0;
for k = 0:M-1
    db_ds2 = db_ds2 + (a2(1,k+2)-a2(1,k+1))*(factorial(M)/(factorial(k)*factorial(M-k-1)))*s^k*(1-s)^(M-k-1);
end

db_ds3 = 0;
for k = 0:M-1
    db_ds3 = db_ds3 + (a3(1,k+2)-a3(1,k+1))*(factorial(M)/(factorial(k)*factorial(M-k-1)))*s^k*(1-s)^(M-k-1);
end

partial_db_ds2 = jacobian(db_ds2,s)*dq1/delq;

partial_db_ds3 = jacobian(db_ds3,s)*dq1/delq;

beta1 = [partial_db_ds2; partial_db_ds3]*dq1/delq;

eta2 = jacobian(K,dq1);

write_symbolic_term_to_mfile(s,[dq1, delq],[a2, a3],beta1)

write_symbolic_term_to_mfile(q,dq,params,eta2)

% --------------- F U N C T I O N S -----------------

function pnew = aug(p)
% ---------------------------------------------------   
% 
% Author:     Andrew Lessieur
% 
% Purpose:    Augments position vector to be a 4x1 by adding a 1 in its
%             fourth entry
% 
% ---------------------------------------------------
    pnew = [p;1];
end

function pnew = extr(p)
% ---------------------------------------------------   
% 
% Author:     Andrew Lessieur
% 
% Purpose:    Extracts position vector from a 4x1 vector given by a
%             homogeneous transformation
% 
% ---------------------------------------------------
    pnew = p(1:2);
end

% ---------------------------------------------------
% Rotation Matrices SO(3)
% ---------------------------------------------------

function R = rx(q)
    R = [1      0        0   ;...
         0    cos(q)  -sin(q);...
         0    sin(q)   cos(q)];
end
function R = ry(q)
    R = [ cos(q)    0    sin(q) ;...
           0        1     0     ;...
         -sin(q)    0    cos(q)];
end
function R = rz(q)
    R = [cos(q)  -sin(q)  0;...
         sin(q)   cos(q)  0;...
          0        0      1];
end
function R = rz2(q) %rotate clockwise w/ positive angle
    R = [cos(q)  sin(q)  0;...
         -sin(q)   cos(q)  0;...
          0        0      1];
end

% ---------------------------------------------------
% Translation Transforms
% ---------------------------------------------------

function H = transx(a) 
    H = [ eye(3)   [a; 0; 0];...
         zeros(1,3)    1   ];
end
function H = transy(b) %#ok
    H = [ eye(3)   [0; b; 0];...
         zeros(1,3)    1   ];
end
function H = transz(c) %#ok
    H = [ eye(3)   [0; 0; c];...
         zeros(1,3)    1   ];
end

% ---------------------------------------------------
% Rotation Transforms
% ---------------------------------------------------

function H = rotx(q) %#ok
    H = [   rx(q)   zeros(3,1);...
         zeros(1,3)     1     ];
end
function H = roty(q) %#ok
    H = [   ry(q)   zeros(3,1);...
         zeros(1,3)     1     ];
end
function H = rotz(q)
    H = [   rz(q)   zeros(3,1);...
         zeros(1,3)     1     ];
end
function H = rotz2(q)
    H = [   rz2(q)   zeros(3,1);...
         zeros(1,3)     1     ];
end


