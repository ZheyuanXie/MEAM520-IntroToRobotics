syms q1 q2 q3 q4 q5 d1 a2 a3 d5 lg

%Frame 1 w.r.t Frame 0
T1 = [cos(q1) 0  -sin(q1)  0;
      sin(q1)  0 cos(q1)  0;
              0            -1            0 d1;
              0                     0                  0     1];
          
%Frame 2 w.r.t Frame 1          
T2 = [cos(q2-(pi/2)) -sin(q2-(pi/2))  0   a2*cos(q2-(pi/2));
      sin(q2-(pi/2))  cos(q2-(pi/2))  0   a2*sin(q2-(pi/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 3 w.r.t Frame 2
T3 = [cos(q3+(pi/2)) -sin(q3+(pi/2))  0   a3*cos(q3+(pi/2));
      sin(q3+(pi/2))  cos(q3+(pi/2))  0   a3*sin(q3+(pi/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 4 w.r.t Frame 3
T4 = [cos(q4-(pi/2)) 0   -sin(q4-(pi/2))   0;
      sin(q4-(pi/2))  0  cos(q4-(pi/2))   0;
              0       -1        0   0;
              0       0        0   1];
%Frame 5 w.r.t Frame 4 
T5 = [cos(q5) -sin(q5)  0        0;
      sin(q5)  cos(q5)  0        0;
              0          0  1       d5;
              0          0  0        1];
          
%Frame 5 w.r.t Frame 4 
T6 = [ 1  0  0   0;
       0  1  0   0;
       0  0  1  lg;
       0  0  0   1];

T0e = T1*T2*T3*T4*T5*T6;
