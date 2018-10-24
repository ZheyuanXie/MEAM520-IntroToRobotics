c1=sym('c1');
c2=sym('c2');
c3=sym('c3');
c4=sym('c4');
c5=sym('c5');
s1=sym('s1');
s2=sym('s2');
s3=sym('s3');
s4=sym('s4');
s5=sym('s5');

R01=[c1 0 -s1;s1 0 c1;0 -1 0];
R12=[s2 c2 0;-c2 s2 0;0 0 1];
R23=[-s3 -c3 0;c3 -s3 0;0 0 1];
R03=R01*(R12*R23);
disp(R03)

R34=[s4 0 c4;-c4 0 s4;0 -1 0];
R46=[c5 -s5 0;s5 c5 0;0 0 1];
R36=R34*R46;
disp(R36)