p = sym("pi");
l1 = sym("L1");
l2 = sym("L2");
l3 = sym("L3");
l4 = sym("L4");
q1 = sym("q1");
q2 = sym("q2");
q3 = sym("q3");
q4 = sym("q4");
q5 = sym("q5");


T01 = dh2tf([0 0 l1 0])
T12 = dh2tf([0 -p/2 0 q1])
T23 = dh2tf([l2 0 0 -p/2+q2])
T34 = dh2tf([l3 0 0 p/2+q3])
T45 = dh2tf([0 p/2 0 p/2+q4])
T5e = dh2tf([0 0 l4 q5])
T0e = T01*T12*T23*T34*T45*T5e