function out = mavlinkSpoofHIL(u)

message_type = 5;
t  =          1.0;
pn =          1.0;
pe =          1.0;
h =           1.0;
u =           1.0;
v =           1.0;
w =           1.0;
alpha =       1.0;
beta  =       1.0;
phi =         1.0;
theta =       1.0;
psi =         1.0;
p =           1.0;
q =           1.0;
r =           1.0;
gamma =       1.0;
chi =         1.0;
wn =          1.0;
we =          1.0;
wd =          1.0;


out(1) = message_type;
out(2) = t;
out(3) = pn;
out(4) = pe;
out(5) = h;
out(6) = u;
out(7) = v;
out(8) = w;
out(9) = alpha;
out(10) = beta;
out(11) = phi;
out(12) = theta;
out(13) = psi;
out(14) = p;
out(15) = q;
out(16) = r;
out(17) = gamma;
out(18) = chi;
out(19) = wn;
out(20) = we;

end