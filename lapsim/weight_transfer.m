clear
clc
%%%%function input%%%%

v = sqrt(100); %tangential velocity [m/s] - contributes to roll
a = 0; %tangential acceleration [m/s^2] - contributes to dive/squat
corner = 1; %boolean corner input [0 or 1]
R = 20; %corner radius [m]

%%%%%Static Car Characteristics%%%%

ms = 210; %sprung mass [kg]
mu = [10, 10, 10, 10]; %unsprung mass [kg] - [front inside, front outside, rear inside, rear otside]

mur = mu(3)+mu(4); %rear unsprung mass [kg]
muf = mu(1)+mu(2); %front unprung mass [kg]

hms = 0.3; %ms CG height [m]
L = 1.575; %wheelbase [m]                                              %Shanks: Added correct value

rf = 0.27; %front wheel radius [m]                                     
rr = 0.28; %rear wheel radius [m]
Tf = 0.7; %front track [m]
Tr = 0.7; %rear track [m]

wdist = 0.493; %fraction of ms on front wheels                          %Shanks: added this for correct lms calculation
lms = (1 - wdist) * L; %ms CG longitudinal location from front axle [m]

hrcf = 0.66; %front roll center to ms height [m]                        %Shanks: Calculate these
hrcr = 0.77; %rear roll center to ms height [m]
hpc = 0.5; %pitch center to ms height [m]

Rm = [1.3, 1.3, 1.3, 1.3]; %motion ratio [-] [front, rear]
Ks = [37200, 37200, 37200, 37200]; %spring rate [N/m] [front, rear]
Kt = [250000, 250000, 250000, 250000]; %tire vertical stifness [N/m]

anti_dive = 0.3; %percent anti dive geometry
anti_squat = 0.3; %percent anti squat geometry

%calculated car characteristics
Kw = Ks./Rm; % Wheel center rate [N/m]
Kr_f = 1/(1/((Kw(1)+Kw(2))/2) + 1/((Kt(1)+Kt(2))/2)); %ride rate front [N/m]
Kr_r = 1/(1/((Kw(3)+Kw(4))/2) + 1/((Kt(3)+Kt(4))/2)); %ride rate rear [N/m]
K_phi_f = Tf^2*Kr_f/114.6; %front roll rate [Nm/deg]
K_phi_r = Tr^2*Kr_r/114.6; %rear roll rate [Nm/deg]

K_theta = L^2*Kr_f/114.6; %pitch rate [Nm/deg]

hra = hms-hrcf+lms*(hrcr-hrcf)/L; %sprung mass center to roll axis height
%%%%Function Calculations%%%%

ay = v^2/R; %lateral acceleration [m/s^2]
phi = ms*ay*hra/(K_phi_f+K_phi_r)*corner; %body roll [deg]
theta = abs(ms*a*hpc)/K_theta; %body pitch [deg]

del_hms = -hpc*(1-cos(theta))-hra*(1-cos(phi)); %change in ms height due to roll/pitch
del_lms = sign(a)*hpc*sin(theta); %change in ms location due to roll/pitch

%update ms location
hms = hms+del_hms;
lms = lms +del_lms;


    %calc static loads
    Wr = 0.5*9.81*(mur+ms*lms/L); %individual static rear wheel loads [N]
    Wf = 0.5*9.81*(mur+muf+ms)-Wr; %individual static front wheel loads [N]

    %calc unsprung mass lateral force transfer
    del_Wuf = ay*muf*rf/Tf; %lateral front unsprung force [N]
    del_Wur = ay*mur*rr/Tr; % lateral rear unsprung force [N]

    %calc sprung mass lateral force transfer through suspension links
    del_Wsff = ay*ms * (L-lms)/L * hrcf/Tf; %lateral front sprung force [N]
    del_Wsfr = ay*ms *lms/L * hrcr/Tr; %lateral rear sprung force [N]

    %calc sprung mass roll couple through the springs
    del_Wscf = (K_phi_f*ay*ms*hra/(K_phi_f+K_phi_r))/Tf; %front axle roll force [N]
    del_Wscr = (K_phi_r*ay*ms*hra/(K_phi_f+K_phi_r))/Tr; %rear axle roll force [N]


%calc longitudinal load transfer
del_Wlr = (sign(a)*abs(ms*a*hms)+9.81*(ms*lms+mur*L))/(2*L)-Wr; %longitudinal load transfer rear [N]
del_Wlf = -1*del_Wlr; %longitudinal load transfer front [N]


%final loads
Wfi = Wf-(del_Wuf+del_Wsff+del_Wscf)*corner+del_Wlf; %total load front inside wheel [N]
Wfo = Wf+(del_Wuf+del_Wsff+del_Wscf)*corner+del_Wlf; %total load front outside wheel [N]
Wri = Wr-(del_Wur+del_Wsfr+del_Wscr)*corner+del_Wlr; %total load rear inside wheel [N]
Wro = Wr+(del_Wur+del_Wsfr+del_Wscr)*corner+del_Wlr; %total load read outside wheel [N]
load = [Wfi, Wfo, Wri, Wro];
delta = load - [Wf, Wf, Wr, Wr]; %total change in loading [N]