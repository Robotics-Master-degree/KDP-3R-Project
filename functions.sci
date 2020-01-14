function[x,y,gamma] = jointPosition2EndEffectPose(theta1,theta2,theta3,L1,L2,g1,g2,g3)
a12=L1
a23=L2
a34_1=g1+g3
a34_2=-g2

x=a12*cos(theta1)+a23*cos(theta1+theta2)+a34_1*cos(theta1+theta2+theta3)+a34_2*sin(theta1+theta2+theta3)
y=a12*sin(theta1)+a23*sin(theta1+theta2)+a34_1*sin(theta1+theta2+theta3)-a34_2*cos(theta1+theta2+theta3)
gamma=theta1+theta2+theta3

endfunction


function[theta1,theta2,theta3] = Endeffector2jointposition(x_,y_,gamma_,g1,g2,g3,L1,L2)
a12=L1
a23=L2
a34=g1


//Calculating theta2
R_gamma = [cos(gamma_), -sin(gamma_); sin(gamma_), cos(gamma_)]
//X = [x_ ; y_]-R_gamma*[g2;0]- R_gamma*[0;g3]
X = [x_ ; y_]-R_gamma*[g3;0]- R_gamma*[0;g2]

x = X(1)
y = X(2)
  
d = 2*a12*a23
f = (x - a34*cos(gamma_))^2 + (y -a34*sin(gamma_))^2 -(a12)^2 - (a23)^2
theta2=acos(f/d)
if (theta2 > %pi) then 
    theta2 = 2*%pi - theta2
end

//calculating theta1
A = a12 + a23*cos(theta2) //ok
B = a23*sin(theta2) //ok
E = x - a34*cos(gamma_) //ok
F = y - a34*sin(gamma_) //ok

[C]=inv([A,-B; B,A])*[E ; F]
theta1 = atan(C(2),C(1))
if (theta1 > %pi) then 
    theta1 = 2*%pi -theta1
end
theta3 = gamma_ - theta1 -theta2

[x_c,y_c,gamma_c]=jointPosition2EndEffectPose(theta1,theta2,theta3,L1,L2,g1,g2,g3)
//Checking if the inputs are inside the workspace of the robot
h=(x_c)^2 +(y_c)^2

if h < (L1+L2+g1+g3) then 
   if h > (L1-L2+g1+g3) then
     disp('checked')
   end
else 
   disp('not checked')
end



endfunction



function[J1,J2,J3,P_g1,P_g2,P] = ComputingJoints(theta1,theta2,gamma_,g1,g2,J1)
    //J1 = [0,0]
    J2 = [L1*cos(theta1),L1*sin(theta1)]
    J3 = [L1*cos(theta1)+ L2*cos(theta1+theta2),L1*sin(theta1)+L2*sin(theta1+theta2)]
    //adding the translation matrix 
    R_gamma = [cos(gamma_), -sin(gamma_); sin(gamma_), cos(gamma_)]

    //computing points
    P_g1=J3'+ R_gamma*[g1;0]
    P_g2=J3'+ R_gamma*[g1;0]+ R_gamma*[0;g2]
    P = J3'+ R_gamma*[g1+g3;0]+ R_gamma*[0;g2]


endfunction


function[] = PlottingPoints(J1,J2,J3,P_g1,P_g2,P)
    clf();
    xname("TriVex SL/SLi")
    a=get("current_axes");//get the handle of the newly created axes
    //a.grid=[1 -1]; //make x-grid
    //a.grid=[1 -1]; //make x-grid
    a.axes_visible ="on";
    a.x_location = "top";
    a.data_bounds=[-0.1,-0.7;1,0.1];
    a.axes_bounds=[0,0,1,0.6];
    h=figure;
    //datacursormode(h,'on');
    //a.view="2d";
    //plot2d([-0.2,1],[-1,0],[-1,-1],frameflag=3)
    xsegs([J1(1),J2(1)],[J1(2),J2(2)],1:1);
    xsegs([J2(1),J3(1)],[J2(2),J3(2)],1:1);
    xsegs([J3(1),P_g1(1)],[J3(2),P_g1(2)],1:1);
    xsegs([P_g1(1),P_g2(1)],[P_g1(2),P_g2(2)],1:1);
    xsegs([P_g2(1),P(1)],[P_g2(2),P(2)],1:1);
    //a=gce();
    s=a.children;
    s.mark_style = 9;
    s.mark_size = 5;
    s.segs_color = 2;
    s.thickness = 5;
    s.mark_mode= "on";
    xtitle("Plotting");
endfunction

function [J] = Jacobian(theta1,theta2,L1,L2)
    J = [[0,L1*sin(theta1),L1*sin(theta1)+L2*sin(theta1 + theta2)];[0,-L1*cos(theta1),-(L1*cos(theta1)+L2*cos(theta1 + theta2))];[1,1,1]];
endfunction

function [thetap] = ComputingVelocities(theta1,theta2,L1,L2,v)
    [J] = Jacobian(theta1,theta2,L1,L2)
    thetap = inv(J)*[0;v;0]
endfunction

function[t_,thetad_1,thetad_2,thetad_3] = PlottingVelocities(thetap,t,t_,thetad_1,thetad_2,thetad_3)
    b=get("current_axes");//get the handle of the newly created axes
    b.data_bounds=[0,-0.3;6,0.3];
    xlabel("t(s)")
    ylabel("Joint Velocities(rad/s)","fontsize",1,"color","black")

    t_ = [t_,t]
    thetad_1 = [thetad_1,thetap(1)]
    thetad_2 = [thetad_2,thetap(2)]
    thetad_3 = [thetad_3,thetap(3)]
    plot(t_,thetad_1,'b')
    plot(t_,thetad_2,'g')
    plot(t_,thetad_3,'r')
    hl=legend(['theta1_p';'theta2_p';'theta3_p'],"in_lower_right");
endfunction

function[T] = ComputingTorqueJ1(theta1,theta2,theta3,L1,L2,g1,g3,m_L1,m_L2,m_EF,n_prod,m_p,g,T)
    //p1=(L1/2)*cos(theta1)
    //p2=(L1/2)*cos(theta1)+(L2/2)*cos(theta1+theta2)
    //p3=(L1/2)*cos(theta1)+(L2/2)*cos(theta1+theta2)+(g1+g3/2)*cos(theta1+theta2+theta3)  
    //M3 = +(g1 + (g3/2))*[m_EF + (num_products*m_product)]*g*cos(theta1+theta2+theta3)
    //M2 = +(L2/2)*cos(theta1 + theta2)*m_L2*g - M3
    //M1 = +(L1/2)*cos(theta1)*m_L1*g - M2
    M3 = +(g1 + (g3/2))*[m_EF + (n_prod*m_p)]*g*cos(theta1+theta2+theta3)
    M2 = +(L2/2)*cos(theta1 + theta2)*m_L2*g - M3
    M1 = +(L1/2)*cos(theta1)*m_L1*g - M2
    
    //To=F1*p1+F2*p2+F3*p3
    T=[T,M1]
    
endfunction



function[] = PlottingTorque(t_,T)
    c=get("current_axes");//get the handle of the newly created axes
    c.data_bounds=[0,-0.3;6,0.3];
    xlabel("t(s)")
    ylabel("Torque(N·m)","fontsize",1,"color","black")
    //plot(t_,T,'ko-')
    plot(t_,T,'k')
    hl=legend(['J1_torque'],"in_lower_right");
endfunction


