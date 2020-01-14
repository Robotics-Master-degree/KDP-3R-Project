clear
clf();
getd('~/KDP-3R-Project')

//constraints - all in International System of Units
L1=0.62 //m
L2=0.57 //m
g1=0.1 //m
g2=-0.2 //m
g3=0.3 //m
J1 = [0,0] //m
m_L1=4 //kg
m_L2=3 //kg
m_EF=2 //kg
m_product=1 //kg
g=9.81 //m/s² 

//initial position
P_Start = [0.9,-0.2,0] //m
//final position
P_Final = [0.9,-0.7,0] //m
///Computing the time to make all movement
dist =abs(P_Final(2)) - abs(P_Start(2)) //m
v=-0.1 //velocity of the end-effector
total_time = abs(dist/v) //s


//Computing the positions
discr = 0.02 //discretization
t=0 //initial time //s
t_= [] //array of times
thetad_1 = [] //array of theta_1 [rad]
thetad_2 = [] //arrat oy theta_2 [rad]
thetad_3 = [] //array of theta_3 [rad]
T=[]  //array of torques
//2.1 Compute the initial pose of the end-effector
P_Computed = P_Start 
while t<=dist/abs(v)

   //2.2 Solve the inverse position analysis to obtain the configuration of the robot and, in particular, the position of each joint.
   [theta1,theta2,theta3] = Endeffector2jointposition(P_Computed(1),P_Computed(2),P_Computed(3),g1,g2,g3,L1,L2)

   //Computing each joint
   [J1,J2,J3,P_g1,P_g2,P] = ComputingJoints(theta1,theta2,P_Computed(3),g1,g2,J1)   
   //1. Plotting 3R Motion
   PlottingPoints(J1,J2,J3,P_g1,P_g2,P)    
   //2.1 Computing the pose of the end-effector in each iteration
   t=t+discr; //computing new time
   P_Computed = P_Start'+ [0;v;0]*t;
   P_Computed = P_Computed';
  

    //Printing variables in scilab IDE
    printf(' t %f:',t)
    printf(' P %f:',P(2))
    
    //2.3 Compute the jacobian at this configuration //2.4 Obtain joint speeds by multiplying the jacobian with the required end-effector twist(joint speeds = J·T)
    [thetap] = ComputingVelocities(theta1,theta2,L1,L2,v)

    //2 .Plotting joint speeds versus time
    xsetech([0,0.50,1,0.3])
    [t_,thetad_1,thetad_2,thetad_3] = PlottingVelocities(thetap,t,t_,thetad_1,thetad_2,thetad_3)

    //3. Computing Torque in joint J1
    xsetech([0,0.75,1,0.25])
    num_products=int(t) //as there is a product each second, it gets the number of products that each iteration has
    //printf(' num_products %i:',num_products)

    //3.1 ComputingTorque in J1
    [T] = ComputingTorqueJ1(theta1,theta2,theta3,L1,L2,g1,g3,m_L1,m_L2,m_EF,num_products,m_product,g,T)

    //3.2 Plotting Torque
    PlottingTorque(t_,T)

    //
   
end








