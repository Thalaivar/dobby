import matplotlib.pyplot as plt
from scipy.integrate import ode
import numpy as np
from numpy import sin,cos,tan,zeros,exp,tanh,dot

def zetta(x1,x2,x3,x4,x5,x6,spr,c):
    mu1,mu2,mu3,mu4,mu5,mu6,mu7,mu8,mu9,mu10,mu11,mu12=np.zeros(len(c)),np.zeros(len(c)),np.zeros(len(c)),np.zeros(len(c)),np.zeros(len(c)),np.zeros(len(c)),np.zeros(len(c)),np.zeros(len(c)),np.zeros(len(c)),np.zeros(len(c)),np.zeros(len(c)),np.zeros(len(c))
		m=[]
    for i in range(len(c)):
        mu1[i]=exp(-0.5*((x1-c[i])/spr)**2)
        mu2[i]=exp(-0.5*((x2-c[i])/spr)**2)
        mu3[i]=exp(-0.5*((x3-c[i])/spr)**2)
        mu4[i]=exp(-0.5*((x4-c[i])/spr)**2)
        mu5[i]=exp(-0.5*((x4-c[i])/spr)**2)
        mu6[i]=exp(-0.5*((x4-c[i])/spr)**2)
        for i in range(len(c)):
        	for j in range(len(c)):
            for k in range(len(c)):
               for l in range(len(c)):
                 for mi in range(len(C)):
                   for n in range(len(C)):
										 for o in range(len(C)):
											 for p in range(len(C)):
												 for q in range(len(C)):
												 	 for r in range(len(C)):
													 	 for s in range(len(C)):
														 	 for t in range(len(C)):
															 	 m.append(mu1[i]*mu2[j]*mu3[k]*mu4[l]*mu5[mi]*mu6[n]*mu7[o]*mu8[p]*mu9[q]*mu10[r]*mu11[s]*mu12[t])
    m=np.array(m)
    S=np.sum(m)
    return m/S
    
    
def f(t,Y,C,spr):
    x1,x2,y1,y2,z1,z2,phi1,phi2,theta1,theta2,psi1,psi2=Y[0],Y[1],Y[2],Y[3],Y[4],Y[5],Y[6],Y[7],Y[8],Y[9],Y[10],Y[11] 
 
		a1,a2,a3,a4=Y[12:12+len(C)**12],Y[12+len(C)**12:12+2*len(C)**12],Y[12+2*len(C)**12:12+3*len(C)**12],Y[12+3*len(C)**12:]
    theta1dot,theta2dot,theta3dot,theta4dot=np.zeros(len(C)**12),np.zeros(len(C)**12),np.zeros(len(C)**12),np.zeros(len(C)**12)
		dtr,T1,T2,m,g,B,rho=np.pi/180,5,5,5,9.8,6,1.1
    CL,CY,CD,Cl,Cm,Cn=1.2,1.1,0.4,1.3,1.5,1.2
    Jx,Jy,Jz,Jxz,S=.3,10,8,4.3,10
    b,az,ax,dx,bx,bz,c,z,x,c1,c2,c3,K0,K,eta,epsilon,sigma=.4,.1,.2,3,.1,.2,.12,.09,.2,1,2,3,5,1,100,.01,.001
    Q=.5*rho*V**2
       # eta1,eta2,zeta=1,.2,5*dtr
    #sin(gamma), sin(mu)cos(gamma) and all
    #sg=(cos(alpha)*cos(beta)*sin(theta))-(sin(beta)*sin(phi)*cos(theta))-(sin(alpha)*cos(beta)*cos(phi)*cos(theta))
    #smcg=(sin(theta)*cos(alpha)*sin(beta))+(sin(phi)*cos(theta)*cos(beta))-(sin(alpha)*sin(beta)*cos(phi)*cos(theta))
    #cmcg=(sin(theta)*sin(alpha))+(cos(alpha)*cos(phi)*cos(theta))
    #import pdb;pdb.set_trace()
    #sliding surface and control design
    e1,e2,e3,e4=x1-sin(t),y1-sin(t),z1-sin(t),psi1-sin(t)
   	de1,de2,de3,de4=x2-cos(t),y2-cos(t),z2-cos(t),psi2-cos(t)
    s1,s2,s3=c1*e1+de1,c2*e2+de2,c3*e3+de3,c4*e4+de4
    Z=zetta(x1,x2,y1,y2,z1,z2,phi1,phi2,theta1,theta2,psi1,psi2,spr,C)
    T1=dot(Z,a1)
    T2=dot(Z,a2)
    T3=dot(Z,a3)
		T4=dot(Z,a4)
 
    #print('the value of p is: ',p)
		Hy1,Hy2,Hy3,Hy4=lambda1*T1,lambda2*T2,lambda3*T3,lambda4*T4
    H=Hy1+Hy2+Hy3+Hy4
    #dynamical equations
    x1dot=x2
		x2dot=
		y1dot=y2
		z1dot=z2
		z2dot=
		phi1dot=phi2
		phi2dot=(1/Jx)*(theta2*phi2*(Jy-Jz)+Jr*theta2*omegar+l(-T2+T4)-h*(H)+Rmx1-Rmx2+Rmx3-Rmx4)
		theta1dot=theta2
		theta2dot=(1/Jy)*(phi2*psi2*(Jz-Jx)-Jr*phi2*omegar+l*(T1-T3)+h*H+Rmx1-Rmx2+Rmx3-Rmx4)
		psi1dot=psi2
		psi2dot=()
				
		for i in range(len(C)**12):
        theta1dot[i]=-eta*Z[i]*s1
        theta2dot[i]=-eta*Z[i]*s2
        #import pdb;pdb.set_trace()
        theta3dot[i]=-eta*Z[i]*s3
				theta4dot[i]=-eta*Z[i]*s4

    f=np.array([vdot,alphadot,betadot,pdot,qdot,rdot,phidot,thetadot,psidot])
    f1=np.concatenate((f,theta1dot),axis=0)
    f2=np.concatenate((f1,theta2dot),axis=0)
    f3=np.concatenate((f2,theta3dot),axis=0)
		f4=np.concatenate((f3,theta4dot),axis=0)

    return f3

def solver(t0,y0,dt,t1,C,spr):
    #plt.ion()
    x,t=[[] for i in range(9+3*len(C)**9)],[]
    r=ode(f).set_integrator('dopri5',method='bdf')
    r.set_initial_value(y0,t0).set_f_params(C,spr)
    while r.successful() and r.t<t1:
        r.integrate(r.t+dt)
        for i in range(9+3*len(C)**9):
            x[i].append(r.y[i])
        t.append(r.t)
        #plt.scatter(r.t,r.y[1])
        #plt.draw()
        #plt.pause(.0001)

    return x,t

if __name__=='__main__':
    names=["V","alpha","beta","p","q","r","phi","theta","psi"]
    x10=np.array([.2,0.1,0,0,0,0,0,0.1,0])
    C=[-1.5,0,1.5]
    theta1=np.zeros(3*len(C)**9)
    x0=np.concatenate((x10,theta1),axis=0 ) 
    x,t=solver(0.0,x0,1e-2,10,C,.6)
    plt.figure(1)
    for i in range(9):
         plt.subplot(2,5,i+1)
         plt.plot(t,x[i],label=names[i])
         plt.legend(loc='upper right')
    plt.savefig('amar.png')
    plt.figure(2)
    for i in range(3*len(C)**9):
        plt.plot(t,x[9+i])
    plt.show()
