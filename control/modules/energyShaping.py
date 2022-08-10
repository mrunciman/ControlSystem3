# SoftBellowControl_experiment(x,v,P1,P2,xd,dt,lin_mode,Mode,adapt,vel)   

# INPUT VARIABLES: 

# position of the payload 'x' in [m]; 
# velocity of the payload 'v' in [m/s];
# gauge pressure 'P1' in [Pa];
# gauge pressure 'P2' in [Pa];
# target position 'xd' in [m];
# sampling interval dt in [s];
# settings: adaptive, pos
#     lin_mode    % 1 for linear volume; 2 for quadratic volume; 3 for cubic volume
#     Mode        % 0 for multi-step energy shaping; 2 for backstepping
#     adapt       % 1 for adaptive observer; 0 without
#     vel         % 1 for velocity feedback; 0 position only


# OUTPUT VARIABLES: 

# control input U1 and U2 in [m^3/s]
# disturbance estimate F_obs in [N]

import math as mt

class energyShaping():
    def __init__(self):
        self.F_hat = 0
        self.x_p = 0
        self.x_dotp = 0
        self.x_dotfp = 0
        self.controlU = []
        self.A_SYRINGE = mt.pi*(13.25**2) # mm^2


    def energyShape(self, x, v, P1, P2, xd, dt, lin_mode, Mode, adapt, vel):

        ## discrete-time differentiation and low-pass filtering to compute velocity
        x_dot=(x - self.x_p)/dt
        x_dotf = 0.9391*self.x_dotfp + 0.03047*(x_dot+self.x_dotp)
        if vel == 1:
            v = x_dotf 
            self.x_p = x
            self.x_dotp = x_dot
            self.x_dotfp = x_dotf

        # IMPORT FROM KINEMATICS.PY
        ## model parameters
        L0 = 50/1000               # length of the bellow [m]
        m = 1.5                    # payload in [kg]
        eps = 1/10^4          
        x = min(x, L0/3 - eps)
        vol0 = 1/10^7              # dead volume of fluid (default)
        nl = 4
        Ds = 12/1000
        dc = 9/1000
        K_B = L0^2/nl*(dc/3 + Ds/2)


        R = 5                        # viscous friciton
        beta0 = 2*10^9               # bulk modulus of water in Pa
        rho = 1000                   # density of water in kg/m^3
        Patm = 10^5                  # atmospheric pressure in Pa
        A = (1.504/10^5)/(L0/3)      # area of linearized volume V(x)

            

        # volume of the bellows used in the system dynamics

        if lin_mode==1:
            # linear approximation

            vol1 = 1.504/10^5-A*x+vol0
            vol2 = A*x+vol0  
            dA1 = -A
            dA2 = A
            dA1x = 0
            dA2x = 0


        elif lin_mode==2:
            # quadratic approximation

            vol1 = vol0 + 3*K_B*(1/6 - ((2*L0)/3 + x)/(6*L0))^(1/2)
            vol2 = vol0 + 3*K_B*(1/6 - (L0 - x)/(6*L0))^(1/2)
            dA1 = -K_B/(4*L0*(1/6 - ((2*L0)/3 + x)/(6*L0))^(1/2))
            dA2 = K_B/(4*L0*(1/6 - (L0 - x)/(6*L0))^(1/2))
            dA1x = -K_B/(48*L0^2*(1/6 - ((2*L0)/3 + x)/(6*L0))^(3/2))
            dA2x = -K_B/(48*L0^2*(1/6 - (L0 - x)/(6*L0))^(3/2))


        elif lin_mode==3:
            # cubic approximation

            vol1 = vol0 + K_B*((2*L0 + 3*x)/(2*L0) + 1/2)*(2/3 - ((4*L0)/3 + 2*x)/(3*L0))^(1/2)      # decreases with x
            vol2 = vol0 + K_B*((3*L0 - 3*x)/(2*L0) + 1/2)*(2/3 - (2*L0 - 2*x)/(3*L0))^(1/2)          # increases with x
            dA1 = (3*K_B*(2/3 - ((4*L0)/3 + 2*x)/(3*L0))^(1/2))/(2*L0) - (K_B*((2*L0 + 3*x)/(2*L0) + 1/2))/(3*L0*(2/3 - ((4*L0)/3 + 2*x)/(3*L0))^(1/2))
            dA2 = (K_B*((3*L0 - 3*x)/(2*L0) + 1/2))/(3*L0*(2/3 - (2*L0 - 2*x)/(3*L0))^(1/2)) - (3*K_B*(2/3 - (2*L0 - 2*x)/(3*L0))^(1/2))/(2*L0)
            dA1x = - K_B/(L0^2*(2/3 - ((4*L0)/3 + 2*x)/(3*L0))^(1/2)) - (K_B*((2*L0 + 3*x)/(2*L0) + 1/2))/(9*L0^2*(2/3 - ((4*L0)/3 + 2*x)/(3*L0))^(3/2))
            dA2x = - K_B/(L0^2*(2/3 - (2*L0 - 2*x)/(3*L0))^(1/2)) - (K_B*((3*L0 - 3*x)/(2*L0) + 1/2))/(9*L0^2*(2/3 - (2*L0 - 2*x)/(3*L0))^(3/2))


        M = m + rho*vol1 + rho*vol2
        p0 = M*v


        P1g = P1-Patm   
        P2g = P2-Patm


        ## tuning parameters

        kp = 1       # 1 or 2 (default)
        Ki = 10      # 10 (default) or 20
        Ki2 = 10     # 10 (default) or 20
        K_obs = 10   # 10 (default) or 20
        Km = 2       # 2 or 4 (default)


        ## nonlinear observer

        dF_hat = K_obs*(P1g*dA1 - self.F_hat + P2g*dA2)+(p0*K_obs*\
            (2*m^2*K_obs \
            - 2*R*m + 2*rho^2*vol1^2*K_obs \
            + 2*rho^2*vol2^2*K_obs \
            - 2*R*rho*vol1 \
            - 2*R*rho*vol2 \
            + dA1*p0*rho \
            + dA2*p0*rho \
            + 4*rho^2*vol1*vol2*K_obs \
            + 4*m*rho*vol1*K_obs \
            + 4*m*rho*vol2*K_obs))/(2*(m + rho*vol1 + rho*vol2)^2)

        self.F_hat = self.F_hat + dF_hat*dt


        if Mode==0:
        ## energy shaping control law

            if adapt==1:
                F_obs = self.F_hat - p0*K_obs

            else:
                F_obs = 0    

            U1 = (dA1*p0)/(m + rho*vol1 + rho*vol2) \
                - (vol1*((Ki*(P1g*dA1 - F_obs + P2g*dA2 + Km*kp*(x - xd)))/dA1 \
                + (p0*(Km*(P1g*dA1x + P2g*dA2x + Km*kp) + 1))/(2*Km*dA1*(m + rho*vol1 + rho*vol2))))/beta0

            U2 = (dA2*p0)/(m + rho*vol1 + rho*vol2) \
                - (vol2*((Ki2*(P1g*dA1 - F_obs + P2g*dA2 + Km*kp*(x - xd)))/dA2 \
                + (p0*(Km*(P1g*dA1x + P2g*dA2x + Km*kp) + 1))/(2*Km*dA2*(m + rho*vol1 + rho*vol2))))/beta0

        
        elif Mode==2:
        # backstepping control law

            if adapt==1:
                F_obs = self.F_hat-p0*K_obs

            else:
                F_obs = 0

            U1 = -(vol1*(Ki*(P1g - (F_obs - Km*kp*(x - xd))/(2*dA1)) \
                + (dA1*v)/Km \
                + (dA1x*v*(F_obs - Km*kp*(x - xd)))/(2*dA1^2) \
                + (Km*kp*v)/(2*dA1) - (beta0*dA1*v)/vol1))/beta0

            U2 = -(vol2*(Ki2*(P2g - (F_obs- Km*kp*(x - xd))/(2*dA2)) \
                + (dA2*v)/Km \
                + (dA2x*v*(F_obs - Km*kp*(x - xd)))/(2*dA2^2) \
                +(Km*kp*v)/(2*dA2) - (beta0*dA2*v)/vol2))/beta0


        self.controlU[0] = U1
        self.controlU[1] = U2
        self.controlU[2] = F_obs
        return self.controlU

    def traject(self, x1_current, x2_current, dt):
        U1 = self.controlU[1]
        U2 = self.controlU[2]

        x1_s_ast = x1_current + (32*U1*dt)/(30*self.A_SYRINGE)
        x2_s_ast = x2_current + (32*U2*dt)/(30*self.A_SYRINGE)
        return x1_s_ast, x2_s_ast