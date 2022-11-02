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

# control input U1 and U2 in [m**3/s]
# disturbance estimate F_obs in [N]



# Position and speed of payload will come from optitrack with 120 Hz sampling rate
# Pressures will come from arduinos, smapling rate 20.83 Hz (1/48 ms)
# dt only affects nonlinear observer


import math as mt

class energyShaper():
    def __init__(self):
        self.F_hat = 0
        self.x_p = 0
        self.x_dotp = 0
        self.x_dotfp = 0
        self.controlU = [0, 0, 0, 0]
        self.first_flag = True

        self.x1_s_ast = 0 
        self.x2_s_ast = 0

        self.x1_s_ast_p = 0
        self.x2_s_ast_p = 0



    def trackToState(self, markerData, dt):
        '''
        Find position and velocity of payload from optitrack markers
        '''
        # opTrack.markerData = [self.timeStamp] + [self.timeRunning] + [X, Y, Z]*n for n markers

        # Find distance between individual markers
        # Use 120 Hz data from stream
        # Find speed by discrete time differentiation
        # Map displacement of markers to coordinate system of energy shaping 

        # FRAMERATE = 1/120
        ZEROPOINT = 117.5/1000       # distance between markers that I define as zero,  in m

        markers = markerData
        dataLen = len(markers)
        if dataLen >= 8:
            x1 = markers[2]
            y1 = markers[3]
            z1 = markers[4]

            x2 = markers[5]
            y2 = markers[6]
            z2 = markers[7]

            dist = mt.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

            # Map to coordinate system
            x_obs = dist - ZEROPOINT

            # ## discrete-time differentiation and low-pass filtering to compute velocity
            if self.first_flag:
                x_dot = 0
                self.first_flag = False
            else:
                x_dot = (x_obs - self.x_p)/dt
            x_dotf = 0.9391*self.x_dotfp + 0.03047*(x_dot + self.x_dotp)
            v_obs = x_dotf 
            self.x_p = x_obs
            self.x_dotp = x_dot
            self.x_dotfp = x_dotf


        else:
            x_obs, v_obs = None, None


        return x_obs, v_obs



    def energyShape(self, x, v, P1, P2, xd, dt, lin_mode, Mode, adapt):
        '''
        position of the payload 'x' in [m]; 
        velocity of the payload 'v' in [m/s];
        gauge pressure 'P1' in [Pa];
        gauge pressure 'P2' in [Pa];
        target position 'xd' in [m];
        sampling interval dt in [s];
        settings:
            lin_mode    % 1 for linear volume; 2 for quadratic volume; 3 for cubic volume
            Mode        % 0 for multi-step energy shaping; 2 for backstepping
            adapt       % 1 for adaptive observer; 0 without
        '''
        simple = 0

        ## model parameters
        L0 = 30/1000                 # length of the bellow [m]
        MAX_STROKE = L0/4
        M_P = 1.5                    # payload in [kg]
        EPSILON = 1/10**4 
        VOL_0 = 1/10**7              # dead volume of fluid (default)
        NUM_L = 3
        D_S = 12/1000
        D_C = 9/1000
        VOL_FACTOR = 1.05
        K_B = VOL_FACTOR*(L0**2/NUM_L)*(D_C/3 + D_S/2)

        R = 5                         # viscous friciton
        BETA_0 = 2*10**9              # bulk modulus of water in Pa
        RHO = 1000                    # density of water in kg/m**3
        P_ATM = 10**5                 # atmospheric pressure in Pa
        A = (1.504/10**5)/(L0/3)      # area of linearized volume V(x)


        if x == None:
            self.controlU[0] = 0
            self.controlU[1] = 0
            self.controlU[2] = 0
            return self.controlU

        offset0=-0.02
        k0=0

        # x = min(x, L0/4 - EPSILON)
        if abs(x) > MAX_STROKE:
            if x<0:
                sign_x = -1
            else:
                sign_x = 1
            x = sign_x*(MAX_STROKE - EPSILON)


        # volume of the bellows used in the system dynamics

        if lin_mode==1:
            # linear approximation

            vol1 = 1.504/10**5-A*x+VOL_0
            vol2 = A*x+VOL_0  
            dA1 = -A
            dA2 = A
            dA1x = 0
            dA2x = 0


        elif lin_mode==2:
            # quadratic approximation

            vol1 = VOL_0 + 3*K_B*(1/6 - ((2*L0)/3 + x)/(6*L0))**(1/2)
            vol2 = VOL_0 + 3*K_B*(1/6 - (L0 - x)/(6*L0))**(1/2)
            dA1 = -K_B/(4*L0*(1/6 - ((2*L0)/3 + x)/(6*L0))**(1/2))
            dA2 = K_B/(4*L0*(1/6 - (L0 - x)/(6*L0))**(1/2))
            dA1x = -K_B/(48*L0**2*(1/6 - ((2*L0)/3 + x)/(6*L0))**(3/2))
            dA2x = -K_B/(48*L0**2*(1/6 - (L0 - x)/(6*L0))**(3/2))


        elif lin_mode==3:
            # cubic approximation

            vol1 = VOL_0 + K_B*((2*L0 + 3*x)/(2*L0) + 1/2)*(2/3 - ((4*L0)/3 + 2*x)/(3*L0))**(1/2)      # decreases with x
            vol2 = VOL_0 + K_B*((3*L0 - 3*x)/(2*L0) + 1/2)*(2/3 - (2*L0 - 2*x)/(3*L0))**(1/2)          # increases with x
            dA1 = (3*K_B*(2/3 - ((4*L0)/3 + 2*x)/(3*L0))**(1/2))/(2*L0) - (K_B*((2*L0 + 3*x)/(2*L0) + 1/2))/(3*L0*(2/3 - ((4*L0)/3 + 2*x)/(3*L0))**(1/2))
            dA2 = (K_B*((3*L0 - 3*x)/(2*L0) + 1/2))/(3*L0*(2/3 - (2*L0 - 2*x)/(3*L0))**(1/2)) - (3*K_B*(2/3 - (2*L0 - 2*x)/(3*L0))**(1/2))/(2*L0)
            dA1x = - K_B/(L0**2*(2/3 - ((4*L0)/3 + 2*x)/(3*L0))**(1/2)) - (K_B*((2*L0 + 3*x)/(2*L0) + 1/2))/(9*L0**2*(2/3 - ((4*L0)/3 + 2*x)/(3*L0))**(3/2))
            dA2x = - K_B/(L0**2*(2/3 - (2*L0 - 2*x)/(3*L0))**(1/2)) - (K_B*((3*L0 - 3*x)/(2*L0) + 1/2))/(9*L0**2*(2/3 - (2*L0 - 2*x)/(3*L0))**(3/2))


        elif lin_mode == 4:
            # Here volume is dependent on the contraction, not position!
            con1 = x + MAX_STROKE/2
            if con1 > MAX_STROKE:
                con1 = MAX_STROKE - EPSILON
            vol1 = VOL_0 + (6**(1/2)*K_B*((L0 - 4*con1)/L0)**(1/2)*(380*L0**2 + 480*L0*con1 + 192*con1**2))/(1536*L0**2)  # decreases with x
            dA1=(6**(1/2)*K_B*((L0 - 4*con1)/L0)**(1/2)*(480*L0 + 384*con1))/(1536*L0**2) - (6**(1/2)*K_B*(380*L0**2 + 480*L0*con1 + 192*con1**2))/(768*L0**3*((L0 - 4*con1)/L0)**(1/2))
            dA1x=(6**(1/2)*K_B*((L0 - 4*con1)/L0)**(1/2))/(4*L0**2) - (6**(1/2)*K_B*(380*L0**2 + 480*L0*con1 + 192*con1**2))/(384*L0**4*((L0 - 4*con1)/L0)**(3/2)) - (6**(1/2)*K_B*(480*L0 + 384*con1))/(384*L0**3*((L0 - 4*con1)/L0)**(1/2))
            
            con2 = x - MAX_STROKE/2
            if con2 > MAX_STROKE:
                con2 = MAX_STROKE + EPSILON
            vol2 = VOL_0 + (6**(1/2)*K_B*((L0 + 4*con2)/L0)**(1/2)*(380*L0**2 - 480*L0*con2 + 192*con2**2))/(1536*L0**2)  # inreases with x
            dA2=(6**(1/2)*K_B*(380*L0**2 - 480*L0*con2 + 192*con2**2))/(768*L0**3*((L0 + 4*con2)/L0)**(1/2)) - (6**(1/2)*K_B*((L0 + 4*con2)/L0)**(1/2)*(480*L0 - 384*con2))/(1536*L0**2)
            dA2x=(6**(1/2)*K_B*((L0 + 4*con2)/L0)**(1/2))/(4*L0**2) - (6**(1/2)*K_B*(380*L0**2 - 480*L0*con2 + 192*con2**2))/(384*L0**4*((L0 + 4*con2)/L0)**(3/2)) - (6**(1/2)*K_B*(480*L0 - 384*con2))/(384*L0**3*((L0 + 4*con2)/L0)**(1/2))
    
        elif lin_mode == 5:
            # vol1 = VOL_0 + (3**(1/2)*K_B*(1 - (8*x)/L0)**(1/2)*(443*L0**2 + 528*L0*x + 192*x**2))/(1536*L0**2)  # decreases with x
            # vol2 = VOL_0 + (3**(1/2)*K_B*((8*x)/L0 + 1)**(1/2)*(443*L0**2 - 528*L0*x + 192*x**2))/(1536*L0**2)  # inreases with x
            # dA1=(3**(1/2)*K_B*(528*L0 + 384*x)*(1 - (8*x)/L0)**(1/2))/(1536*L0**2) - (3**(1/2)*K_B*(443*L0**2 + 528*L0*x + 192*x**2))/(384*L0**3*(1 - (8*x)/L0)**(1/2))
            # dA2=(3**(1/2)*K_B*(443*L0**2 - 528*L0*x + 192*x**2))/(384*L0**3*((8*x)/L0 + 1)**(1/2)) - (3**(1/2)*K_B*(528*L0 - 384*x)*((8*x)/L0 + 1)**(1/2))/(1536*L0**2)
            # dA1x=(3**(1/2)*K_B*(1 - (8*x)/L0)**(1/2))/(4*L0**2) - (3**(1/2)*K_B*(443*L0**2 + 528*L0*x + 192*x**2))/(96*L0**4*(1 - (8*x)/L0)**(3/2)) - (3**(1/2)*K_B*(528*L0 + 384*x))/(192*L0**3*(1 - (8*x)/L0)**(1/2))
            # dA2x=(3**(1/2)*K_B*((8*x)/L0 + 1)**(1/2))/(4*L0**2) - (3**(1/2)*K_B*(443*L0**2 - 528*L0*x + 192*x**2))/(96*L0**4*((8*x)/L0 + 1)**(3/2)) - (3**(1/2)*K_B*(528*L0 - 384*x))/(192*L0**3*((8*x)/L0 + 1)**(1/2))

            vol1 = VOL_0 + (K_B*(3 - (24*offset0 + 24*x)/L0)**(1/2)*(443*L0**2 + 528*L0*x + 192*x**2))/(1536*L0**2)
            vol2 = VOL_0 + (K_B*(3 - (24*offset0 - 24*x)/L0)**(1/2)*(443*L0**2 - 528*L0*x + 192*x**2))/(1536*L0**2)
            dA1=(K_B*(3 - (24*offset0 + 24*x)/L0)**(1/2)*(528*L0 + 384*x))/(1536*L0**2) - (K_B*(443*L0**2 + 528*L0*x + 192*x**2))/(128*L0**3*(3 - (24*offset0 + 24*x)/L0)**(1/2))
            dA2=(K_B*(443*L0**2 - 528*L0*x + 192*x**2))/(128*L0**3*(3 - (24*offset0 - 24*x)/L0)**(1/2)) - (K_B*(3 - (24*offset0 - 24*x)/L0)**(1/2)*(528*L0 - 384*x))/(1536*L0**2)
            dA1x=(K_B*(3 - (24*offset0 + 24*x)/L0)**(1/2))/(4*L0**2) - (K_B*(528*L0 + 384*x))/(64*L0**3*(3 - (24*offset0 + 24*x)/L0)**(1/2)) - (3*K_B*(443*L0**2 + 528*L0*x + 192*x**2))/(32*L0**4*(3 - (24*offset0 + 24*x)/L0)**(3/2))
            dA2x=(K_B*(3 - (24*offset0 - 24*x)/L0)**(1/2))/(4*L0**2) - (K_B*(528*L0 - 384*x))/(64*L0**3*(3 - (24*offset0 - 24*x)/L0)**(1/2)) - (3*K_B*(443*L0**2 - 528*L0*x + 192*x**2))/(32*L0**4*(3 - (24*offset0 - 24*x)/L0)**(3/2))



        M = M_P + RHO*vol1 + RHO*vol2
        p0 = M*v


        P1g = 1*(P1 - 109960)
        P2g = 1*(P2 - 128290)


        ## tuning parameters

        kp = 4       # 1 or 2 (default)
        Ki = 10      # 10 (default) or 20
        Ki2 = 10     # 10 (default) or 20
        K_obs = 10   # 10 (default) or 20
        Km = 4       # 2 or 4 (default)


        ## nonlinear observer
        #Simplified observer
        if simple == True:
            dF_hat= K_obs*(self.F_hat + P1g - P2g - K_obs*p0)
        else:
            # dF_hat = K_obs*(P1g*dA1 - self.F_hat + P2g*dA2)+(p0*K_obs\
            #     *(2*M_P**2*K_obs \
            #     - 2*R*M_P + 2*RHO**2*vol1**2*K_obs \
            #     + 2*RHO**2*vol2**2*K_obs \
            #     - 2*R*RHO*vol1 \
            #     - 2*R*RHO*vol2 \
            #     + dA1*p0*RHO \
            #     + dA2*p0*RHO \
            #     + 4*RHO**2*vol1*vol2*K_obs \
            #     + 4*M_P*RHO*vol1*K_obs \
            #     + 4*M_P*RHO*vol2*K_obs))\
            #     /(2*(M_P + RHO*vol1 + RHO*vol2)**2)

            dF_hat=(K_obs*(\
                2*P1g*dA1 \
                - 2*self.F_hat \
                + 2*P2g*dA2 \
                - 2*R*v \
                - 2*k0*x \
                + dA1*RHO*v**2 \
                + dA2*RHO*v**2 \
                + 2*M_P*v*K_obs \
                + 2*RHO*v*vol1*K_obs \
                + 2*RHO*v*vol2*K_obs))/2


        self.F_hat = self.F_hat + dF_hat*dt


        if Mode==0:
        ## energy shaping control law

            if adapt==1:
                F_obs = (self.F_hat - p0*K_obs)

            else:
                F_obs = 0    
            
            # Simplified expression for individual pressures
            if simple == True:
                mu=1
                U1=-(Ki*(P1g - (kp*(x - xd))/2 + mu*F_obs))/BETA_0
                U2=-(Ki2*(P2g + F_obs*(mu - 1) + (kp*(x - xd))/2))/BETA_0

            else:
                # U1 = (dA1*p0)/(M_P + RHO*vol1 + RHO*vol2) \
                #     - (vol1*((Ki*(P1g*dA1 - F_obs + P2g*dA2 + Km*kp*(x - xd)))/dA1 \
                #     + (p0*(Km*(P1g*dA1x + P2g*dA2x + Km*kp) + 1))/(2*Km*dA1*(M_P + RHO*vol1 + RHO*vol2))))/BETA_0

                # U2 = (dA2*p0)/(M_P + RHO*vol1 + RHO*vol2) \
                #     - (vol2*((Ki2*(P1g*dA1 - F_obs + P2g*dA2 + Km*kp*(x - xd)))/dA2 \
                #     + (p0*(Km*(P1g*dA1x + P2g*dA2x + Km*kp) + 1))/(2*Km*dA2*(M_P + RHO*vol1 + RHO*vol2))))/BETA_0

                U1=(dA1*p0)/(M_P + RHO*vol1 + RHO*vol2) \
                    - (vol1*((Ki*(P1g*dA1 - F_obs + P2g*dA2 - k0*x - Km*kp*1000*(x - xd)))/dA1\
                    + (p0*(Km*(P1g*dA1x - k0 + P2g*dA2x + Km*kp) + 1))/(2*Km*dA1*(M_P + RHO*vol1 + RHO*vol2))))/BETA_0

                U2=(dA2*p0)/(M_P + RHO*vol1 + RHO*vol2) \
                    - (vol2*((Ki2*(P1g*dA1 - F_obs + P2g*dA2 - k0*x - Km*kp*1000*(x - xd)))/dA2 \
                    + (p0*(Km*(P1g*dA1x - k0 + P2g*dA2x + Km*kp) + 1))/(2*Km*dA2*(M_P + RHO*vol1 + RHO*vol2))))/BETA_0    


        
        # elif Mode==2:
        # # backstepping control law

        #     if adapt==1:
        #         F_obs = self.F_hat-p0*K_obs

        #     else:
        #         F_obs = 0

        #     U1 = -(vol1*(Ki*(P1g - (F_obs - Km*kp*(x - xd))/(2*dA1)) \
        #         + (dA1*v)/Km \
        #         + (dA1x*v*(F_obs - Km*kp*(x - xd)))/(2*dA1**2) \
        #         + (Km*kp*v)/(2*dA1) - (BETA_0*dA1*v)/vol1))/BETA_0

        #     U2 = -(vol2*(Ki2*(P2g - (F_obs- Km*kp*(x - xd))/(2*dA2)) \
        #         + (dA2*v)/Km \
        #         + (dA2x*v*(F_obs - Km*kp*(x - xd)))/(2*dA2**2) \
        #         +(Km*kp*v)/(2*dA2) - (BETA_0*dA2*v)/vol2))/BETA_0

        self.controlU[0] = U1
        self.controlU[1] = U2
        self.controlU[2] = F_obs
        self.controlU[3] = self.F_hat
        return self.controlU, vol1, vol2




    def traject(self, steps1_current, steps2_current, dt):

        STEPS_PER_MM = 400
        A_SYRINGE = mt.pi*((13.25)**2) # m**2
        # M_TO_MM = 1000
        MCUBE_TO_MMCUBE = 1e9

        U1 = self.controlU[0]*MCUBE_TO_MMCUBE # in mm**3/s
        U2 = self.controlU[1]*MCUBE_TO_MMCUBE

        k_U = 6

        self.x1_s_ast_p = steps1_current
        self.x2_s_ast_p = steps2_current

        delta_x1_s = k_U*((32*U1*dt)/(30*A_SYRINGE))*STEPS_PER_MM
        delta_x2_s = k_U*((32*U2*dt)/(30*A_SYRINGE))*STEPS_PER_MM

        # print("Deltas: ", delta_x1_s, delta_x2_s)
        deltas = [delta_x1_s, delta_x2_s]
        for i in range(len(deltas)):
            if abs(deltas[i]) > 0.1:
                if deltas[i] > 0:
                    newVal = max(deltas[i], 1)
                    deltas[i] = newVal
                else:
                    newVal = -max(abs(deltas[i]), 1)
                    deltas[i] = newVal

        delta_x1_s = deltas[0]
        delta_x2_s = deltas[1]

        # print("Deltas: ", delta_x1_s, delta_x2_s)

        self.x1_s_ast = self.x1_s_ast_p + delta_x1_s
        self.x2_s_ast = self.x2_s_ast_p + delta_x2_s

        # self.x1_s_ast_p = self.x1_s_ast
        # self.x2_s_ast_p = self.x2_s_ast

        step_1 = round(self.x1_s_ast)
        step_2 = round(self.x2_s_ast)

        return step_1, step_2




    def pressFilt(self, Pf_prev, P, P_prev):
        Pf= 0.854*Pf_prev + 0.073*(P + P_prev)
        return Pf