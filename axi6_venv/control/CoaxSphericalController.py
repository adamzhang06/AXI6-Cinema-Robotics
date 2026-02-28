import numpy as np
from scipy.optimize import fsolve

class CoaxSphericalController:
    def __init__(self, alpha1_deg, alpha2_deg, beta_deg):
        # Convert inputs to radians
        self.alpha1 = np.radians(alpha1_deg)
        self.alpha2 = np.radians(alpha2_deg)
        self.beta = np.radians(beta_deg)
        
        # Calculate alpha3
        self.alpha3 = 2 * np.arcsin(np.sin(self.beta) * np.cos(np.pi / 6)) 
        
        # Define eta_i for i = 1, 2, 3 
        self.eta = [2 * (i - 1) * np.pi / 3 for i in range(1, 4)]

    def _calculate_wi(self, theta_rad):
        # Calculate w_i of intermediate joints based on theta_rad and eta_i
        w = []
        for i in range(3):
            wi = np.array([
                np.sin(self.eta[i] - theta_rad[i]) * np.sin(self.alpha1), 
                np.cos(self.eta[i] - theta_rad[i]) * np.sin(self.alpha1),
                -np.cos(self.alpha1)                                      
            ])
            w.append(wi)
        return w

    def forward_kinematics(self, theta_deg, x0):
        theta_rad = np.radians(theta_deg)
        w = self._calculate_wi(theta_rad)

        def equations(vars):
            # vars contains the 9 components of v1, v2, v3 
            v1 = vars[0:3]
            v2 = vars[3:6]
            v3 = vars[6:9]
            
            # System of equations 
            eqs = [
                np.dot(w[0], v1) - np.cos(self.alpha2),    # w1 * v1 = cos(alpha2) 
                np.dot(w[1], v2) - np.cos(self.alpha2),    # w2 * v2 = cos(alpha2) 
                np.dot(w[2], v3) - np.cos(self.alpha2),    # w3 * v3 = cos(alpha2) 

                np.dot(v1, v2) - np.cos(self.alpha3),      # v1 * v2 = cos(alpha3)
                np.dot(v1, v3) - np.cos(self.alpha3),      # v1 * v3 = cos(alpha3)
                np.dot(v2, v3) - np.cos(self.alpha3),      # v2 * v3 = cos(alpha3)

                np.linalg.norm(v1)**2 - 1,                 # ||v1|| = 1
                np.linalg.norm(v2)**2 - 1,                 # ||v2|| = 1 
                np.linalg.norm(v3)**2 - 1                  # ||v3|| = 1 
            ]
            return eqs

        # Numerically solve the system of equations using the initial guess x0 
        sol = fsolve(equations, x0)
        
        v1 = sol[0:3]
        v2 = sol[3:6]
        v3 = sol[6:9]
        
        return v1, v2, v3

    def inverse_kinematics(self, v1, v2, v3, return_all_roots=False):
        v_vectors = [v1, v2, v3]
        thetas = []

        for i in range(3):
            vx, vy, vz = v_vectors[i]
            eta_i = self.eta[i]

            vx_loc = vx * np.cos(eta_i) - vy * np.sin(eta_i)
            vy_loc = vx * np.sin(eta_i) + vy * np.cos(eta_i)
            
            # Calculate coefficients Ai, Bi, Ci 
            A = -vy_loc * np.sin(self.alpha1) - vz * np.cos(self.alpha1) - np.cos(self.alpha2) 
            B = -vx_loc * np.sin(self.alpha1)                                                   
            C = vy_loc * np.sin(self.alpha1) - vz * np.cos(self.alpha1) - np.cos(self.alpha2) 
            
            # Solve the quadratic equation A*T^2 + 2*B*T + C = 0 
            # Using quadratic formula for roots of aT^2 + bT + c = 0 where a=A, b=2B, c=C
            discriminant = (2*B)**2 - 4*A*C
            
            if discriminant < 0:
                raise ValueError(f"No real solution for leg {i+1}.")
                
            T_plus = (-2*B + np.sqrt(discriminant)) / (2*A)
            T_minus = (-2*B - np.sqrt(discriminant)) / (2*A)
            
            # Find theta_i using T_i = tan(theta_i / 2)
            theta_plus = np.degrees(2 * np.arctan(T_plus))
            theta_minus = np.degrees(2 * np.arctan(T_minus))
            
            if return_all_roots:
                thetas.append((theta_plus, theta_minus))
            else:
                # Use the positive roots of equation 6 
                if T_plus > 0:
                    root = theta_plus 
                else:
                    root = theta_minus
                
                thetas.append(root)

        return thetas