import numpy as np
from CoaxSphericalController import CoaxSphericalController

def run_kinematics_test():
    # Initialize the controller with the paper's geometric parameters
    # alpha1 = 45, alpha2 = 90, beta = 90
    controller = CoaxSphericalController(alpha1_deg=45, alpha2_deg=90, beta_deg=90)
    
    # Define the test inputs from the paper
    theta_input = [75, 90, 65]
    x0 = [1, 1, 1, 1, -1, -1, -1, -1, 1] # Initial guess for the l-l-l posture 
    
    print(f"Forward Kinematics:")
    print(f"Input Joint Angles (Theta): {theta_input}")
    print(f"Initial Guess (x0): {x0}\n")
    
    # Run Forward Kinematics
    v1, v2, v3 = controller.forward_kinematics(theta_input, x0)
    
    print("Computed Unit Vectors (v_i):")
    print(f"v1 = [{v1[0]:.4f}, {v1[1]:.4f}, {v1[2]:.4f}]")
    print(f"v2 = [{v2[0]:.4f}, {v2[1]:.4f}, {v2[2]:.4f}]")
    print(f"v3 = [{v3[0]:.4f}, {v3[1]:.4f}, {v3[2]:.4f}]\n")
    
    print("Expected Unit Vectors from Paper:")
    print("v1 = [0.2348, 0.9717, 0.0247]")  
    print("v2 = [0.6966, -0.6769, -0.2379]") 
    print("v3 = [-0.9316, -0.2948, 0.2125]\n") 
    
    # Test Inverse Kinematics using the computed unit vectors
    print(f"Inverse Kinematics:")
    
    # Get all possible roots to see the two working modes for each leg
    thetas_all = controller.inverse_kinematics(v1, v2, v3, return_all_roots=False)
    
    print("Computed IK Roots:")
    for i, roots in enumerate(thetas_all):
        print(f"Leg {i+1} roots: [{roots:.2f}]")
        
    print("\nExpected Roots from Paper:")
    print("Leg 1: 75") 
    print("Leg 2: 90")
    print("Leg 3: 65")

if __name__ == "__main__":
    run_kinematics_test()