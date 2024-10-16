#!/usr/bin/env python3

from math import atan2, pi, sin, cos, pow, sqrt



from dataclasses import dataclass
@dataclass
class EndEffectorWorkingSpace:
    # Pos of The P2, which is the one we resolved for
    P_x: float
    P_y: float
    P_z: float


class ComputeIk():

    def __init__(self, DH_parameters):
        
        # DH parameters
        self.DH_parameters_ = DH_parameters

    def get_dh_param(self, name):

        if name in self.DH_parameters_:
            return self.DH_parameters_[name]
        else:
            assert False, "Asked for Non existen param DH name ="+str(name)

    def compute_ik(self, end_effector_pose, elbow_configuration = "plus-minus"):
        
        # Initialization
        P_x = end_effector_pose.P_x
        P_y = end_effector_pose.P_y
        P_z = end_effector_pose.P_z

        # We get all the DH parameters
        a2 = self.get_dh_param("a2")
        a3 = self.get_dh_param("a3")

        print("Input Data : elbow config: "+elbow_configuration)
        print("P_x = "+str(P_x))
        print("P_y = "+str(P_y))
        print("P_z = "+str(P_z))       
        print("a2 = "+str(a2))
        print("a3 = "+str(a3))

        # Theta 3 Calculation
        cos_theta3 = (P_x**2 + P_y**2 + P_z**2 - a2**2 - a3**2)/(2*a2*a3)
        sin_theta3_positive = sqrt(1-cos_theta3**2)
        sin_theta3_negative = -sin_theta3_positive 
        theta_3_positive = atan2(sin_theta3_positive, cos_theta3) # theta_3_I solution
        theta_3_negative = atan2(sin_theta3_negative, cos_theta3) # theta_3_II solution
        
        # Theta 2 Calculation
        a2_plus_a3c3 = a2+ a3*cos_theta3
        length_px_py = sqrt(P_x**2 + P_y**2)
        theta2_3pos_1 = atan2(a2_plus_a3c3*P_z - a3*sin_theta3_positive*length_px_py,  a2_plus_a3c3*length_px_py+a3*sin_theta3_positive*P_z) # theta_2_I solution
        theta2_3pos_2 = atan2(a2_plus_a3c3*P_z + a3*sin_theta3_positive*length_px_py, -a2_plus_a3c3*length_px_py+a3*sin_theta3_positive*P_z) # theta_2_II solution
        theta2_3neg_1 = atan2(a2_plus_a3c3*P_z - a3*sin_theta3_negative*length_px_py,  a2_plus_a3c3*length_px_py+a3*sin_theta3_negative*P_z) # theta_2_III solution
        theta2_3neg_2 = atan2(a2_plus_a3c3*P_z + a3*sin_theta3_negative*length_px_py, -a2_plus_a3c3*length_px_py+a3*sin_theta3_negative*P_z) # theta_2_IV solution

        # Theta 1 Calculation
        theta1_1 = atan2(P_y, P_x) # theta1_I solution
        theta1_2 = theta1_1  - pi if P_y >= 0 else theta1_1 + pi # theta1_II solution

        solution1 = [{'config': 'none','theta': theta1_1}, {'config': 'plus','theta':theta2_3pos_1}, {'config': 'plus','theta':theta_3_positive}]
        solution2 = [{'config': 'none','theta': theta1_1}, {'config': 'plus','theta':theta2_3neg_1}, {'config': 'minus','theta':theta_3_negative}]
        solution3 = [{'config': 'none','theta': theta1_2}, {'config': 'minus','theta':theta2_3pos_2}, {'config': 'plus','theta':theta_3_positive}]
        solution4 = [{'config': 'none','theta': theta1_2}, {'config': 'minus','theta':theta2_3neg_2}, {'config': 'minus','theta':theta_3_negative}]
        list_raw_solutions = [solution1, solution2, solution3, solution4]

        for sol in list_raw_solutions:
            theta1 = sol[0].get('theta')
            theta2 = sol[1].get('theta')
            config2 = sol[1].get('config')
            theta3 = sol[2].get('theta')
            config3 = sol[2].get('config')
            is_possible, possible_message = is_possible_solution(theta2, theta3)

            if config2 +"-" + config3 ==  elbow_configuration:
                break        

        return [theta1, theta2, theta3], is_possible ,possible_message

def is_possible_solution(theta2, theta3):
    if -pi/4 <= theta2 <= 3*pi/4:
        is_possible = True
        message = "theta2 is possible, "
    else:
        is_possible = False
        message = "theta2 is NOT POSSIBLE, "        
    if -3*pi/4 <= theta3 <= 3*pi/4:
        is_possible = is_possible and True
        message += "theta3 is possible."
    else:
        is_possible = is_possible and False
        message += "theta3 is NOT POSSIBLE."    

    return is_possible , message


def calculate_ik(P_x, P_y, P_z, DH_parameters, elbow_configuration = "plus-minus"):

    ik = ComputeIk(DH_parameters = DH_parameters)
    end_effector_pose = EndEffectorWorkingSpace(P_x = P_x, P_y = P_y, P_z = P_z)
    solution , is_possible, message = ik.compute_ik(  end_effector_pose=end_effector_pose, elbow_configuration= elbow_configuration)
    theta1 = solution[0]
    theta2 = solution[1]
    theta3 = solution[2]
    if is_possible:
        #print(f"Solution {i} theta1: {theta1}, theta2: {theta2} config: {config2}, theta3: {theta3} config: {config3}")
        print(f"Angle theta solved =[{theta1},{theta2},{theta3}], solution possible: True")
    else:
        #print(f"*** Not possible solution: {message}, Solution {i} theta1: {theta1}, theta2: {theta2} config: {config2}, theta3: {theta3} config: {config3}")
        print(f"Angle theta solved =[{theta1},{theta2},{theta3}], solution possible: False. " + message)

    return solution, is_possible

if __name__ == '__main__':
    


    a2 = 1.0
    a3 = 1.0

    # theta_i here are valriables of the joints
    # We only fill the ones we use in the equations, the others were already 
    # replaced in the Homogeneous matrix
    DH_parameters={"a2":a2, "a3":a3}

    P_x = 0.5
    P_y = 0.6
    P_z = 0.7

    config_space = ['plus', 'minus']

    for theta2_config in config_space:
        for theta3_config in config_space:
            config_type = theta2_config + "-" + theta3_config
            calculate_ik(P_x=P_x, P_y=P_y, P_z = P_z, DH_parameters = DH_parameters, elbow_configuration = config_type)
