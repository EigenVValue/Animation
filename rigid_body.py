import numpy as np
from quaternion_computation import normalize_quaternion, quaternion_multiply, quaternion_to_rotation

class RigidBody(object):
    def __init__(self):
        # Rigid Body Parameters
        self.vertices= (
            (-5,-5, 5),
            (-5, 5, 5),
            ( 5, 5, 5),
            ( 5,-5, 5),
            (-5,-5,-5),
            (-5, 5,-5),
            ( 5, 5,-5),
            ( 5,-5,-5)
            )

        # Mass of the object
        self.m = 100.0
        self.m_inverse = 1 / self.m

        # Inertia for the object
        self.iBody = np.zeros((3, 3))
        self.iBody[0, 0] = (1.0 / 12.0) * (self.m * ((10 * 10) + (10 * 10)))
        self.iBody[1, 1] = (1.0 / 12.0) * (self.m * ((10 * 10) + (10 * 10)))
        self.iBody[2, 2] = (1.0 / 12.0) * (self.m * ((10 * 10) + (10 * 10)))
        
        # Initial state values 
        self.velocity = np.array([10, 10, 10])
        self.position = np.array([-100, 80, -100])
        self.quaternion = np.array([1, 0, 0, 0])
        self.angular_velocity = np.array([1, 1, 0])

        # Simulation Parameters
        self.gravity_acceleration = np.array([0, -9.8, 0]) # Gravitation acceleration
        self.ks = 1000 # Spring Stiffness
        self.timestep = 1/100.0 # Amount of time by which the simulation advances
        self.nsubsteps = 2 # Number of steps within a timestep for finer calculations
        self.floor_y = 0.0 # Position of the floor


    def simulate_one_substep(self):
        # detect collision and determine vertices that collided
        is_collision, collision_vertices = self.collision_detection()
        # calculate the total forces and torques exerted on the body
        total_force, total_torque = self.calculate_forces_and_torques(is_collision, collision_vertices)
        # calculate the linear and angular accelerations based on the forces and torques
        linear_acceleration, angular_acceleration = self.calculate_accelerations(total_force, total_torque)
        # integrate everything to the get new state variables
        new_v, new_x, new_w, new_q = self.integrate(linear_acceleration, angular_acceleration)
        
        # update the class variables
        self.velocity = new_v
        self.position = new_x
        self.angular_velocity = new_w
        self.quaternion = new_q

    def collision_detection(self):
        is_collision = False
        collision_vertices = []

        # your code begins here
        for vertex in self.vertices:
            np_vertex = np.empty(len(vertex), dtype=object)
            np_vertex[:] = vertex
            if (self.floor_y > vertex[1]+self.position[1]) :
                is_collision = True
                collision_vertices.append(vertex)
        
        return is_collision, collision_vertices
    
    def calculate_forces_and_torques(self, is_collision, collision_vertices):
        total_force = np.array([0.0, 0.0, 0.0])
        total_torque = np.array([0.0, 0.0, 0.0])
        
        # your code begins here 

        # Q to Rotation
        rotation = quaternion_to_rotation(self.quaternion)

        # Gravity F = mg
        for vertex in self.vertices:
            np_vertex = np.empty(len(vertex), dtype=object)
            np_vertex[:] = vertex
            G = self.gravity_acceleration * self.m/8
            total_force = total_force + G
            m1 = np.dot(np_vertex, rotation)
            m1 = m1.astype('float32')
            m2 = np.cross(m1, G)
            total_torque = total_torque + m2

        # f = (ks * d)*n
        if (is_collision == True):
            for vertex in collision_vertices:
                np_vertex = np.empty(len(vertex), dtype=object)
                np_vertex[:] = vertex
                f1 = self.ks * abs(self.floor_y - np_vertex[1])
                total_force[1] = total_force[1] + f1
                
                m1 = np.dot(np_vertex, rotation)
                m1 = m1.astype('float32')
                m2 = np.cross(m1, np.array([0.0, f1, 0.0]))
                total_torque = total_torque + m2

        return total_force, total_torque
    
    def calculate_accelerations(self, force, torque):
        linear_acceleration = np.array([0.0, 0.0, 0.0])
        angular_acceleration = np.array([0.0, 0.0, 0.0])

        # your code begins here 
        
        # a = F/m
        linear_acceleration = force / self.m

        # I = Rt * Iref * (Rt)T
        rotation = quaternion_to_rotation(self.quaternion)
        self.iBody = rotation.dot(self.iBody).dot(rotation.transpose())

        # a
        iBody_inverse = np.diag(self.iBody)
        m1 = np.cross(rotation, self.iBody)
        m2 = np.dot(m1, rotation)
        m3 = torque - m2
        m4 = np.dot(iBody_inverse, m3)
        angular_acceleration = m4

        return linear_acceleration, angular_acceleration
        
    def integrate(self, linear_acceleration, angular_acceleration):
        # your code begins here

        #print(linear_acceleration)
        # default values (the following lines can be modified)
        new_v = self.velocity + self.timestep * linear_acceleration
        new_x = self.position + self.timestep * new_v

        new_w = self.angular_velocity + self.timestep * angular_acceleration
        #print(self.angular_velocity)
        #print(new_w)

        tmp_w = self.timestep/2 * new_w
        new_qua = np.array([0, tmp_w[0], tmp_w[1], tmp_w[2]])
        qua = quaternion_multiply(new_qua, self.quaternion)
        new_q = normalize_quaternion(self.quaternion + qua)

        return new_v, new_x, new_w, new_q