from helpers import param
from helpers import State, AccelerationState, Path, StateRange
from pathlib import arc_step, ackerman_radius

class DynamicModel(object):
    """ Encapsulates the dynamics of the car """
    def __init__(self):
        self.max_accel = float(param("dynamics.max_linear_accel")) / float(param("planning_freq"))
        self.max_decel = float(param("dynamics.max_linear_decel")) / float(param("planning_freq"))
        self.max_angular_accel = float(param("dynamics.max_angular_accel")) / float(param("planning_freq"))
        self.wheelbase = param("dynamics.wheelbase")
        self.planning_freq = param("planning_freq")

        self.raw_data = param("dynamics.steering_prediction")
        self.raw_data.sort(key=lambda x: x["steering_angle"])
        self.sorted_angles = map(lambda x: x["steering_angle"], self.raw_data)
        self.ackerman_transition = map(lambda x: x["ackerman_transition"], self.raw_data)

        self.min_dynamic_angle = self.sorted_angles[0]
        self.max_dynamic_angle = self.sorted_angles[-1]

        self.TABLE = {}

    def arc_estimate(self, sample_index, speed, steering_angle):
        if speed < self.ackerman_transition[sample_index]:
            # use ackerman steering
            return ackerman_radius(self.wheelbase, steering_angle)
        else:
            # estimate arc radius based off of nonlinear model
            coeffs = self.raw_data[sample_index]["polynomial_coefficients"]
            return coeffs[0] * speed * speed + coeffs[1] * speed + coeffs[2]

            
    def neighbor_samples(self, a):
        # find closest two entries for the given angle, along with interpolatoin weights.
        if a < self.min_dynamic_angle or a > self.max_dynamic_angle:
            return None
        elif self.min_dynamic_angle == a:
            return [(1, 0)]
        elif self.max_dynamic_angle == a:
            return [(1, len(self.raw_data)-1)]
        else:
            for i in xrange(len(self.sorted_angles)-1):
                if self.sorted_angles[i] == a:
                    return [(1,i)]
                elif self.sorted_angles[i] < a and a < self.sorted_angles[i+1]:
                    # find distance
                    w1 = a - self.sorted_angles[i]
                    w2 = self.sorted_angles[i+1] - a

                    # normalize
                    t = w1 + w2
                    w1 = 1 - w1 / t
                    w2 = 1 - w2 / t
                    return [(w1,i), (w2, i+1)]
        assert(False)
        return 

    def estimate_effective_arc(self, state):
        # return the expected path arc radius of the given state
        #   special cases: steering = 0 -> 0
        #                  steering < 0 -> -arc_radius
        
        sign = 1 if state.steering_angle >= 0 else -1

        if (state.speed, state.steering_angle) in self.TABLE:
            # print("MEMOIZED")
            return sign*self.TABLE[(state.speed, abs(state.steering_angle))]
        else:
            # print("not")
            a = abs(state.steering_angle)

            effective_radius = 0

            neighbors = self.neighbor_samples(a)

            if a == 0:
                effective_radius = 0.0
            elif neighbors == None:
                # outside of good data range, use ackerman TODO: expand the data range with approximation
                effective_radius = ackerman_radius(self.wheelbase, a)
            elif len(neighbors) == 1:
                # compute value of given angle
                effective_radius = self.arc_estimate(neighbors[0][1], state.speed, state.steering_angle)
            elif len(neighbors) == 2:
                # interpolate between two nearest angles
                effective_radius = neighbors[0][0]*self.arc_estimate(neighbors[0][1], state.speed, state.steering_angle) \
                                 + neighbors[1][0]*self.arc_estimate(neighbors[1][1], state.speed, state.steering_angle)
            
            self.TABLE[(state.speed, abs(state.steering_angle))] = effective_radius

            return sign*effective_radius

    def propagate(self, state, t=1.0):
        """ Gives an estimate for the final pose after assuming a given state after a given time. t=1 is time time of the next control timestep
        """
        effective_radius = self.estimate_effective_arc(state)
        # print(state.steering_angle, state.speed, effective_radius)
            
        propagated = arc_step(effective_radius, t * state.speed / self.planning_freq,
            state.x, state.y, state.theta)
        return State(x=propagated[0], y=propagated[1], theta=propagated[2], \
            steering_angle=state.steering_angle, speed=state.speed)

    def propagate_accel(self, start_state, linear_accel, steering_velocity):
        # NOTE: this function returns a list that DOES NOT include the given start state as 
        # it is assumed to be the last state in the previous path segment
        # TODO: need to consider the state execution frequency
        # given a start state and accelerations, this returns a list of control states that make up the action
        # control_states = [start_state]
        control_states = []
        ls = start_state

        # return DYNAMICS.propagate_accel(start)

        # [(x,y,theta,steering,speed),...]

        for i in xrange(param("planner.control_decisions_per_segment")):
            # bound the speed to [0, max_speed]
            next_speed = max(min(param("dynamics.max_speed"), ls.speed + linear_accel / float(param("execution_freq"))), 0)
            # bound the angle to [-max_deflection, max_deflection]
            next_steering = ls.steering_angle + steering_velocity / float(param("execution_freq"))
            next_steering = max(min(param("dynamics.max_deflection"), next_steering), -param("dynamics.max_deflection"))

            # add the new control and old position to new state
            ns = State(x=ls.x, y=ls.y, theta=ls.theta, speed=next_speed, steering_angle=next_steering)
            # propagate the old position forward in time
            propagated = self.propagate(ns)
            control_states.append(propagated)
            # set the most recent step as the last state for use in the next iteration
            ls = propagated

        return control_states