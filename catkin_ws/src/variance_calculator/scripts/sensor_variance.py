#!/usr/bin/env python
import rospy
import numpy as np
import importlib
import matplotlib.pyplot as plt
from os.path import dirname, abspath, join
from operator import attrgetter

class VarianceCalculator:
    def __init__(self):
         # Get parameters from provate namespace
        self.topic_name = rospy.get_param('~topic_name')
        self.topic_type = rospy.get_param('~topic_type')
        [msg_module, msg_type] = self.topic_type.split("/")
        msg_class = getattr(importlib.import_module(msg_module + ".msg"), msg_type)
        self.subscriber = rospy.Subscriber(self.topic_name,  msg_class, self.callback)

        self.variable_names = rospy.get_param('~variable_names')
        self.variable_types = rospy.get_param('~variable_types')
        self.n_samples = rospy.get_param('~samples')
        self.n_measurements = 0  # Count of measurements taken

        self.n_variances = 0
        self.messages = []  # List of messages classes 
        for variables_type in self.variable_types:
            [msg_module, msg_type] = variables_type.split("/")
            msg = getattr(importlib.import_module(msg_module + ".msg"), msg_type)
            self.n_variances += len(msg.__slots__)
            self.messages.append(msg)

        # Place to store measurements
        self.measurements = np.empty([self.n_variances, self.n_samples])

    def callback(self, msg_data):
        # Don't run if measurement count reached
        if self.n_measurements < self.n_samples:
            measurement_i = 0  # Measurement row
            for variable_name in self.variable_names:
                selectors = variable_name.split("/")
                variable = attrgetter(".".join(selectors))(msg_data)
                # variable = getattr(msg_data, variable_name)
                try:
                    for slot in variable.__slots__:
                        self.measurements[measurement_i, self.n_measurements] = getattr(variable, slot)
                        measurement_i += 1
                except AttributeError:  # Only one value for this error
                    self.measurements[measurement_i, self.n_measurements] = variable
                    measurement_i += 1

            self.n_measurements += 1
            # If finished
            if self.n_measurements == self.n_samples:
                variances = self.calculate_variance()
                rospy.loginfo("Finished, variances for /{} are: ".format(self.topic_name))
                rospy.loginfo(variances)
                self.plot_results()

    def calculate_variance(self):
        self.variances = np.var(self.measurements, axis=1)
        return self.variances

    def calculate_variance_progression(self, points=10):
        if self.n_measurements < points:
            raise ValueError("Number of points (arg) must the same or greater then the number of measurements")

        staged_variances = np.zeros([self.n_variances, points])
        stages = []
        
        for i in range(0, points):
            max_index = int(round(self.n_measurements / points * (i + 1)))
            staged_variances[:, i] = np.var(self.measurements[:, :max_index], axis=1)
            stages.append(max_index)

        return [staged_variances, stages]

    def publish(self):
        pass

    def plot_results(self):
        # Plot each variable in turn
        # n_variables = len(self.variable_names)
        measurement_i = 0
        points = 50 if self.n_measurements >= 50 else self.n_measurements
        [variance_progression, stages] = self.calculate_variance_progression(points=points)
        for variable_name, variables_type in zip(self.variable_names, self.variable_types):
            [msg_module, msg_type] = variables_type.split("/")
            msg = getattr(importlib.import_module(msg_module + ".msg"), msg_type)
            try:
                n_sub_variables = len(msg.__slots__)
            except AttributeError:
                n_sub_variables = 1

            fig, axes = plt.subplots(nrows=2, ncols=n_sub_variables, figsize=[14,8], squeeze=False)
            fig.suptitle("Variance analysis for {}".format(variable_name))
            for sub_variable_i in range (0, n_sub_variables):
                axes[0, sub_variable_i].hist(self.measurements[measurement_i, :])
                axes[0, sub_variable_i].set_title("Variance histogram for {}".format(msg.__slots__[sub_variable_i])) 
                axes[1, sub_variable_i].plot(stages, variance_progression[measurement_i, :])
                axes[1, sub_variable_i].set_title("Variance progression for {}".format(msg.__slots__[sub_variable_i]))

                measurement_i += 1

            filepath = join(
                dirname( dirname(abspath(__file__)) ),
                "output/variance_plot_{}_{}.png".format(self.topic_name.strip("/").replace("/", "_"), variable_name.replace("/", "_"))
            )
            fig.savefig(filepath)
            plt.show(block = False)


if __name__ == '__main__':
    rospy.init_node("variance_calculator", anonymous=True)
    variance_calculator = VarianceCalculator()

    # r = rospy.Rate(1)  # 1hz
    # while not rospy.is_shutdown():
    #     variance_calculator.publish()
    #     r.sleep()
    #     rospy.spinOnce()

    rospy.spin()
    