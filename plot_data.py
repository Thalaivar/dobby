import numpy as np
import matplotlib.pyplot as plt
import os

PLOT_DIR = "/Users/dhruvlaad/quadcopter/plots/"
RAD_TO_DEG = 57.32484

class Data:
    def __init__(self):
        self.attitude_data_set = "attitude_data_file.txt"
        self.attitude_err_data_set = "attitude_error_data_file.txt"
        self.desired_attitude_data_set = "desired_attitude_data_file.txt"
        self.channel_data_set = "channel_data_file.txt"
        self.desired_body_rates_data_set = "desired_body_rates_data_file.txt"
        self.body_rate_error_data_set = "error_data_file.txt"
        self.body_rates_data_set = "body_rates_data_file.txt"

        self.plot_labels = None
        self.plot_colors = None
        self.title = None

    def get_data(self, data_set, type):
        if type == 1:
            if data_set == "body_rates":
                data_1, data_2, data_3 = np.loadtxt(self.body_rates_data_set, delimiter=",", unpack=True)
            if data_set == "attitude":
                data_1, data_2, data_3 = np.loadtxt(self.attitude_data_set, delimiter=",", unpack=True)
            elif data_set == "attitude_err":
                data_1, data_2, data_3 = np.loadtxt(self.attitude_err_data_set, delimiter=",", unpack=True)
            elif data_set == "desired_attitude":
                data_1, data_2, data_3 = np.loadtxt(self.desired_attitude_data_set, delimiter=",", unpack=True)
            elif data_set == "desired_body_rates":
                data_1, data_2, data_3 = np.loadtxt(self.desired_body_rates_data_set, delimiter=",", unpack=True)
            elif data_set == "body_rate_error":
                data_1, data_2, data_3 = np.loadtxt(self.body_rate_error_data_set, delimiter=",", unpack=True)

            return [data_1, data_2, data_3]

        elif type == 2:
            if data_set == "channel":
                data_1, data_2, data_3, data_4 = np.loadtxt(self.channel_data_set, delimiter=",", unpack=True)

            return [data_1, data_2, data_3, data_4]

    def process_data(self, data_set, data_points):
        if data_set == "body_rates" or data_set == "attitude" or data_set == "attitude_err" or data_set == "desired_body_rates" or data_set == "body_rate_error" :
            for i in range(len(data_points)):
                 data_points[i] = data_points[i]*RAD_TO_DEG

            self.plot_labels = ["Roll", "Pitch", "Yaw"]
            self.plot_colors = ['r', 'g', 'b']
            self.title = data_set

            return data_points

        elif data_set == "channel":
            self.plot_labels = ["CH1", "CH2", "CH3", "CH4"]
            self.plot_colors = ['r', 'g', 'b', 'm']

            return data_points

    def plot_data(self, data_set, type):
        data_points = self.get_data(data_set, type)
        processed_data_points = self.process_data(data_set, data_points)

        for i in range(len(processed_data_points)):
            plt.plot(processed_data_points[i], self.plot_colors[i], label=self.plot_labels[i])

        plt.legend(loc="upper right")
        plt.grid()
        plt.title(self.title)
        save_path = PLOT_DIR + data_set + ".png"
        plt.savefig(save_path)
        #plt.show()


def main():
    data = Data();
    choice = input("Choose a data set:\n1 - Attitude error\n2 - Body rates error\n3 - Desired body rates\n4 - Channel values - PWM\n5 - Attitude\n6 - Body Rates\nChoose your option: ")
    if choice == "1":
        data_set = "attitude_err"
    elif choice == "2":
        data_set = "body_rate_error"
    elif choice == "3":
        data_set = "desired_body_rates"
    elif choice == "4":
        data_set = "channel"
    elif choice == "5":
        data_set = "attitude"
    elif choice == "6":
        data_set = "body_rates"

    if data_set == "body_rates" or data_set == "attitude" or data_set == "attitude_err" or data_set == "desired_body_rates" or data_set == "body_rate_error" :
        type = 1
    elif data_set == "channel":
        type = 2

    data.plot_data(data_set, type)

if __name__ == '__main__':
    main()
