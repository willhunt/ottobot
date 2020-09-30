import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import parse
    
class OttobotResponse:
    def __init__(self, filename, title=None, delta_t_pre=0.5, delta_t_post=3):
        self.title = title
        self.delta_t_pre = delta_t_pre  # Time span to view before step/cmd [s]
        self.delta_t_post = delta_t_post  # Time span to view after step/cmd [s]
        self.datasets = self.load_data(filename)
        
    def load_data(self, filename):
        dataframe = pd.read_csv(filename, skipinitialspace=True)
        datasets = [
            {'t_index': 0, 'data_index': 1, 'title': "speed"},
            {'t_index': 2, 'data_index': 3, 'title': "error"},
            {'t_index': 4, 'data_index': 5, 'title': "output"},
        ]
        return_data = {}
        for dataset in datasets:
            # Load data
            ts = dataframe.iloc[:, dataset['t_index']].to_numpy()
            y = dataframe.iloc[:, dataset['data_index']].to_numpy()
            # Remove empty values (nan)
            indexes_nan = np.isnan(ts)
            ts = ts[~indexes_nan]
            y = y[~indexes_nan]
            # Find index of step
            index_step = np.argmax(abs(y) > 0.01)
            t_step = ts[index_step]
            t_start = t_step - self.delta_t_pre
            index_start = np.argmax(ts >= t_start)
            t_end = t_step + self.delta_t_post
            index_end = np.argmax(ts >= t_end)
            return_data[dataset['title']] = {
                'title': dataset['title'],
                'ts': ts[index_start : index_end],
                't': ts[index_start : index_end] - ts[index_start],
                'values': y[index_start : index_end]
            }
        return return_data
    
                
    def get_steady_state_speed(self):
        # Average second half of response
        n_samples = len(self.datasets['speed']['values'])
        w_ss = np.mean(self.datasets['speed']['values'][int(n_samples/2):])
        return w_ss
    
    def get_time_steady_state(self):
        w_ss = self.get_steady_state_speed()
        index_ss = np.argmax(np.abs(self.datasets['speed']['values']) >= 0.99 * np.abs(w_ss))
        t_ss = self.datasets['speed']['ts'][index_ss]
        return t_ss

class OttobotStepResponse(OttobotResponse):
    def __init__(self, filename, title=None, delta_t_pre_step=0.5, delta_t_post_step=3):
        super().__init__(filename, title, delta_t_pre_step, delta_t_post_step)
   
    def plot_response(self, show_SIMC=False):
        fig, ax1 = plt.subplots(figsize=[16,6])
        ax1.set_xlabel('time (s)')
        ax1.margins(x=0)

        color = 'tab:red'
        ax1.plot(self.datasets['speed']['ts'], self.datasets['speed']['values'], color=color, label="Speed")
        ax1.set_ylabel('speed (rad/s)')
        ax1.tick_params(axis='y', labelcolor=color)

        ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
        color = 'tab:blue'
        ax2.plot(self.datasets['output']['ts'], self.datasets['output']['values'], color=color, label="Command")
        ax2.set_ylabel('command (-)')
        ax2.tick_params(axis='y', labelcolor=color)
        ax1.set_title(self.title)
        
        if show_SIMC:
            w_ss = self.get_steady_state_speed()
            ax1.plot(ax1.get_xlim(), [w_ss, w_ss], '--', c='grey', label="Steady state: {:.2f}rad/s".format(w_ss))
            t_63percent = self.get_time_63percent()
            ax1.plot([t_63percent, t_63percent], ax1.get_ylim(), '--', label="Time at 63%: {:.1f}s".format(t_63percent))
            t_ss = self.get_time_steady_state()
            ax1.plot([t_ss, t_ss], ax1.get_ylim(), '--', label="Time at steady state: {:.1f}s".format(t_ss))
            ax1.legend()
        
    def plot_against(self, other_step_responses, zoom=None):
        fig, ax1 = plt.subplots(figsize=[16,6])
        ax1.set_xlabel('time (s)')
        ax1.set_ylabel('speed (rad/s)')
        ax1.tick_params(axis='y')
        step_responses = [self] + other_step_responses
        for step_response in step_responses:
            ax1.plot(step_response.datasets['speed']['t'], step_response.datasets['speed']['values'], label=step_response.title)
        ax1.legend()
        ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
        ax2.plot(self.datasets['output']['t'], self.datasets['output']['values'], color='pink', label="Output")
        ax2.set_ylabel('command (-)')
        ax2.tick_params(axis='y', labelcolor=ax2.get_lines()[0].get_color())
        
        if zoom == True:
            # find min and max values
            value_range = [10000, 0]
            
            for step_response in step_responses:
                # Look in second half of samples
                n_samples = len(step_response.datasets['speed']['values'])
                for value in step_response.datasets['speed']['values'][int(n_samples/2):]:
                    value_range[0] = value if value < value_range[0] else value_range[0]
                    value_range[1] = value if value > value_range[1] else value_range[1]
            ax1.set_ylim(value_range[0]-0.5, value_range[1]+0.5)
        
    def get_step_time(self):
        # Get index of step change
        index_step = np.argmax(self.datasets['output']['values'] >= 0.1) - 1
        t_step = self.datasets['speed']['ts'][index_step]
        return t_step
    
    def get_time_to_steady_state(self):
        t_start = self.get_response_start_time()
        t_ss = self.get_time_steady_state()
        return t_ss - t_start
    
    def get_response_start_time(self):
        # Get index of response to step change
        index_respond = np.argmax(np.abs(self.datasets['speed']['values']) >= 0.1) - 1
        t_respond = self.datasets['speed']['ts'][index_respond]
        return t_respond
    
    def get_response_delay(self):
        # Time at which step occurs
        t_step = self.get_step_time()
        # Time at which motor responds
        t_respond = self.get_response_start_time()
        theta = t_respond - t_step
        return theta
    
    def get_time_63percent(self):
        w_ss = self.get_steady_state_speed()
        index_63percent = np.where(self.datasets['speed']['values'] >= 0.63 * w_ss)[0][0]
        ts_interp = np.arange(self.datasets['speed']['ts'][index_63percent - 1], self.datasets['speed']['ts'][index_63percent], 0.0001)
        yinterp = np.interp(
            ts_interp,
            self.datasets['speed']['ts'][index_63percent-1:index_63percent+1],
            self.datasets['speed']['values'][index_63percent-1:index_63percent+1]
        )
        index_63percent_interp = np.where(yinterp >= 0.63 * w_ss)[0][0]
        t_63percent = ts_interp[index_63percent_interp]
        return t_63percent
    
    def get_tau1(self):
        t_63percent = self.get_time_63percent()
        t_respond = self.get_response_start_time()
        return t_63percent - t_respond


class OttobotPidResponse(OttobotResponse):
    def __init__(self, filename, title=None, delta_t_pre_step=0.5, delta_t_post_step=3):
        super().__init__(filename, title, delta_t_pre_step, delta_t_post_step)
        
    def plot_response(self):
        fig, ax1 = plt.subplots(figsize=[16,6])
        ax1.set_xlabel('time (s)')
        ax1.margins(x=0)

        color = 'tab:red'
        ax1.plot(self.datasets['speed']['ts'], self.datasets['speed']['values'], color=color, label="Speed")
        ax1.plot(self.datasets['error']['ts'], self.datasets['error']['values'], ':', color=color, label="Error")
        ax1.set_ylabel('speed (rad/s)')
        ax1.tick_params(axis='y', labelcolor=color)

        ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
        color = 'tab:blue'
        ax2.plot(self.datasets['output']['ts'], self.datasets['output']['values'], color=color, label="Command")
        ax2.set_ylabel('command (-)')
        ax2.tick_params(axis='y', labelcolor=color)
        ax1.set_title(self.title)
        
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2)
        
    def plot_against(self, other_step_responses, zoom=None, error=False):
        fig, ax1 = plt.subplots(figsize=[16,6])
        ax1.set_xlabel('time (s)')
        ax1.set_ylabel('speed (rad/s)')
        ax1.tick_params(axis='y')
        step_responses = [self] + other_step_responses
        for step_response in step_responses:
            if error:
                ax1.plot(step_response.datasets['error']['t'], step_response.datasets['error']['values'], label=step_response.title)
                ax1.set_title("Error")
            else:
                ax1.plot(step_response.datasets['speed']['t'], step_response.datasets['speed']['values'], label=step_response.title)
                ax1.set_title("Speed")
        ax1.legend()
        
        if zoom == True:
            # find min and max values
            value_range = [10000, 0]
            
#             for step_response in step_responses:
            for line in ax1.lines:
                y = line.get_ydata()
                # Look in second half of samples
                n_samples = len(y)
                for value in y[int(n_samples/2):]:
                    value_range[0] = value if value < value_range[0] else value_range[0]
                    value_range[1] = value if value > value_range[1] else value_range[1]
            ax1.set_ylim(value_range[0]-0.5, value_range[1]+0.5)