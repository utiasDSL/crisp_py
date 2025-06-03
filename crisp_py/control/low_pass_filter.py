import numpy as np
from scipy.signal import butter


class LowPassFilter:
    def __init__(self, cutoff_freq, sample_time, order=4):
        self.cutoff_freq = cutoff_freq
        self.sample_time = sample_time
        self.order = order
        self.sample_rate = 1.0 / sample_time
        self.nyquist_freq = 0.5 * self.sample_rate
        self.Wn = cutoff_freq / self.nyquist_freq
        self.b, self.a = butter(order, self.Wn, btype='lowpass')
        
        # Initialize history buffers
        self.unfiltered_history = None
        self.filtered_history = None
        
    def initialize_buffers(self, signal_shape):
        """Initialize history buffers with the correct shape"""
        self.unfiltered_history = np.zeros([self.order, *signal_shape])
        self.filtered_history = np.zeros([self.order, *signal_shape])
        
    def filter(self, x0):
        """Apply the lowpass filter to new input"""
        if self.unfiltered_history is None:
            self.initialize_buffers(x0.shape)
            
        result = self.b[0] * x0
        for i in range(self.order):
            result += -self.a[i+1]*self.filtered_history[i, :] + self.b[i+1]*self.unfiltered_history[i, :]
            
        # Update history buffers
        self.unfiltered_history[1:, :] = self.unfiltered_history[:-1, :]
        self.unfiltered_history[0, :] = x0
        self.filtered_history[1:, :] = self.filtered_history[:-1, :]
        self.filtered_history[0, :] = result
        
        return result
