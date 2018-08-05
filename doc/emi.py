import numpy as np
import scipy.optimize as so

def exponential_decay_fun(amplitude, delay, decay):
    return lambda t: amplitude * np.minimum(1.0, np.exp(- (t - delay) / decay))

def exponential_decay(t, amp, delay, decay):
    return exponential_decay_fun(amp, delay, decay)(t)

def sample(f, sample_count, sampling_frequency):
    sampling_time = np.linspace(0, sample_count / sampling_frequency, sample_count)
    samples = np.array([f(t) for t in sampling_time])
    return sampling_time, samples

def generate_sample(amplitude, delay, decay, n_samples=100):
    f = exponential_decay_fun(amplitude, delay, decay)
    return sample(f, n_samples, 5000)

def fit_exponential_decay(x, y, initial_guess=(10.0, 0.005, 0.005)):
    return so.curve_fit(exponential_decay, x, y, initial_guess)

def relative_accuracy(truth, estimation):
    return (estimation - truth) / truth
