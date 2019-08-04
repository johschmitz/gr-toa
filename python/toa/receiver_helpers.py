import numpy as np

def calculate_fft_size(num_samples, min_exponent):
    max_exponent = int(np.log2(num_samples))
    len_sum_power_two = 2**int(max_exponent)
    # Add power of twos until we are close to the original length
    for exponent in range(max_exponent,min_exponent-1,-1):
        if len_sum_power_two + 2**exponent <= num_samples:
            len_sum_power_two = len_sum_power_two + 2**exponent
    return 3 * len_sum_power_two
