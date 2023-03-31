import numpy as np

# Define signal types
SIGNAL_TYPE_STEP = 'step'
SIGNAL_TYPE_RAMP = 'ramp'
SIGNAL_TYPE_OSCILLATING = 'oscillating'

def generate_signal(signal_type, t, magnitude, period=None):
    if signal_type == SIGNAL_TYPE_STEP:
        return generate_step_signal(magnitude)
    elif signal_type == SIGNAL_TYPE_RAMP:
        return generate_ramp_signal(t, magnitude)
    elif signal_type == SIGNAL_TYPE_OSCILLATING:
        if period is None:
            raise ValueError("Period must be provided for oscillating signal.")
        return generate_oscillating_signal(t, magnitude, period)
    raise ValueError(f"Invalid signal type '{signal_type}', expected one of {SIGNAL_TYPE_STEP}, {SIGNAL_TYPE_RAMP}, {SIGNAL_TYPE_OSCILLATING}.")

def generate_step_signal(magnitude):
    return magnitude

def generate_ramp_signal(t, magnitude):
    return t * magnitude

def generate_oscillating_signal(t, magnitude, period):
    frequency = 1.0 / period
    return magnitude * np.sin(2 * np.pi * frequency * t)

def corrupt_array(data, corruption_specs, t):
    if not isinstance(data, np.ndarray):
        raise TypeError("Data must be a NumPy array.")
    if not isinstance(corruption_specs, list):
        raise TypeError("Corruption specifications must be a list.")
    
    for spec in corruption_specs:
        signal_type = spec.get('signal_type')
        magnitude = spec.get('magnitude')
        indices = spec.get('indices')
        period = spec.get('period', None)
        
        if signal_type not in [SIGNAL_TYPE_STEP, SIGNAL_TYPE_RAMP, SIGNAL_TYPE_OSCILLATING]:
            raise ValueError(f"Invalid signal type '{signal_type}', expected one of {SIGNAL_TYPE_STEP}, {SIGNAL_TYPE_RAMP}, {SIGNAL_TYPE_OSCILLATING}.")
        if not isinstance(magnitude, (int, float)):
            raise TypeError("Magnitude must be a numeric value.")
        if not isinstance(indices, (int, list)):
            raise TypeError("Indices must be an integer or a list of integers.")
        if isinstance(indices, list) and not all(isinstance(i, int) for i in indices):
            raise TypeError("All elements in the list of indices must be integers.")
        
        signal = generate_signal(signal_type, t, magnitude, period)
        data[spec['indices']] = signal
    return data
