from signal_generator import *

# Test case 1: Generate step signal on a single index
def test_generate_step_signal_single_index():
    # Data array
    data = np.array([0.0, 0.0, 0.0, 0.0])

    # Specify corruption: apply step signal of magnitude 5.0 to index 2
    corruption_specs = [{'signal_type': SIGNAL_TYPE_STEP,
                         'magnitude': 5.0, 'indices': 2, 'period': None}]

    # Current time step
    t = 0

    # Generate signal and update array
    corrupted_data = corrupt_array(data, corruption_specs, t)

    # Expected result
    expected_data = np.array([0.0, 0.0, 5.0, 0.0])

    # Test
    np.testing.assert_array_equal(corrupted_data, expected_data)

# Run test case 1
test_generate_step_signal_single_index()

# Test case 2: Generate ramp signal on multiple indices
def test_generate_ramp_signal_multiple_indices():
    # Data array
    data = np.array([0.0, 0.0, 0.0, 0.0])

    # Specify corruption: apply ramp signal of magnitude 4.0 to indices 1 and 3
    corruption_specs = [{'signal_type': SIGNAL_TYPE_RAMP,
                         'magnitude': 4.0, 'indices': [1, 3], 'period': None}]

    # Current time step
    t = 2

    # Generate signal and update array
    corrupted_data = corrupt_array(data, corruption_specs, t)

    # Expected result
    expected_signal = generate_ramp_signal(t, 4.0)
    expected_data = np.array([0.0, expected_signal, 0.0, expected_signal])

    # Test
    np.testing.assert_array_equal(corrupted_data, expected_data)


# Run test case 2
test_generate_ramp_signal_multiple_indices()

# Test case 3: Generate oscillating signal on a single index with multiple time steps
def test_generate_oscillating_signal_single_index_multiple_timesteps():
    # Data array
    data = np.array([0.0, 0.0])

    # Specify corruption: apply oscillating signal of magnitude 3.0 and period 2.0 to index 1
    corruption_specs = [{'signal_type': SIGNAL_TYPE_OSCILLATING,
                         'magnitude': 3.0, 'indices': 1, 'period': 2.0}]

    # Test for multiple time steps
    for t in [0.0, 0.5, 1.0, 1.5, 2.0]:
        # Generate signal and update array
        corrupted_data = corrupt_array(np.copy(data), corruption_specs, t)

        # Expected result
        expected_signal = generate_oscillating_signal(t, 3.0, 2.0)
        expected_data = np.array([0.0, expected_signal])

        # Test
        np.testing.assert_array_almost_equal(corrupted_data, expected_data)


# Run test case 3
test_generate_oscillating_signal_single_index_multiple_timesteps()

# Test case 4: Invalid signal type


def test_invalid_signal_type():
    data = np.array([0.0, 0.0, 0.0, 0.0])
    corruption_specs = [{'signal_type': 'invalid',
                         'magnitude': 5.0, 'indices': 2, 'period': None}]
    t = 0
    with np.testing.assert_raises(ValueError):
        corrupt_array(data, corruption_specs, t)


# Run test case 4
test_invalid_signal_type()

# Test case 5: Invalid indices type
def test_invalid_indices_type():
    data = np.array([0.0, 0.0, 0.0, 0.0])
    corruption_specs = [{'signal_type': SIGNAL_TYPE_STEP, 'magnitude': 5.0, 'indices': 'invalid', 'period': None}]
    t = 0
    with np.testing.assert_raises(TypeError):
        corrupt_array(data, corruption_specs, t)

# Run test case 5
test_invalid_indices_type()

# Test case 6: Missing period for oscillating signal
def test_missing_period_for_oscillating_signal():
    data = np.array([0.0, 0.0])
    corruption_specs = [{'signal_type': SIGNAL_TYPE_OSCILLATING, 'magnitude': 3.0, 'indices': 1}]
    t = 0
    with np.testing.assert_raises(ValueError):
        corrupt_array(data, corruption_specs, t)

# Run test case 6
test_missing_period_for_oscillating_signal()
