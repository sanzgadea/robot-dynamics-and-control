"""
Robot Dynamics and Control Assignment 1b: frequency response of PID controller
-------------------------------------------------------------------------------
DESCRIPTION:
1-DoF mass-damper plant controlled by a PID controller.

Important variables:
xr -> plant reference position
x -> plant actual position
dx -> plant velocity
ddx -> plant acceleration
F -> actuator force

TASK:
Analyse the frequency response of the provided PID controller for a mass-damper
plant using the Bode plot. Experimentally sample the actual position (output)
for a given sine reference position signal (input) at frequencies in Hertz (Hz)
provided in vector freq. Display magnitude in decibels (dB) and phase in
degrees. You can use the provided time vector to create the input sine. Feel
free to reuse your codes from Assignment 1a. Suggestion: look at the later
periods of the output signal when it becomes more repeatable.
-------------------------------------------------------------------------------


INSTRUCTOR: Luka Peternel
e-mail: l.peternel@tudelft.nl

"""



import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks # useful for analysis



'''PARAMETERS'''

m = 1.0 # mass
b = 1.0 # damping (e.g., air/water resistance)
g = 9.81 # gravity

dt = 0.001 # sample time
T = 10.0 # simulation time

freq = np.linspace(0.1, 10, 100) # frequencies in Hz at which to sample the response

bode_gain = [] # gains at corresponding frequencies
bode_phase = [] # phases at corresponding frequencies


'''SIMULATION'''

Kp = 10.0 # proportional gain
Ki = 5.0 # intergral gain
Kd = 1.0 # derivative gain
se = 0.0 # integrated error
pe = 0.0 # previous error (used for derivation)
ts = T/dt # trajectory size
t = np.linspace(0,T,int(ts)) # observation time
inputs = np.zeros((len(t),len(freq))) # array of input signals per frequency
outputs = np.zeros((len(t),len(freq))) # array of output signals per frequency
input_pks = [] # list of input peaks per frequency
output_pks = [] # list of output peaks per frequency
x = 0.0 # position
dx = 0.0 # velocity
ddx = 0.0 # acceleration

'''*********** Student should fill in ***********'''

# Create the sine input signal    
for idx, frequency in enumerate(freq):
    inputs[:, idx] = np.sin(2 * np.pi * frequency * t)
    
'''
print(inputs.shape)

plt.figure(1,figsize=(10, 5))
plt.subplot(211)
plt.title("1 Hz", fontsize=10)
plt.plot(t, inputs[:, 10])
plt.subplot(212)
plt.title("5 Hz", fontsize=10)
plt.plot(t, inputs[:, 50])
plt.tight_layout()
'''

# Sample the output for every frequency
for f_idx, frequency in enumerate(freq):
    
    # Reset values
    se = 0.0
    pe = 0.0
    x = 0.0
    dx = 0.0
    ddx = 0.0
    
    # Sample the output for every time step of a given frequency
    for t_idx, time in enumerate(t):
        
        # PID controller
        e = inputs[t_idx, f_idx] - x 
        se += e * dt
        F = Kp*(e) + Ki*se + Kd*(e-pe)/dt
        pe = e
        
        # Dynamics & numerical integration
        ddx = F/m - dx*b/m
        dx = dx + ddx*dt
        x = x + dx*dt
    
        # Add position to outputs array
        outputs[t_idx, f_idx] = x

'''
print(inputs.shape)

plt.figure(1,figsize=(10, 5))
plt.subplot(211)
plt.title("1 Hz", fontsize=10)
plt.plot(t, outputs[:, 10])
plt.subplot(212)
plt.title("5 Hz", fontsize=10)
plt.plot(t, outputs[:, 50])
plt.tight_layout()
'''

# Input peaks
input_peaks = []
for f_idx, frequency in enumerate(freq):
    input_peaks.append(find_peaks(inputs[:, f_idx]))
#print(len(input_peaks))

# Ouput peaks
output_peaks = []
for f_idx, frequency in enumerate(freq):
     output_peaks.append(find_peaks(outputs[:, f_idx]))
     
# Find the magnitude gain and phase shift
for f_idx, frequency in enumerate(freq):
    
    # Using index -1 to look at the last peak
    last_peak_index_output = output_peaks[f_idx][0][-1]
    last_peak_index_input = input_peaks[f_idx][0][-1]
    
    # Last peak amplitudes
    output_amplitude= outputs[last_peak_index_output, f_idx]
    input_amplitude = inputs[last_peak_index_input, f_idx]
    
    # Calculate gain 
    gain =  output_amplitude / input_amplitude
    gain = 20 * np.log10(output_amplitude / input_amplitude)
    
    # Calculate phase shift
    index_diff = last_peak_index_output - last_peak_index_input
    phase_shift =  index_diff * dt * 360 * frequency
    
    # Append to lists
    bode_gain.append(gain)
    bode_phase.append(phase_shift)
    
'''*********** Student should fill in ***********'''


'''ANALYSIS'''

plt.figure(1,figsize=(10, 5))

plt.subplot(211)
plt.title("GAIN", fontsize=10)
plt.plot(freq,bode_gain,"xb")
plt.xscale('log')
plt.grid('on')
plt.xlabel("frequency [Hz]", fontsize=12)
plt.ylabel("maginitude [dB]", fontsize=12)

plt.subplot(212)
plt.title("PHASE", fontsize=10)
plt.plot(freq,bode_phase,"xr")
plt.xscale('log')
plt.grid('on')
plt.xlabel("frequency [Hz]", fontsize=12)
plt.ylabel("phase [degrees]", fontsize=12)

plt.tight_layout()
