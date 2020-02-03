# Gaggia-Classic-PID-TSIC306
A simple PID controller in python3 to control the temperature of a Gaggia Classic coffee machine.
The project uses:
- Raspberry pi 
- a TSIC306 temp controller  
- 1 Fostek 25 Amp SSRs to control the boiler
- 1 Fostek 25 Amp SSRs to control the boiler the brew pump.

# Features:
- PID with anti-windup; seems to be stable at +/- 0.5c
- able to heat boiler while brew switch is on to maintain water temp during brew
- switchable logging to CSV
- switchable verbose logging to console
- LCD temp display for 1602 via I2C interface
- sleep mode: turns the boiler off after x minutes
- Steam mode
- temp setpoint adjustment
- Flush function
- settings autosave to disk

# Other:
- 3d printable enclosure: https://www.thingiverse.com/thing:4140909
