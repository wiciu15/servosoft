# AC servo drive firmware

This STM32 firmware is used as a servo driver for 3-phase PMSM motor with encoder feedback

It's in alpha development so there are many bugs and things included in software but not really working/tested. Code is not really organized

Right now working with ABB BSM 750-watt motor equipped with Tamagawa SSI encoder

# Working/tested

- Motor control with manual voltage and frequency control
- FOC vector control with encoder feedback
- Basic position/status communication with SSI encoders (Mitsubishi J2/J2s series and some Tamagawa encoders - tested with old ABB BSM-series motor). Mitsubishi use full-duplex communication, for tamagawa a half-duplex transciever is needed or DE/RE line to disable line driver. Baud rate 2 500 000 bits/s
- ABZ encoder but needs one manual rotation to index with a rotor properly
- MODBUS communication (some organization with registers needed)

# To-do

- ramp generators and limiters for control loops
- proper tuning of control loops
- position loop
- sensorless rotor angle aquisition
- code cleanup and testing for different hardware/software configuration variants
- parameter set management and storage
- analog and digital I/O configuration and usage
- 7-segment display
