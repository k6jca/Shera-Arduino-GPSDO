# Shera-Arduino-GPSDO

This repository contains Mathworks Simulink models for simulating the Brooks Shera GPSDO algorithm on my Arduino platform and for comparing this Arduino/Shera simulation against my original Brooks Shera GPSDO simulation (the latter located in my "Brooks-Shera-GPSDO" GitHub repository).

It also contains my Arduino code implementing this same algorithm.

The files in this repository are:

Shera's Original PIC Code (Ver 1.28):

o  SHERA_GPSDO_PIC_CODE_VER_1P28.pdf.
  
Simulink Models:

o  k6jca_181230_ARDUINO.slx  (Arduino model using Shera Algorithm).

o  k6jca_181230_SHERA_ARDUINO_Comparison.slx (Compares Shera model (see Brooks-Shera-GPSDO repository) and Arduino Models results).

Excel Stimulus files:

o  Quectel_PD_181226.xlsx  (Phase deltas using Quectel L76 GPS receiver).

o  Trimble_PD_181227.xlsx  (Phase deltas using Trimble Resolution T GPS receiver).
