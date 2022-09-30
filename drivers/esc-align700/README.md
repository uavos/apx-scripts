# APX script for align700 esc

Script reads and parses telemetry data from KONTRONIK electronic speed control (ESC) on helicopter Align700
and sends some critical data to vehicle's list of variables (mandala):

- Revolution speed, RPM. mandala/est/env/eng/rpm
- Battery voltage, V. mandala/sns/env/eng/voltage
- Battery current, A. mandala/sns/env/eng/current
- BEC temperature, °C. mandala/sns/env/eng/temp

KONTRONIK model: JIVE Pro 120+ HV

[Operating manual](https://www.kontronik.com/fileadmin/kontronik-sobek/Public/Content/Images/Content/Downloads/Anleitungen/JIVE_Pro.pdf)

[Telemetry protocol](https://www.kontronik.com/fileadmin/kontronik-sobek/Public/Content/Images/Content/Downloads/Software/Kontronik_TelMe_V4.12_1.12_EN.pdf)
