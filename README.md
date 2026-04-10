# esphome-webasto

First of all, I am not a good programmer. I took over a repository from @felixwelsch that could not be compiled with the current version of ESPHome. With this repository, ESPHome Build 2024.10 can also be installed, which works without any problems.
https://github.com/khenderick/esphome-legacy-addons


The files espSharan.yaml and webasto.h need to be placed in the ESPHome folder. In webasto.h, you may need to try different values in lines 12–16 to see what works.

I have a WEBASTO Thermo Top VEVO in a VW Sharan that is coded as an auxiliary heater and not as a parking heater. @felixwelsch used different values there.

In this code, I left out my control for the Sharan Climatronic blower that I use, because otherwise it would become very confusing. If you have any questions about that, feel free to ask.
