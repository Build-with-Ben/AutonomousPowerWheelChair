# 10. Hardware

The physical parts the software runs on and drives: Raspberry Pi hardware and wheelchair hardware.

## 10.1 Compute

**10.1.1** The on-chair computer shall be a Raspberry Pi 3 (or newer) running Ubuntu Server 24.04, 64-bit. — P1 · I

**10.1.2** The Pi shall be powered from the chair battery through a DC-DC converter rated for the Pi's peak load plus the CAN adapter. The converter shall have its own fuse. — P1 · I

**10.1.3** The Pi shall show no under-voltage flags during a full test sequence (7.2.6). — P2 · T

**10.1.4** The Pi shall be mounted in an enclosure that protects it from rain spray and grass debris. — P3 · I

## 10.2 CAN interface

**10.2.1** The CAN transceiver shall be the same model on the dev PC and the Pi, so bench results carry over. — P1 · I

**10.2.2** The CAN adapter shall connect to the R-Net bus through an R-Net-compatible connector or a wired-in tap. The pinout shall be documented in this repository. — P1 · I

**10.2.3** The bus termination shall be checked: the existing R-Net termination shall be kept and no extra terminator added. — P1 · I

## 10.3 Hardware stop

**10.3.1** The chair shall have a hardware stop that removes drive power without relying on the Pi, ROS or any software. — P1 · D

**10.3.2** The hardware stop shall include a physical button on the chassis that is easy to reach. — P1 · I

**10.3.3** Before the chair drives with nobody at the chassis, a wireless stop shall be added that:

- **10.3.3.1** opens the stop circuit when its button is pressed
- **10.3.3.2** opens the stop circuit if its radio link is lost

— P2 · D

**10.3.4** The stop circuit state shall be readable by the Pi (e.g. through GPIO) for Safety (5.4.2). — P2 · T

**10.3.5** The stop shall bring the chair to a halt; the method (power-module power cut or R-Net-level stop) shall be chosen and documented after reviewing the power module's brake behavior. — P1 · A · *TBD*

## 10.4 Wheelchair base

**10.4.1** The R-Net power module, motors and batteries shall be checked and documented before the first powered drive:

- **10.4.1.1** model and serial
- **10.4.1.2** battery voltage and capacity
- **10.4.1.3** brake release behavior

— P1 · I

**10.4.2** A test stand or blocks shall lift both drive wheels clear of the ground for off-ground testing (8.3.1). — P1 · I

**10.4.3** The chair's manual brake release (freewheel) shall be known and reachable, so the chair can be pushed when unpowered. — P1 · I
