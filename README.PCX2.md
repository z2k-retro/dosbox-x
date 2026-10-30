# PowerVR PCX2 in DOSBox-X

This branch adds an emulated PowerVR PCX2, the chip on the Matrox m3D and
VideoLogic Apocalypse 3Dx. It is aimed at DOS games that drive the card
directly, chiefly the PowerVR version of Tomb Raider (TOMBPCX2.EXE) and
Unfinished Business.

The PCX2 has no video output of its own. It reads its scene from system RAM,
renders it, and writes the finished pixels into the 2D card's VESA linear
frame buffer. So it needs an SVGA machine type with a linear frame buffer,
such as the default S3.

## Settings

```ini
[dosbox]
machine=svga_s3
memsize=32
vmemsize=4

[cpu]
cputype=pentium
cycles=max

[powervr]
pcx2=true
# pcx2 irq=11
# pcx2 render threads=0      (0 = automatic)
# pcx2 render delay=2.0      (ms of emulated time per frame)
```

The PCI bus must be on (`enable pci bus=true` in `[dosbox]`, the default).

## Tomb Raider

From the game folder inside DOSBox-X, run `TOMBPCX2`. If it does not find a
VESA mode, run `TOMBPCX2 uvbe` instead. DOSBox-X's S3 BIOS already provides
VBE 2.0 with a linear frame buffer, so UniVBE itself is not needed.

If the 3D picture is garbage or missing while the menus work:

* Try `ems=false` in `[dos]`. The DOS driver has to know the physical
  address of its parameter buffer. Under a memory manager (VCPI) a program's
  addresses and the physical addresses can differ.
* Try `pcx2 render delay=0` or a larger value if the game hangs waiting for
  a frame.
* Start DOSBox-X from a console, or look in its log. The device logs where
  it sits on the bus and the registers of the first frame, beginning `PCX2:`.

## How it works

* The device is PCI 1033:0046 rev 1.
* BAR0 is the register file and BAR1 is the 4 MB texture memory.
* Writing STARTRENDER makes the device:
  1. read the ISP parameter pages through the 128-entry page table (TLB);
  2. render the frame with the renderer from the 86Box PCX2 device, on host
     threads;
  3. write the pixels the chip drew to SOFADDR in the PACKMODE format;
  4. report end of render (INTSTATUS = 7, plus the IRQ if INTMASK enables
     it) after the render delay.

To test it, a captured GLQuake frame (PCXDUMP1) was replayed through the
device from a DOS/4GW program. The frame buffer matched the capture from a
real m3D byte for byte.

The renderer (`src/hardware/pcx2/`) is MIT-licensed. The device
(`src/hardware/pcx2.cpp`) is GPL-2.0-or-later, like DOSBox-X.
