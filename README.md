   # PicoCalc driver sources

   This repository provides the out-of-tree Linux kernel driver sources and a top-level build Makefile for the PicoCalc device and related hardware.

   For Yocto/OpenEmbedded consumption and BitBake recipes, see the `meta-picocalc-drivers` or `meta-calculinux` repositories which contain recipes that can fetch and package these sources for images.

   What this repo contains
   - A top-level Makefile that can build all drivers against a kernel source (`KERNEL_SRC` / `KSRC`).
   - Subdirectories for individual drivers (MFD core and sub-drivers, legacy keyboard, LCD, sound drivers, etc.).

   Quick standalone build
   1. Ensure you have a matching kernel build directory (for example the kernel source or headers for the target kernel):

   ```bash
   # KERNEL_SRC should point to the kernel build directory (the directory with the top-level Makefile)
   export KERNEL_SRC=/path/to/kernel/build
   # Build everything
   make KERNEL_SRC=${KERNEL_SRC} all

   # Install modules into a staging directory (honors INSTALL_MOD_PATH)
   make KERNEL_SRC=${KERNEL_SRC} INSTALL_MOD_PATH=/tmp/picocalc-mods modules_install
   ```

   Notes:
   - The Makefile delegates to the kernel build system (it runs `make -C ${KERNEL_SRC} M=$(PWD)/<driver> modules`).
   - You can then copy the produced `.ko` files from the install tree to your target rootfs or package them for your distribution.

   Display drivers
   - Two panel controllers ship in the field: the Ilitek ILI9488 (`picocalc_lcd_fb`, `picocalc_lcd_drm`) and the Sitronix ST7365P (`picocalc_lcd_fb_st7365p`, `picocalc_lcd_drm_st7365p`). The register maps match, but the drivers are kept separate so each panel's init sequence and tuning can move independently.
   - Which driver binds is decided by the panel node's `compatible`. The base device tree describes an ILI9488; apply `devicetree-overlays/st7365p-lcd-overlay.dts` on units fitted with an ST7365P.
   - Pick either the fbdev driver or the DRM driver for a given panel, not both.

   SPI bus tuning
   - Register writes and pixel data are clocked separately. Register writes stay at 10 MHz, where a glitch would corrupt panel state; the bulk pixel transfer runs at the panel node's `spi-max-frequency`.
   - The reachable SCLK is half the spi0 functional clock, because the Rockchip SPI block divides by an even value of at least 2. `picocalc-luckfox-lyra.dtsi` pins that clock with `assigned-clock-rates`, so raising `spi-max-frequency` alone is not enough.
   - Stock `spi-rockchip` clamps SCLK to 50 MHz for every Rockchip SoC through one shared constant. `kernel-patches/0001-spi-rockchip-raise-MAX_SCLK_OUT-to-100MHz.patch` lifts that to 100 MHz, which is what spi0 reaches on the rk3506. The patch only raises the ceiling; without it a node asking for more is silently clamped, so the device tree here is safe either way. Nothing above 50 MHz has been validated on hardware.
   - Both ST7365P drivers override `spi-max-frequency` from a module parameter, which is the quick way to find a panel's ceiling without reflashing a device tree: `st7365p_drm.spi_speed_hz` and `st7365p_fb.p_spi_speed_hz`. The fbdev driver additionally exposes `p_cmd_speed_hz` and `p_bpw16`.
   - `st7365p_fb` sends RGB565 as 16-bit SPI words by default, so frames go from the framebuffer to the SPI core with no byte-swap pass and no bounce buffer. Set `p_bpw16=0` to fall back to the byte-swapped 8-bit path.

   Kernel patches
   - `kernel-patches/` holds patches against `Calculinux/luckfox-linux-6.1-rk3506` for things that live in the kernel rather than in this repo. See the README in that directory for what each one does and how to apply it.

   Device tree / runtime notes
   - The drivers support both a legacy keyboard driver and a new MFD-based keyboard. These can be installed simultaneously; the device tree 'compatible' strings and driver probe order control which driver binds at runtime.
   - The MFD core driver populates child platform devices for sub-drivers (keyboard, backlight, BMS, LED).
   - Device tree source-of-truth lives in this repo. Linux files are prefixed `linux-` and U-Boot files are prefixed `uboot-`.
      - Linux: `linux-rk3506-luckfox-lyra.dtsi`, `linux-rk3506g-luckfox-lyra.dts`
      - U-Boot: `uboot-rk3506-luckfox.dtsi`, `uboot-rk3506-luckfox.dts`
      The Yocto `picocalc-devicetree` recipe installs these into the sysroot using the historical filenames (no prefixes), so kernel/U-Boot recipes do not need to change.

   Contributing
   - Keep each driver in its own subdirectory with a small `Makefile`/Kbuild fragment if needed (see existing subdirs).
   - Prefer the kernel build system's `modules`/`modules_install` rules rather than custom build logic.
   - Update licensing and headers for any new files, and add tests or instructions where appropriate.

   License
   - Kernel parts are GPL-compatible; check each driver subdirectory for the specific licensing header.

   Contact
   - Raise issues or PRs against the Calculinux organization repositories on GitHub.
