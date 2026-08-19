# Kernel patches

Patches against the Calculinux kernel tree
(`Calculinux/luckfox-linux-6.1-rk3506`). These touch files that live in the
kernel, not in this repo, so they cannot be carried as driver code.

Apply order is the numeric prefix. Each patch is `git format-patch` output, so
`git am` keeps the commit message and authorship:

```bash
git am /path/to/picocalc-drivers/kernel-patches/*.patch
```

Consumers that patch a source tree rather than a git repo (Yocto `SRC_URI`,
quilt) can use them directly as `-p1` patches.

Verified to apply cleanly at `e0846929` (the revision pinned by this repo's
build-test workflow) and at current `main`.

## 0001-spi-rockchip-raise-MAX_SCLK_OUT-to-100MHz

`spi-rockchip` caps SCLK at 50MHz through a single constant shared by every
SoC it supports, rk3036 through rk3506. The hardware does not require that
value: BAUDR is a 16-bit even divider with a minimum of 2, so anything up to
spiclk/2 is representable. On rk3506 with spi0's functional clock pinned at
200MHz that is 100MHz.

This matters for the display, which is the one device on the PicoCalc whose
throughput is bounded by the SPI bus. A 320x320 RGB565 frame is 200KiB, so
the cap is worth roughly half the achievable frame rate.

The patch raises the ceiling only. It changes no board's behaviour on its own
— `spi-max-frequency` and the clock tree still choose the rate, and a node
asking for 50MHz still gets 50MHz. Without the patch, a node asking for more
is silently clamped, so the device tree in this repo is safe either way.

Nothing here has been validated on a scope. Signal integrity above 50MHz on
the PicoCalc's panel ribbon is unverified, as is the ST7365P's own SCLK
rating. Treat 100MHz as the ceiling to explore towards, not a known-good
setting, and use the LCD drivers' `spi_speed_hz` / `p_spi_speed_hz` module
parameters to find the real limit before committing a rate to the device tree.
