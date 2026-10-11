# Partial Refresh (局部刷新)

Clock mode updates the large time digits with a **partial refresh**: only the
time strip rectangle is rewritten and refreshed, the update completes in a few
hundred milliseconds without flashing. The header (date / lunar / battery /
temperature) and footer are still updated by full refreshes.

Currently implemented for **SSD1619** (400x300, both BW and BWR panels). Other
driver ICs fall back to full refresh automatically.

## How it works (SSD1619A)

The SSD1619A keeps a volatile 70-byte waveform table (LUT) in registers, loaded
via command `0x32`. The controller picks the LUT row **by the pixel's new
color** (`LUT0`=black, `LUT1`=white, `LUT2/3`=red, `LUT4`=VCOM), so no
old-frame tracking is needed on the MCU side — unchanged pixels simply receive
a short pulse in their current direction.

The partial update sequence (see `EPD/SSD16xx.c`):

1. `SSD16xx_InitPartial`: hardware reset → SW reset → minimal config →
   `0x21 ← 0x40` (bypass RED RAM as 0, black/white only) → `0x32 ← partial LUT`
2. Write only the time strip into RAM via the existing windowed
   `write_image()` (`0x44/0x45/0x4E/0x4F` window + `0x24`, red plane `0x26`
   written blank / bypassed)
3. `SSD16xx_RefreshPartial`: `0x22 ← 0xC7` (enable clock + analog + DISPLAY,
   **without** the load-LUT-from-OTP / load-temperature bits, so the register
   LUT stays in effect) → `0x20` Master Activation → wait BUSY

The waveform itself is a single ~10-frame pulse (~200 ms at the 50 Hz default
frame rate) toward each pixel's new color:

| LUT | drives | phase A |
|-----|--------|---------|
| LUT0 | black pixels | `10` = VSL |
| LUT1 | white pixels | `01` = VSH1 |
| LUT2/3 | red pixels | not driven (red bypassed) |
| LUT4 | VCOM | `00` = DCVCOM |

LUT byte layout (datasheet §6.6–6.7, Figure 6-6): bytes `0–6` = LUT0 groups
0–6, `7–13` = LUT1, `14–27` = LUT2/3, `28–34` = LUT4 (each byte packs phases
A,B,C,D, 2 bits each), bytes `35–69` = per group `TP[nA..nD], RP[n]` timing.

## Ghosting management

Partial pulses are not charge-balanced for unchanged pixels, so ghosting
accumulates slowly. Policy in `EPD/EPD_service.c`:

- clock mode uses partial refresh for every minute update
- a **full refresh** runs at day boundary (00:00, the header date changes),
  on mode switch / time set (`force_update`), and after
  `EPD_PARTIAL_REFRESH_MAX` (default 30) consecutive partials

The full refresh path is untouched; partial is only taken when
`drv->init_partial` / `drv->refresh_partial` are present (NULL for
unsupported models) and the mode is clock.

## Tuning

All knobs live in `EPD/SSD16xx.c`:

- `SSD16xx_LUT_PARTIAL_FRAMES`: pulse length in frames (default 10 ≈ 200 ms).
  Increase if new digits show weak contrast (e.g. to 15–20), decrease for
  faster updates.
- Weaker flips can also be improved by adding a second pulse group (groups
  1–6 are unused): e.g. group 1 with phase A = opposite polarity, short TP,
  before/after the main pulse.
- If the whole window stays unchanged (no update visible), first suspect the
  `0x22` sequence: try `0xCF` (DISPLAY Mode 2) instead of `0xC7`. The canned
  sequence list is in the datasheet command table for `0x22`.
- The register LUT is not temperature compensated; extreme temperatures may
  need longer pulses. The periodic full refresh corrects accumulated drift.

## Extending to other driver ICs

Implement `init_partial` / `refresh_partial` in the IC driver and attach them
to a dedicated `epd_driver_t` (see `epd_drv_ssd1619`). Note the schemes differ
by family:

- **SSD1619A**: LUT selected by pixel's *new* color — no old-frame bookkeeping
  (this implementation)
- **SSD1683-class** (UC-style old/new RAM planes): partial = write previous
  frame to old-data RAM and new frame to new-data RAM inside the window, then
  master activate with a sequence that skips OTP LUT reload
- **SSD1677**: uses a different (20/30-byte) LUT format via the same `0x32`
  command — a partial LUT for it can be derived the same way
- **UC81xx family**: partial LUTs via `LUTWW/LUTBW/LUTWB/LUTBB` (0x21–0x24)
  with `PTIN/PTL/PTOUT` (0x90–0x92) windowing, see `docs/README.md` for OTP
  waveform dumps of the stock panels

## Verification status

- Host-side tests (clang/MSVC): region geometry, page tiling, determinism and
  minute-change content assertions all pass for 400x300
- `SSD16xx.c` clean under `-Wall -fsyntax-only`
- **On-panel waveform validation is pending real hardware**: flash the
  firmware, set clock mode via the web UI, and watch a minute rollover —
  expected: no flash, digits swap in ~0.3 s, full flash refresh every 30th
  minute and at 00:00
