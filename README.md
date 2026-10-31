# Software GPS Receiver (Qt)

A post-processing GPS L1 C/A and Galileo E1 receiver for recorded IF sample files. It is built to show every
step of the signal processing: raw-signal statistics, C/A code generation, acquisition, tracking loops, bit
synchronisation, navigation-message decoding with parity or CRC, ephemeris/almanac/health, satellite orbits,
pseudoranges, and the least-squares position/velocity/time solution. GPS and Galileo can be used alone or
combined in one fix.

![Position page](docs/screenshots/17_Position_Summary__plots.png)

## Building and running in Qt Creator

1. Open `CMakeLists.txt` in Qt Creator (*File > Open File or Project*).
2. Select the **Desktop Qt 6.12.0 MinGW 64-bit** kit.
3. Choose the **Release** build configuration and press **Run** (Ctrl+R).
   Debug builds also work: the DSP core (`gnsscore`) is optimised even in Debug
   (option `GNSS_OPTIMIZE_CORE_IN_DEBUG`), so only the GUI is unoptimised.
4. On first start the program looks for a large `*.bin` file in the working directory and up to five levels above
   the executable, so `GPSdata-DiscreteComponents-fs38_192-if9_55.bin` is opened automatically. Otherwise use
   *File > Open*. After that the last file is reopened, unless you closed it.
5. Press **F5** (*Run all*), or run the stages one at a time with F6 (acquire), F7 (track), F8 (decode) and F9 (solve).
6. *File > Close recording* (Ctrl+W) closes the file and clears all results, plots and sample buffers, leaving the
   program as it was when it started. It is available when nothing is running, and works on a stopped live stream too.

The sampling frequency, IF and sample format come from the file name (`fs38_192` = 38.192 MHz,
`if9_55` = 9.55 MHz, int8 real samples by default, packed 2-bit real samples if the name contains `2bit`). You can
change them in *File > Settings*, along with every acquisition, tracking-loop and navigation parameter.

*File > Settings* also selects **GPS + Galileo**, **GPS only** or **Galileo only**. Searching for Galileo adds several
seconds to acquisition, so choose GPS only for a recording made before Galileo was operating.

Galileo satellites are named E and a number, such as E24, while GPS satellites go by their PRN. The Overview, Live and
Acquisition tables have a *System* column, and the bar charts continue from the GPS PRNs into E01 to E36. On the sky plots
GPS satellites are circles and Galileo satellites are squares, with a count of each in the corner.

The file name gives the IF rounded to 9.55 MHz. The true IF of this recording is fs/4 = 9.548 MHz. The receiver
copes either way, and the Position page reports the measured *effective IF* (9.547984 MHz) from the
Doppler/clock-drift solution. If you set the IF to 9.548 MHz, the Doppler values are shown without the
2 kHz offset.

## Live mode: USRP N200

Besides recordings, the receiver can process samples as they arrive from a USRP (written for the N200; other USRPs
that UHD supports should work too). Everything updates while it runs: a **Live** tab shows the radio, the channels and
the position, and whichever other page is open keeps refreshing, so the tracking loops, navigation message, orbits and
least-squares solution can all be watched on the running signal.

![Live page](docs/screenshots/23_Live.png)

### Setting up the N200

1. Install UHD from <https://files.ettus.com/binaries/uhd/>. The program looks for `uhd.dll` in `UHD_PKG_PATH`,
   `C:\Program Files\UHD\bin` and `PATH`, or you can give its path in *Settings > Live*. UHD is loaded when you start,
   so the program builds and runs without it.
2. Connect the N200 to a **wired gigabit Ethernet** port. Give that port a static address such as 192.168.10.1 with mask
   255.255.255.0 (the N200 ships as 192.168.10.2). Wi-Fi cannot carry this stream.
3. Check the radio with UHD's own tools. The N200's firmware and FPGA image must match the UHD version;
   `uhd_usrp_probe` says so if they do not.

   ```
   uhd_find_devices
   uhd_usrp_probe --args addr=192.168.10.2
   ```

4. The daughterboard must cover 1575.42 MHz (DBSRX2, WBX, SBX and UBX do). An active GPS antenna needs DC power:
   check whether your daughterboard supplies it, and use a bias tee if not.

### Running it

Set the address, sample rate and gain in *Settings > Live*, then press **F10** or *Start live* on the Live tab. Defaults
are `addr=192.168.10.2`, 5 MS/s at 1575.42 MHz and 30 dB.

* **First fix** takes about 30 to 60 s: a receiver with no stored data has to read a whole orbit from each satellite.
  Satellites are found within seconds, and the channel table shows each one's progress.
* **Gain:** watch *Signal level* on the Live tab, which warns when the level is too high or low.
* **Sample rate:** GPS works from 2 MS/s, Galileo needs 4 MS/s. The N200 offers 100 MHz divided by an integer, and the
  program uses the rate the device reports.
* **Overflows** (samples lost on the way to the computer) are counted on the Live tab. The gap is filled so later
  samples stay at the right time and the loops coast across it. Adding `recv_buff_size=50000000` to the device address
  can help, and so can using a lower sample rate.
* **Examining a page:** tick *Freeze pages* to stop the display while the receiver keeps running. Plots you zoom keep
  their zoom across updates; double-click to release it.
* **Recording:** tick *Record the samples to disk* in *Settings > Live* (20 MB/s at 5 MS/s). The file name carries
  the rate and format, so *File > Open* processes it later like any recording.

The tracking pages show the last 60 s and the Position pages the last 10 minutes (both adjustable). Long sessions use
constant memory.

### Trying it without a radio

Set *Live source* to *Replay the open recording in real time* in *Settings > Live*: the open file then behaves like a
stream, with no hardware or UHD.

The USRP code itself can be tested through `tests/fake_uhd`, a stand-in `uhd.dll` that streams a file and can simulate
lost packets and signal loss. It is built with `-DGNSS_BUILD_TESTS=ON`.

## Editing the GUI in Qt Designer

Every page, the main window and the settings dialog are `.ui` forms in `src/gui/`:

| Form | Content |
|---|---|
| `MainWindow.ui` | menus, toolbar, log dock, and the tab widget holding all pages (each tab is a promoted page widget) |
| `SettingsDialog.ui` | input, acquisition, tracking and navigation settings |
| `OverviewPage.ui` | recording info, pipeline status, satellite summary, position summary, algorithm documentation |
| `RawSignalPage.ui` | time-domain samples, ADC histogram, Welch PSD, spectrogram, statistics |
| `CaCodePage.ui` | G1/G2/C-A sequences, autocorrelation, cross-correlation, sampled correlation peak, code spectrum, code table |
| `AcquisitionPage.ui` | metric and C/N0 bars, results table, correlation surface heat map, code/Doppler slices, fine-frequency spectrum |
| `TrackingPage.ui` | 12 per-channel loop plots, all-channel overlays, multi-correlator shape, loop design tables |
| `BitSyncPage.ui` | transition histogram, prompt with bit edges, soft bits, preamble correlation, raw bit grid |
| `NavMessagePage.ui` | subframe list, words with parity check, colour-coded bit map, decoded fields with scaling and interpretation |
| `EphemerisPage.ui` | ephemeris table, almanac, health/A-S/configuration, iono/UTC, special message, NMCT, received pages |
| `SatellitesPage.ui` | sky plot, elevation, sub-satellite points, satellite state, step-by-step Kepler computation |
| `MeasurementsPage.ui` | per-epoch pseudorange corrections table, PR change, residuals, code-minus-carrier, Doppler, tropo, SV clock |
| `LivePage.ui` | live dashboard: radio status, channel table, signal strength, sky plot, recent fixes |
| `PositionPage.ui` | summary, scatter/ENU/clock/DOP/velocity plots, solutions table, least-squares iterations, H and Q matrices |

Notes for editing:

* The plots and tables are **promoted widgets**: `gui::PlotWidget` and `gui::SkyPlotWidget` (from `QWidget`) and
  `gui::DataTable` (from `QTableWidget`). You can move, resize, split or regroup them freely.
* Plot titles and axis labels are dynamic properties of the promoted widget (`title`, `xLabel`, `yLabel`, plus
  `aspectLocked` and `legendVisible`). Edit them in the Property Editor. Some titles are replaced at run time
  with PRN-specific text.
* Keep the **objectName** of any control the code uses (e.g. `channelCombo`, `epochSpin`, `surfacePlot`). The code
  accesses them as `ui->name`.
* Table **column headers are set in code** (`setColumns(...)` in each page's constructor), because the code fills
  the table by column index. Change the header list and the matching `setNum/setText` column numbers together.
* The combo boxes in `SettingsDialog.ui` list their items in enum order (sample format, Doppler sign, constellations). Keep that order.

Plot interaction: mouse wheel zooms (Ctrl = x only, Shift = y only), left-drag box-zooms, right-drag pans,
double-click resets, and right-click opens a menu (copy/save image, export CSV, legend, grid). The tracking-page
time axes are linked. Tables support Ctrl+C and *Export to CSV* from the context menu.

## Command line

```
gnss_cli <file.bin> [--ms N] [--channels N] [--fs Hz] [--if Hz] [--gnss all|gps|gal] [--csv solutions.csv]
SoftGpsReceiver <file.bin> --screenshots <dir>   # process, save a PNG of every page and sub-tab, exit

gnss_cli <file.bin> --live [--speed X] [--seconds N]    # play a recording in real time
gnss_cli --usrp [--args addr=192.168.10.2] [--rate 5e6] [--gain 30] [--antenna RX2] [--record dir] [--seconds N]
```

## Results for the sample recording

`GPSdata-DiscreteComponents-fs38_192-if9_55.bin` is 50.08 s of 4-bit-quality int8 real samples.

* 9 satellites acquired (PRN 3, 6, 9, 15, 18, 21, 22, 26, 29). All tracked with 100 % phase lock at 42–51 dB-Hz.
* 63 subframes decoded, 630/630 words pass parity, and all 9 ephemerides complete and consistent.
* 99 fixes (every 0.5 s): **40.00802° N, 105.26270° W, h = 1634 m** (Boulder, Colorado), with σ E/N/U =
  1.95/2.08/3.27 m and PDOP 1.65.
* GPS week 297 (mod 1024), i.e. 2005-05-07 19:10 UTC.
* Sample clock error +10.4 ppb (clock-bias slope 3.1 m/s). Effective IF 9.547984 MHz.
* Subframe 4 page 18 (ionosphere/UTC) is broadcast every 12.5 min and is not in this 50-s recording, so no
  ionospheric correction is applied (residual RMS ≈ 5 m).

Processing the whole file takes about 13 s on a 12-core CPU: acquisition 2 s, tracking 9 s with one thread per channel.
With Galileo enabled, the search for it adds several seconds to acquisition.

## Results with Galileo

The GPS-only recordings above cannot exercise Galileo. The test recording is `IFEN_Bands-L1.stream` from the
[ION GNSS SDR sample data](https://sdr.ion.org/api-sample-data.html): 18.6 minutes of packed 2-bit real samples at
20.48 MHz with an IF of 5.5 MHz. It is not part of this repository. Save it as
`IFEN_Bands-L1-fs20_48-if5_5-2bit.stream` and those settings are read from the name.

The file's metadata gives 2016-01-19 at a fixed position. The navigation data shows that it was recorded on
2017-02-06, near 48.08 N, 11.64 E, and that the receiver was in a vehicle moving at up to 14 m/s for much of the time.

* 8 GPS and 5 Galileo satellites acquired (E02, E08, E19, E20, E30). E1-B and E1-C agree on delay and Doppler for all five.
* 1860 of 2046 I/NAV words pass the CRC, with no TOW inconsistencies. The failures come from lost tracking, mostly late
  in the recording, not from random errors.
* E20 broadcasts only dummy words (type 63), so it is tracked and decoded but gives no ephemeris. The other four share
  IODnav 91.

For the first 58 s, while the vehicle is stationary:

| Satellites | Latitude | Longitude | Height | std E/N/U | PDOP |
|---|---|---|---|---|---|
| 8 GPS | 48.0781398 | 11.6383598 | 601.73 m | 1.83 / 2.18 / 6.53 m | 1.88 |
| 4 Galileo | 48.0781221 | 11.6383477 | 598.52 m | 1.77 / 2.98 / 1.85 m | 2.28 |
| 8 GPS + 4 Galileo | 48.0781303 | 11.6383537 | 600.19 m | 1.44 / 2.00 / 4.24 m | 1.39 |

The two independent solutions agree to about 3 m. The Galileo-GPS bias is 2.0 m, in line with the broadcast time offset of
3.8 ns plus the different receiver delays of the two signals. Comparing the Galileo-only and GPS-only tracks epoch by epoch
over the first 710 s, including driving, gives a mean difference of less than 1 m horizontally and 3.6 m vertically, and RMS
differences of 2.8 m east and 3.4 m north. Galileo-only fixes are available for about 1000 of those 1420 epochs.

After about 710 s every satellite weakens at once, at all elevations, most likely from obstructions along the route, and the
weakest signals are lost. No Galileo-only fix is possible from then on: of the Galileo satellites only E30 and E20 stay
tracked, and E20 has no ephemeris. The receiver has no outlier rejection, so the GPS and combined solutions then show 60 to
80 jumps of up to about 100 m. There are none before that point.

The whole recording takes about 3 minutes on a 12-core CPU and 6.5 GB of memory, most of it for the per-epoch tracking
history. Limit *Data to process* for a quicker run.

### Adding simulated Galileo satellites to a GPS recording

`tests/add_galileo.cpp`, built with `-DGNSS_BUILD_TESTS=ON`, makes a recording that exercises Galileo from one that has
none. It takes an int8 real recording and writes a new one:

```
add_galileo <in.bin> <out.bin> [--sats 6] [--cn0 47] [--isb 3]
```

It solves the recording with GPS, then computes E1-B and E1-C signals for made-up satellites in a Walker 24/3/1
constellation, arriving as they would at that position on that receiver's clock. They carry a full I/NAV message apart
from the almanac. `--cn0` is the strength at the zenith, and `--isb` is the Galileo-GPS time offset in metres.

The samples are multiplied by a whole number first, because a signal well under one count would otherwise disappear
when rounded. The GPS signals are unchanged, but the raw signal pages and signal levels show the larger scale.

Results from `GPS_Data_NordNav1e_fs16_3676-if4_1304.bin` with the default settings:

| Satellites | Latitude | Longitude | Height | std E/N/U |
|---|---|---|---|---|
| 6 GPS, original file | 32.59086653 | -85.48919624 | 178.98 m | 2.73 / 2.63 / 5.87 m |
| 6 Galileo | 32.59086767 | -85.48919626 | 179.01 m | 0.66 / 1.09 / 1.55 m |
| 6 GPS + 6 Galileo | 32.59086694 | -85.48919640 | 178.98 m | 1.58 / 1.14 / 3.26 m |

This tests the receiver chain and is not evidence of Galileo accuracy. The simulated signals have no multipath,
ionosphere or front-end filtering, which is why their scatter is so small, and they take their position from the GPS
solution and use the receiver's own orbit and atmosphere code, so agreement between the rows is expected.

## Screenshots

All screenshots below come from processing the sample recording. You can regenerate them with
`SoftGpsReceiver <file.bin> --screenshots <dir>`.

### Overview
Recording parameters, pipeline status, a per-satellite summary (acquisition, tracking, bit sync, parity,
ephemeris, health, elevation) and the built-in description of the algorithms.

![Overview](docs/screenshots/01_Overview.png)

### Raw signal
Time-domain samples, ADC level histogram against a Gaussian, Welch power spectral density showing the
front-end passband around the IF, spectrogram, and signal statistics.

![Raw signal](docs/screenshots/02_Raw_signal.png)

### C/A codes
G1, G2<sub>i</sub> and C/A chip sequences, the three-valued autocorrelation, cross-correlation between two PRNs,
the correlation peak sampled at the receiver rate, the 1 kHz line spectrum, and a table that checks all 32 codes
against IS-GPS-200.

![C/A codes](docs/screenshots/03_C-A_codes.png)

### Acquisition
Detection metric and C/N0 for every PRN, the full results table, and for the selected PRN: the code-phase ×
Doppler correlation surface, code-phase and Doppler slices through the peak, and the fine-frequency search.

![Acquisition](docs/screenshots/04_Acquisition.png)

### Tracking
**Channel detail:** prompt I/Q scatter, prompt correlator outputs (navigation bits), early/prompt/late
envelopes, PLL, FLL and DLL discriminators, carrier Doppler, code NCO, C/N0, phase lock indicator, accumulated
Doppler and code-epoch drift, all on linked time axes.

![Tracking - channel detail](docs/screenshots/05_Tracking_Channel_detail.png)

**All channels:** C/N0, Doppler, phase lock indicator and DLL error overlaid for every channel.

![Tracking - all channels](docs/screenshots/06_Tracking_All_channels.png)

**Correlator shape:** the correlation function recomputed with a bank of correlators from the stored NCO
state, showing the peak rounded by the front-end filter, with the loop's early/prompt/late points on it.

![Tracking - correlator shape](docs/screenshots/07_Tracking_Correlator_shape.png)

**Summary & loop design:** per-channel statistics and the PLL/FLL/DLL loop-filter coefficients.

![Tracking - summary and loop design](docs/screenshots/08_Tracking_Summary__loop_design.png)

### Bit sync & data bits
Histogram of prompt sign transitions modulo 20 ms, prompt I with the detected bit edges, the soft bits and their
distribution, the bit stream correlated with the preamble (parity-verified subframe starts in red), and the raw
bit grid.

![Bit sync and data bits](docs/screenshots/09_Bit_sync__data_bits.png)

### Navigation message
Every decoded subframe (TOW, page, parity, polarity, flags). For the selected subframe: each word with its
received and computed parity, a colour-coded map of the 300 bits by field, and every field with raw bits, scale
factor, value, units and interpretation.

![Navigation message](docs/screenshots/10_Navigation_message.png)

### Ephemeris, almanac & health
Broadcast ephemeris and clock parameters for all satellites, decoded almanac pages, satellite health from
subframe 1, the almanac and the page-25 summaries, and the ionosphere/UTC, special-message, NMCT and
received-pages tables.

![Ephemeris](docs/screenshots/11_Ephemeris,_almanac__health_Ephemeris_%28subframes_1-3%29.png)

![Almanac](docs/screenshots/12_Ephemeris,_almanac__health_Almanac.png)

![Health and configuration](docs/screenshots/13_Ephemeris,_almanac__health_Health__configuration.png)

![Iono, UTC and other pages](docs/screenshots/14_Ephemeris,_almanac__health_Iono_-_UTC__other_pages.png)

### Satellites & orbits
Sky plot with tracks, elevation over time, sub-satellite points, satellite position/velocity/clock at the
selected epoch, and the IS-GPS-200 orbit computation step by step (including each Kepler iteration).

![Satellites and orbits](docs/screenshots/15_Satellites__orbits.png)

### Pseudoranges
Every correction applied to each pseudorange at the selected epoch, with pseudorange change, post-fit residuals,
code-minus-carrier, measured vs predicted Doppler, tropospheric delay and satellite clock correction over time.

![Pseudoranges](docs/screenshots/16_Pseudoranges.png)

### Position
**Summary & plots:** mean position, accuracy statistics, horizontal scatter with CEP/2DRMS circles, E/N/U
error, receiver clock bias and drift, DOP, satellites used, and Doppler velocity.

![Position - summary and plots](docs/screenshots/17_Position_Summary__plots.png)

**Solutions:** every epoch's fix (geodetic, ECEF, ENU, clock, DOP, velocity, residual RMS), exportable to CSV/KML.

![Position - solutions](docs/screenshots/18_Position_Solutions.png)

**Least-squares detail:** every Gauss–Newton iteration from the Earth's centre to convergence, the design
matrix and residuals, and the cofactor matrix in the local E/N/U/clock frame.

![Position - least-squares detail](docs/screenshots/19_Position_Least-squares_detail.png)

### Galileo

Sky plot with GPS and Galileo together, the BOC(1,1) correlation function measured on E30, with its negative side lobes,
and the fields of an I/NAV word:

![Sky plot with Galileo](docs/screenshots/20_Galileo_Sky_plot.png)

![BOC(1,1) correlation function](docs/screenshots/21_Galileo_Correlator_shape.png)

![I/NAV word](docs/screenshots/22_Galileo_Navigation_message.png)

## Code structure

```
src/core/            GUI-independent receiver (Qt Core + Concurrent only)
  Settings           parameters, file-name parsing, persistence
  SignalSource       memory-mapped sample file (int8/uint8/int16, real or I/Q)
  Fft                radix-2 + Bluestein FFT (any length)
  CaCode             G1/G2 Gold code generator (tap and delay methods), ICD table
  SignalAnalysis     statistics, Welch PSD, spectrogram
  Acquisition        parallel code-phase search, fine frequency estimation
  Tracking           FLL-assisted Costas PLL + early/late DLL, C/N0, lock detector, multi-correlator
  NavMessage         bit sync, preamble search, parity, IS-GPS-200 field layouts, ephemeris/almanac/health/iono
  Ephemeris          satellite position/velocity/clock (Table 20-IV) with step-by-step trace
  Geodesy            ECEF/geodetic/ENU/UTM, az/el, Saastamoinen, Klobuchar, GPS time
  Pvt                pseudoranges, corrections, iterative least squares, DOP, Doppler velocity
  Receiver           pipeline orchestration (runs on a worker thread)
  LiveEngine         real-time receiver: channels start and stop, rolling decoding, incremental solution
  LiveInput          sample sources for it: UhdInput (USRP through the UHD C API), ReplayInput (a file at its real rate)
  LiveRing           sample ring buffer between the radio thread and everything that reads samples
  GalileoCodes       E1-B and E1-C primary codes (memory codes, from a table)
  GalileoNav         I/NAV: page sync, deinterleaver, Viterbi decoder, CRC-24Q, word fields, ephemeris
src/gui/             Qt Widgets front end (.ui forms + page classes, PlotWidget, SkyPlotWidget, DataTable)
src/cli/             command-line front end
tests/               decoder test against the worked example in the Galileo ICD (option GNSS_BUILD_TESTS)
docs/algorithms.html algorithm description shown on the Overview page
resources/           application icon (make_icon.py regenerates the .ico)
```

## Acknowledgements

The Galileo E1 primary codes come from Annex C of the European GNSS (Galileo) Open Service Signal-In-Space Interface
Control Document, Issue 2.1, (c) European Union 2023, and are used under the Authorisation in its Annex H. The tables were
transcribed from GNSS-DSP-tools and checked against GNSS-SDR, and the codes in use here are confirmed by correlating them
with live signals. This project is not endorsed by the EU.
