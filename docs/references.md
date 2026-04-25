# References & Citations

> Full annotated reference list for the bladerf-gps-spoofer project.
> Every mathematical derivation, algorithm, and constant in this codebase
> traces back to one of the sources below.

---

## Primary Standards

### IS-GPS-200 (Interface Specification)
**Interface Specification IS-GPS-200: GPS SPS Signal Specification**
U.S. Air Force, Global Positioning Systems Directorate.
Latest revision: IS-GPS-200L (2020).

This is the authoritative source for every GPS L1 C/A signal parameter in this project:
- §3.3 — Navigation signal characteristics (L1 frequency, BPSK modulation, C/A chip rate)
- §20.3.3.4.3 — Algorithm for computing satellite ECEF position from ephemeris
- §20.3.5.2 — Navigation message parity encoding (Hamming)
- Table 3-Ia — C/A code phase assignments (G2 phase offsets for PRN 1–32)
- §6.3.2.3 — Keplerian orbital element definitions

**Access:** [https://www.gps.gov/technical/icwg/IS-GPS-200L.pdf](https://www.gps.gov/technical/icwg/IS-GPS-200L.pdf)
*(Free, public domain — US government publication)*

---

### IS-GPS-705
**Interface Specification IS-GPS-705: GPS IIF/IIIA L5 Signal**
U.S. Air Force, Global Positioning Systems Directorate.

Consulted for: GPS system architecture overview, satellite constellation parameters.

**Access:** [https://www.gps.gov/technical/icwg/IS-GPS-705H.pdf](https://www.gps.gov/technical/icwg/IS-GPS-705H.pdf)

---

### WGS-84
**World Geodetic System 1984: Its Definition and Relationships with Local Geodetic Systems**
National Geospatial-Intelligence Agency (NGA), Technical Report TR8350.2, 3rd edition.

Used for: Earth gravitational constant μ = 3.986005 × 10¹⁴ m³/s², Earth rotation rate
Ωₑ = 7.2921151467 × 10⁻⁵ rad/s, speed of light c = 2.99792458 × 10⁸ m/s.

**Access:** [https://earth-info.nga.mil/GandG/publications/tr8350.2/tr8350_2.html](https://earth-info.nga.mil/GandG/publications/tr8350.2/tr8350_2.html)

---

## Textbooks

### Tsui (2000)
**Fundamentals of Global Positioning System Receivers: A Software Approach**
James Bao-yen Tsui.
Wiley-Interscience, 2000. ISBN: 978-0471385981.

Used for: C/A code generator circuit description (Ch. 3), signal acquisition theory,
correlation properties of Gold codes, software receiver architecture.

---

### Misra & Enge (2006)
**Global Positioning System: Signals, Measurements and Performance**
Pratap Misra & Per Enge. 2nd edition.
Ganga-Jamuna Press, 2006. ISBN: 978-0970954428.

Used for: Complete GPS signal model derivation (Ch. 2), navigation message structure (Ch. 4),
satellite position computation (Ch. 4.3), ionospheric and tropospheric error models.

---

### Borre et al. (2007)
**A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach**
Kai Borre, Dennis M. Akos, Nicolaj Bertelsen, Peter Rinder, Søren Holdt Jensen.
Birkhäuser, 2007. ISBN: 978-0817643904.

Used for: MATLAB/software receiver reference, RINEX file format details,
pseudorange measurement model, BPSK signal generation.

**Companion code:** [https://www.gnss-sdr.org/docs/sp-blocks/](https://www.gnss-sdr.org/docs/sp-blocks/)

---

### Kaplan & Hegarty (2017)
**Understanding GPS/GNSS: Principles and Applications**
Elliott D. Kaplan & Christopher J. Hegarty. 3rd edition.
Artech House, 2017. ISBN: 978-1630810580.

Used for: Comprehensive GPS system overview (Ch. 1–3), signal structure tables,
receiver architecture reference.

---

## Open-Source Reference Implementations

### gps-sdr-sim
**GPS Signal Simulator**
Takuji Ebinuma (osqzss).
GitHub: [https://github.com/osqzss/gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim)
License: MIT

The primary open-source reference implementation this project was benchmarked against.
The C/A code generator, navigation message encoder, and channel summation approach
in `core/` were validated against gps-sdr-sim output before any RF transmission.

Key differences from gps-sdr-sim:
- Live bladeRF streaming (no intermediate file)
- REST API + React UI for real-time position updates
- CMake + modern C++20 codebase
- Moving path (GPX track) mode

---

### GNSS-SDR
**An Open Source Global Navigation Satellite Systems Software Defined Receiver**
Carles Fernández-Prades, Javier Arribas, et al.
GitHub: [https://github.com/gnss-sdr/gnss-sdr](https://github.com/gnss-sdr/gnss-sdr)
License: GPLv3

Used as: Verification receiver. Generated I/Q samples were decoded with GNSS-SDR
to confirm correct signal generation before hardware testing.

**Paper:** Fernández-Prades, C. et al. (2011). GNSS-SDR: An Open Source Tool For
Researchers and Developers. Proc. ION GNSS 2011.
[https://www.ion.org/publications/abstract.cfm?articleID=9640](https://www.ion.org/publications/abstract.cfm?articleID=9640)

---

## Hardware Documentation

### bladeRF 2.0 micro
**bladeRF 2.0 micro Hardware Reference**
Nuand LLC.

- bladeRF wiki: [https://github.com/Nuand/bladeRF/wiki](https://github.com/Nuand/bladeRF/wiki)
- libbladeRF API: [https://nuand.com/libbladeRF-doc/](https://nuand.com/libbladeRF-doc/)
- SC16Q11 sample format: [https://github.com/Nuand/bladeRF/wiki/SC16Q11-Format](https://github.com/Nuand/bladeRF/wiki/SC16Q11-Format)

Key parameters used:
- Sample format: SC16Q11 (16-bit signed I, 16-bit signed Q)
- TX sample rate range: 520 ksps – 61.44 Msps
- TX frequency range: 47 MHz – 6 GHz (covers GPS L1 at 1575.42 MHz)
- TX gain range: -23.5 to +66.5 dB

---

## RINEX Data Sources

### IGS RINEX Navigation Files
**International GNSS Service — Navigation Data**

Daily broadcast ephemeris files (RINEX 3 format) used by `tools/rinex_fetch.py`:

- IGS FTP: [https://cddis.nasa.gov/archive/gnss/data/daily/](https://cddis.nasa.gov/archive/gnss/data/daily/)
- RINEX 3 format specification: [https://www.igs.org/wg/rinex/](https://www.igs.org/wg/rinex/)

RINEX 3 Navigation File Format: RINEX Working Group and IGS Analysis Center Coordinator.
*RINEX The Receiver Independent Exchange Format Version 3.05*. IGS, 2021.
[https://files.igs.org/pub/data/format/rinex305.pdf](https://files.igs.org/pub/data/format/rinex305.pdf)

---

## Papers on GPS Spoofing / SDR

These papers provide additional context on GPS signal synthesis and spoofing detection
and were consulted during the design of the signal generator architecture.

### Humphreys et al. (2008)
Humphreys, T.E., Ledvina, B.M., Psiaki, M.L., O'Hanlon, B.W., Kintner, P.M. (2008).
**Assessing the Spoofing Threat: Development of a Portable GPS Civilian Spoofer.**
*Proc. ION GNSS 2008*, Savannah, GA.
[https://cornell.edu/~psiaki/humphreys_etal_iongnss2008.pdf](http://gps.ece.cornell.edu/pubs/Humphreys_etal_ION_GNSS_2008.pdf)

*The first published open description of a civilian GPS spoofer architecture.*

### Psiaki & Humphreys (2016)
Psiaki, M.L., Humphreys, T.E. (2016).
**GNSS Spoofing and Detection.**
*Proceedings of the IEEE*, 104(6), 1258–1270.
DOI: [10.1109/JPROC.2016.2526658](https://doi.org/10.1109/JPROC.2016.2526658)

*Comprehensive survey of spoofing techniques and countermeasures.*

---

## Gold Code Theory

### Gold (1967)
Gold, R. (1967).
**Optimal binary sequences for spread spectrum multiplexing.**
*IEEE Transactions on Information Theory*, 13(4), 619–621.
DOI: [10.1109/TIT.1967.1054048](https://doi.org/10.1109/TIT.1967.1054048)

*The original paper defining Gold codes and their cross-correlation properties.*

### Proakis (2001)
Proakis, J.G. (2001). *Digital Communications*, 4th edition.
McGraw-Hill. ISBN: 978-0072321111.

Used for: BPSK modulation theory (Ch. 5), spread-spectrum fundamentals (Ch. 8),
correlation receiver derivation.

---

## Orbital Mechanics

### Vallado (2013)
Vallado, D.A., McClain, W.D. (2013).
*Fundamentals of Astrodynamics and Applications*, 4th edition.
Microcosm Press. ISBN: 978-1881883180.

Used for: Kepler's equation numerical solution (Ch. 2), ECEF/ECI coordinate transforms (Ch. 3).

---

*Last updated: 2025. To suggest a missing reference, open a GitHub issue.*
