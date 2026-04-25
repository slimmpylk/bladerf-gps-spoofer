# References & Citations

> Full annotated reference list for the bladerf-gps-spoofer project.
> Every equation and constant traces back to a primary source below.

---

## Primary Standards

### IS-GPS-200N
**Interface Specification IS-GPS-200N: GPS SPS Signal Specification**
U.S. Space Force. Latest revision: IS-GPS-200N (August 2022).

Key sections used in this project:
- §3.3 — L1 signal characteristics, C/A chip rate, BPSK modulation
- §20.3.3.4.3 — Satellite ECEF position algorithm from ephemeris
- §20.3.5.2 — Navigation message parity encoding (Table 20-XIV)
- Table 3-Ia — C/A code G2 phase assignments for PRN 1–32

**Direct PDF (USCG NAVCEN):**
https://www.navcen.uscg.gov/sites/default/files/pdf/gps/IS-GPS-200N.pdf

---

### WGS-84
**World Geodetic System 1984 — TR8350.2**
National Geospatial-Intelligence Agency.

Source for: μ = 3.986005×10¹⁴ m³/s², Ωₑ = 7.2921151467×10⁻⁵ rad/s

https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84

---

### RINEX 3.05
**The Receiver Independent Exchange Format v3.05**
IGS RINEX Working Group, 2021.

https://files.igs.org/pub/data/format/rinex305.pdf

---

## Textbooks

**Misra & Enge (2006)** — *Global Positioning System: Signals, Measurements and Performance*, 2nd ed.
Ganga-Jamuna Press. ISBN 978-0970954428.
GPS signal model, satellite position algorithm, pseudorange & Doppler derivation.

**Tsui (2000)** — *Fundamentals of GPS Receivers: A Software Approach*
Wiley. ISBN 978-0471385981.
C/A code generator circuit, Gold code correlation properties.

**Borre et al. (2007)** — *A Software-Defined GPS and Galileo Receiver*
Birkhäuser. ISBN 978-0817643904.
RINEX format, BPSK signal generation, software receiver reference.

**Proakis (2001)** — *Digital Communications*, 4th ed.
McGraw-Hill. ISBN 978-0072321111.
BPSK modulation theory, spread-spectrum, I/Q baseband.

---

## Open-Source Reference Implementations

**gps-sdr-sim** — Takuji Ebinuma. MIT License.
Primary reference implementation. Signal output validated against this before RF transmission.
https://github.com/osqzss/gps-sdr-sim

**GNSS-SDR** — Fernández-Prades et al. GPLv3.
Used as verification receiver for generated I/Q samples.
https://github.com/gnss-sdr/gnss-sdr

---

## Hardware

**bladeRF 2.0 micro — Nuand LLC**
- Wiki: https://github.com/Nuand/bladeRF/wiki
- libbladeRF API: https://nuand.com/libbladeRF-doc/

---

## Papers

**Gold (1967)** — Optimal binary sequences for spread spectrum multiplexing.
IEEE Trans. Information Theory, 13(4), 619–621.
https://doi.org/10.1109/TIT.1967.1054048

**Humphreys et al. (2008)** — Assessing the Spoofing Threat: Development of a Portable GPS Civilian Spoofer.
Proc. ION GNSS 2008.

**Psiaki & Humphreys (2016)** — GNSS Spoofing and Detection.
Proc. IEEE, 104(6), 1258–1270.
https://doi.org/10.1109/JPROC.2016.2526658
