---
name: Earbud test report
about: Report how a pair of LE Audio earbuds works with this project
labels: earbud-report
---

**Earbuds**: (brand, model, earbud firmware version)
**Board**: (M5Stack Tab5 / other P4 + C6 board)
**Firmware**: P4 build id and C6 version (`/api/status` -> bt.coprocessor_fw)

- [ ] Both earbuds connect (stereo, 2 CIS)
- [ ] Audio plays without dropouts for 5 minutes
- [ ] Earbuds reconnect automatically after taking them out of the case

**Measurement** (optional, see docs/measurements.md): SDU air loss per CIS, dropouts, environment (home / crowded).

**Log excerpt** around any problem (remove addresses and keys):
