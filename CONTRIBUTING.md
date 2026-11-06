# Contributing

Thanks for helping! Most useful right now:

1. **Earbud test reports**: use the "Earbud test report" issue template. Include the model, firmware version of the
   earbuds, whether stereo (2 CIS) works, and a measurement window (see docs/measurements.md).
2. **Bug reports**: the C6 and P4 versions, the device log around the problem, and steps to reproduce.
3. **Pull requests**: keep changes focused; for the CIS engine add or extend a host test (`cis_ll/test`) and run
   `sh c6_firmware/build.sh` (it runs the host tests, builds, and verifies the controller hooks in the linked ELF).

Radio changes on the C6 can make a device unreachable over SDIO. Never publish a C6 image that did not reach
`RESULT: PASS`, and keep the `fw_guard` rollback intact (docs/flashing-and-recovery.md).
