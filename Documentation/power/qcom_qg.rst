.. SPDX-License-Identifier: GPL-2.0-only

Qualcomm QGauge estimation
=========================

The PM6150 QGauge driver can estimate charge from signed current FIFO and
accumulator samples when the monitored simple-battery supplies an OCV table
and a positive design charge capacity. Boards without that information keep
the legacy voltage estimate. The read-only ``fifo_estimator`` module parameter
can disable FIFO estimation. Charging limits are controlled by the separate
charger driver.

Sampling and calibration
------------------------

A ``fifo-done`` interrupt, when supplied by the board, is a wake source and
schedules a capture after the system has thawed. A 15-minute polling watchdog
backs up interrupt-driven capture. Without this interrupt, polling runs every
five minutes and cannot provide continuous coverage of long system sleeps.
Capture harvests real-time data before suspend and immediately after resume.
Missing coverage is recorded rather than extrapolated from the last current.

The initial terminal-voltage seed is provisional. A new hardware GOOD_OCV or
qualified full-charge indication supplies an anchor. After the first anchor,
OCV correction is bounded unless coverage has been lost. Hardware S3 can stop
FIFO sampling; a valid rested OCV recovers the estimate without pretending
that every sleeping coulomb was measured. Nominal capacity and the reference
OCV curves do not establish actual battery health or an aging profile.

Board-specific S3 qualification can use ``qcom,s3-entry-fifo-length`` (1..8),
``qcom,s3-entry-ibat-microamp`` and ``qcom,s3-exit-ibat-microamp`` (0..155550,
exit >= entry). All three must be supplied together. The current thresholds
are quantized in 610 microamp steps; the exit register contains the increase
above the quantized entry threshold. These properties qualify measurements,
not the permitted charging current.

Short restart state
-------------------

``qcom,persist-soc`` opts a board into the existing Android QG SDAM fields:
validity 0x46, rounded percent 0x47, temperature 0x48..0x49, voltage
0x4c..0x4f, discharge current 0x50..0x53 and PMIC RTC time 0x54..0x57.
Resistance, learned capacity, cycle counts and profile magic are untouched.
The private validity value 0x51 identifies this version of the saved format;
records left by Android are not restored. This marker must be changed if an
incompatible estimation/profile format is introduced.

Only anchored estimates without an active coverage gap are saved. Validity
is cleared before writing and committed last. Restore requires a percent in
0..100, a timestamp no more than 120 seconds old (not in the future), and a
temperature change no greater than 5 degrees Celsius. The PMIC RTC does not
need to match calendar/wall time. Shutdown harvests another sample and saves
state. Persistence errors do not prevent ordinary charge tracking.

A restored value remains uncertain until the next anchor because powered-off
charge was not measured. Long power-off falls back to a provisional voltage
seed. The saved percentage has one-percent resolution. This is short restart
continuity, not long-term coulomb-counter retention or capacity learning.
