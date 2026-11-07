"""Measured constants shared by `flow_node` and its mixins.

Each one is in `measured-bars.md` with the measurement behind it.

Kept out of `flow_node.py` so the mixins can import them without importing
the node module itself.
"""
import cv2


# Window 31, measured. On this camera over a real ~16-frame baseline, with
# 172 candidate corners:
#
#     window   fb<=1px   RANSAC inliers   residual   ms
#       15       128          127          0.54     3.7
#       21       146          140          0.40     5.0
#       31       153          150          0.40     6.8
#       41       164          165          0.29     8.8
#
# A bigger window carries more texture, which is what makes a match unique on
# a blurry or low-contrast floor -- and blur is what we measured this camera to
# have (sharpness 70 against 321 and 1180 on archived competition clips). 41
# is the best of the four and 31 is taken instead only because LK runs on
# every arriving frame for the ripeness check: at ~90 fps that is 0.6 of a core
# at 31 and 0.8 at 41. Revisit if the frame path ever gets cheaper.
#
# maxLevel 4 measured IDENTICAL to 3 on every window, so it stays at 3.
_LK_PARAMS = dict(winSize=(31, 31), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                            30, 0.01))


# The FOCAL LENGTHS ARE MEASURED, and there are two of them because a flat port
# is a lens. Round 25: 63.8 deg in air / 46.7 deg in water, +-0.7, 25 views,
# calibrateCameraRO, held-out validated with an external tape check. At 640 px
# that is 514 and 741 -- a ratio of 1.44, where the flat-port literature says
# 25-33 %. The discrepancy is expected (port thickness and geometry) and the
# MEASUREMENT wins; the literature only explains why the two differ.
#
# Using the air number underwater is the single most dangerous mistake
# available here: a clean 44 % multiplier on every velocity, invisible in every
# plot, integrated by everything downstream. So `medium` is an explicit
# parameter with no safe default -- it is printed at startup.
_F_AIR_PX = 513.94


_F_WATER_PX = 741.0


# The de-rotation gains default to the canonical -1.0, deliberately.
#
# The bench rig measured -1.150 (dx<-gx) and -1.095 (dy<-gy), cond(MtM) 27.7,
# and cut false velocity under pure rotation by 90 %. But rotation about the
# lens gives EXACTLY f*omega*dt, so a 12 % excess needs an owner and does not
# have one: a pivot lever arm was tried and its sign came out WRONG, an f of
# 577 contradicts a stronger calibration, and a dt bias would produce the same
# number. measured-bars.md section 12 says it plainly -- "the axis mapping and
# the method carry over; THIS GAIN DOES NOT, whatever its cause. Re-derive it
# on the hull."
#
# The sign was wrong, and -1.0 was the worst available choice. 2026-09-07.
#
# Re-derived on the hull, which is what the §12 note above asked for. Two
# independent measurements, neither of which alone would have been enough:
#
#  1. ON HARDWARE, from tilt-only motion, one axis per run, fitted through the
#     origin with k-fold held-out scoring: the coupling is S = -1.058 (roll)
#     and -0.830 (pitch). Held-out rms 4.23 px against 14.42 for predicting
#     zero, R2 0.913, fold sd 0.008.
#  2. Offline and exact, driving this node with a synthetic rotation whose
#     ground truth is chosen rather than measured, the residual is linear in
#     (S + g) and vanishes at g = -S:
#
#         true S = -1        g -1: 96.13 cm   g 0: 48.13 cm   g +1: 0.13 cm
#         true S = +1        g -1:  0.18 cm   g 0: 48.18 cm   g +1: 96.18 cm
#
# So g = -S, and with S = -1.058 the shipped -1.0 was not merely wrong, it
# DOUBLED the error against applying no correction at all. That is the
# "a bad correction is worse than none" rule biting the default itself.
#
# And then the validation a/B refuted the fitted gains. Read this before
# "fixing" the default back to the fitted numbers.
#
# Shipping 1.058/0.830 was the obvious next step and it is WRONG. Three
# slides, three gains, every arm on the same slide:
#
#     A  (-1.000, -1.000)   median 122.3 %   |err| 22.3 %
#     B  (+1.058, +0.830)   median  75.5 %   |err| 24.5 %   <- the fitted pair
#     C  ( 0.000,  0.000)   median  90.1 %   |err|  9.9 %   <- best
#
# The fitted gains OVER-correct: they subtract too much and the distance
# under-reads. A over-reads by as much in the other direction. The zero
# crossing is near g = 0.
#
# The reconciliation is physical and §12 already named it: the effective gain
# depends on WHERE THE PIVOT IS -- `f * (1 - r/h)` -- so a gain fitted while
# tilting about the lens does not transfer to a slide pivoting about the
# operator's shoulder, and no single constant serves both. That is also the
# most likely reason §12's -1.150 and this round's -1.058 coupling cannot be
# reconciled: they were excited with different pivots.
#
# So the default is ZERO -- de-rotation OFF -- and that is a measured choice,
# not a surrender:
#   * it is the best of the three on the actual task (9.9 % vs 22.3 / 24.5);
#   * it is the only value that CANNOT double the error, since the residual
#     is (S + g) and g = 0 leaves exactly the uncorrected term;
#   * -1.0 is measurably the WORST available value on this hull, so leaving
#     it would have been the one indefensible option.
#
# This is not "de-rotation does not work". §12 measured it HALVING the
# error at 0.638 rad/s, and that regime -- a yawing vehicle -- is most of a
# mission, while every slide here is translation-dominated. Re-enabling it
# needs a HIGH-ROTATION test on this hull, with the gain excited at the pivot
# the vehicle actually rotates about. BUGS.md §11 item 24.
# What the fit's own error cannot see (issue #23). The published variance
# was the standard error of the mean of the fit residual alone -- ~1 mm/s --
# while the measured error of this sensor is centimetres. Two terms are added,
# each from a measurement in measured-bars.md, not a guess:
#   SCALE: worst of three taped 30 cm slides read 31.09 cm, 3.6 % (§5 of the
#     downward-camera results; an upper bound -- the tape and hand are in it).
#   ROTATION: roll/pitch rate makes h * omega of false velocity. With a
#     de-rotation gain of 0 (the shipped default) all of it remains; a
#     calibrated gain removed 90 % (575.7 -> 57.1 mm/s, §12), so 10 % remains.
FLOW_SCALE_SIGMA_FRAC = 0.036


# The camera<->gyro offset the online estimator STARTS from. Measured on the
# vehicle's own downward camera and SROT board, three 60 s hand-wobble runs:
# -11.3 ms, sd 0.84 ms (measured-bars, 2026-10-03). It was 0.0, so every run
# began 11 ms wrong and stayed so until enough rotation arrived for a fit --
# and on a level transit that may be never. Negative: the board's ATTITUDE
# rates are stamped AFTER the motion they describe. The estimator still runs
# and slews from here.
FLOW_TD_MEASURED_S = -0.0113


FLOW_ROT_RESIDUAL_CALIBRATED = 0.10


_GYRO_GAIN_X_DEFAULT = 0.0


_GYRO_GAIN_Y_DEFAULT = 0.0


# The width the `focal_air_px` / `focal_water_px` parameters are stated for --
# the 640x360 profile the camera has always streamed. Used ONLY to scale them
# when no calibration file is given (issue #32).
FOCAL_PARAMS_WIDTH = 640.0


# The values measured on this mount, kept so they are not re-derived: the
# coupling is S = -1.058 (roll) / -0.830 (pitch) about the LENS, and the node
# cancels at g = -S (pinned by TestDeRotationSignConvention, exactly, offline).
_GYRO_GAIN_LENS_PIVOT = (1.058, 0.830)
