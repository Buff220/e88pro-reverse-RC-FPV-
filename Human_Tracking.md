# Person Tracking Logic

This document explains how the autopilot decides where to steer: follow, prediction, and search. It does not cover the drone's control protocol.

## Overview

Each video frame goes through one pipeline:

```
frame -> person detector (YOLO) -> target selection -> state logic -> steering -> stick output
```

Only the front camera (CAM 1) can drive follow mode. Steering only touches yaw (turning left and right), and optionally altitude. Forward motion is pushed only when the person is centered.

The autopilot runs in a loop on each new frame. It has no memory of the drone's position in the room. It only remembers recent sightings of the person in the image.

## 1. Detection and target selection

- YOLO runs on every new frame and keeps only the `person` class.
- If several people are visible, the **largest bounding box** is the target. Size is used as a proxy for "closest".
- Each detection is stored as a sighting: `(time, center_x, center_y, height)`, with coordinates normalized to 0..1 of the frame.

## 2. Frame health (safety gate)

The autopilot never steers on a picture it cannot trust.

| Check | Setting | Result when it fails |
|---|---|---|
| No new frame for too long | `FRAME_STALE` (0.4 s) | Hover: all sticks neutral |
| Incoming video is too slow | `FRAME_MIN_FPS` (3 fps) | Hover |
| Video is slow but alive | `YAW_SCALE_MIN` (0.35) | Turns and forward push are scaled down |

Video frame rate also scales yaw: at 15 fps or more, full gain; below that, proportionally gentler.

## 3. Follow (live tracking)

When the person is seen, the follower steers the box center toward the middle of the image.

**Error:**
- `dx` = horizontal distance from the box center to the image center (pixels)
- `R` = follow circle radius = `radius` (default 12%) × the shorter side of the frame
- `dist` = `|dx|`, or the full 2D distance if altitude alignment is on

**Control:**
- Proportional control: `yaw = KP_YAW × error`, clipped to `MAX_YAW`.
- Deadzone: if the error is inside `0.4 × R`, no turn is commanded, so the drone doesn't jitter.
- Minimum command: once a correction is needed, it is at least `MIN_CMD` (0.10), so small errors still produce movement.

**States while following:**

| State | Meaning | Action |
|---|---|---|
| `ALIGN` | Person is off-center | Turn toward them, no forward push |
| `FORWARD` | Person is inside the circle | Turn gently and push forward (`FWD` × speed gear) |
| `ARRIVED` | Person's box height ≥ `stop` (default 60% of frame) | Keep centering, stop pushing forward |

**Hysteresis:** `FORWARD` drops back to `ALIGN` only when the person leaves a circle `RELEASE` (1.6) times larger. This stops the drone flipping between states on the boundary.

**Confirmation:** after the person is lost, they must be detected in `DETECT_CONFIRM` (2) consecutive frames before steering resumes. This ignores one-frame false positives.

## 4. Prediction (short-term memory)

Used when the person disappears behind something or leaves the frame briefly. Enabled with the PREDICT button.

**Estimating motion:**
- Keep the sightings from the last `PRED_WINDOW` (0.8 s).
- Velocity is the change in position from the oldest to the newest sighting, divided by the time between them. It needs at least 0.15 s of data.

**Extrapolating:**
- Predicted position = last seen position + velocity × min(time since last seen, `PRED_LEAD` 1.0 s).
- The point is clamped to the frame edges.

**Steering on the prediction:**
- Same error logic as follow, but gains are reduced by `PRED_GAIN` (0.6) so the drone turns more gently.
- **No forward push.** The drone turns toward the guess but never flies at it.

**Expiry:**
- Prediction stops `PRED_MAX_AGE` (1.5 s) after the last sighting. After that, the drone moves to search or hover.

## 5. Search (looking for the person)

Used when the person is gone and prediction has expired. Enabled with the SEARCH button. If SEARCH is off, the drone hovers.

**Direction** is chosen once, when search starts:
1. If the person was moving clearly sideways in the image (speed ≥ `SEARCH_MOVE_MIN`), search goes the way they were moving. A leftward move means search left.
2. Otherwise, if the person was last seen off-center, search goes toward that side.
3. Otherwise, search goes left.

The direction is kept for the whole search, so the drone doesn't zigzag.

**Sweep pattern:**
- Turn at `SEARCH_YAW` (0.10) for `SEARCH_TURN` (0.35 s).
- Stop for `SEARCH_PAUSE` (1.0 s) so the camera can capture still frames.
- Repeat until the person is confirmed.

The turns are small on purpose. A fast turn blurs the image and the detector misses people.

## 6. State flow

```
person visible --------------------------> ALIGN / FORWARD / ARRIVED
     |                                            ^
     | lost                                       | confirmed (DETECT_CONFIRM)
     v                                            |
prediction on? --yes--> PREDICT (up to 1.5 s) ----+
     | no / expired
     v
search on? --yes--> SEARCH (turn, pause, repeat) -+
     | no
     v
LOST - HOVER
```

Any frame-health failure overrides everything and puts the drone in `NO VIDEO - HOVER`.

## 7. Safety layers (outside the tracker)

- **Dead-man:** if the browser stops sending stick messages for `STALE` (0.8 s), all sticks go neutral.
- **Takeover:** any manual stick input above `TAKEOVER` (0.15) pauses the autopilot immediately.
- **Slew limit:** stick changes are capped at `SLEW_RATE` per update, so the drone doesn't jerk.
- **Emergency land:** SPACE or the EMERGENCY LAND button turns the autopilot off and lands.

## 8. Tuning

| Symptom | Setting to change |
|---|---|
| Overshoots the person while turning | Lower `KP_YAW` |
| Turns too slowly to catch the person | Raise `KP_YAW` or `MAX_YAW` |
| Jitters left and right when the person is nearly centered | Raise the deadzone (`0.4` in `follow` / `predict_steer`) |
| Drone walks forward too early or too late | Change `radius` (the circle) |
| Stops too far or too close | Change `stop` |
| Flips between ALIGN and FORWARD | Raise `RELEASE` |
| Prediction overshoots past where the person went | Lower `PRED_LEAD` to 0.5 |
| Prediction gives up when the person ducks out | Raise `PRED_MAX_AGE` (expect more blind turning) |
| Predicted turns feel sluggish | Raise `PRED_GAIN` to 0.8 |
| Search is too slow to cover the room | Raise `SEARCH_YAW` to 0.15 (frames smear above that) |
| Search turns the wrong way | Flip the sign in `last_direction()` |
| Hovers too eagerly on slow Wi-Fi | Raise `FRAME_STALE` to 0.8 |

## 9. Known limitations

- **No obstacle avoidance.** Tracking only sees the person, not walls or furniture.
- **No depth.** Distance is estimated from box height, which changes with the person's posture and the camera angle.
- **Largest box wins.** With several people, the drone can switch targets when a bigger person enters the frame.
- **Direction assumptions.** Search and prediction assume that a person moving left in the image is moving left in the room. This holds when the drone faces them.
- **Motion is extrapolated linearly.** Anyone who stops or turns sharply will break the prediction.

Test with propellers removed first.
