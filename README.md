# Cricket Radar: phone-camera speed gun, tuned for the Huawei Y9 Prime (2019)

Pocket Radar measures speed with a Doppler radar chip. A phone has no radar, so
Cricket Radar measures speed with the camera instead. It tracks the ball through
the video, then fits its flight with air drag and gravity. The result is the
**release speed in km/h**, the figure broadcasts show. It uses the Laws of
Cricket pitch length (20.12 m) and stump height (71.1 cm).

## Two ways to use it

| Mode | How | Accuracy (tests on rendered clips) |
|---|---|---|
| **Measure a delivery** (best) | Film with the Y9 Prime camera app in **Slow-mo**, then pick the clip in the app | within about 3 km/h |
| **Live radar** | Point the phone at the pitch; a speed pops up after every ball | within about 1–2 km/h on clean video at 30 fps; real conditions will be rougher |

Live radar needs camera access. The published Claude page can't open the
camera, so for live mode open `cricket-radar-standalone.html` from a web
address you control (below).

## How it is tuned for the Y9 Prime

| Y9 Prime fact | What the app does with it |
|---|---|
| Main camera 16 MP f/1.8; video 1080p at 30 fps | Expects 30 fps in live mode and checks the real rate from frame timestamps |
| No electronic stabilisation setting | Treats the picture as an unwarped pinhole camera |
| Main lens field of view ≈ 65° across the video (estimated, not published by Huawei) | Uses it as the starting lens model. Your stump taps refine it. A 10 % error changes speed by about 0.2 km/h |
| Budget CMOS sensor with a rolling shutter (rows read one after another, ≈ 25 ms per frame, estimated) | Corrects each ball position for the moment its row was read. Without this, 30 fps readings were about 1 km/h off |
| Slow-mo mode saves clips that play back slowly at 30 fps, and Huawei doesn't publish the capture rate | Works out the slow-mo factor (4× or 8×) from how fast the ball falls, which has to be 9.81 m/s². It reports the factor it found, e.g. "4× slow-mo, confirmed by the ball's fall" |
| 0.6× ultra-wide lens (13 mm, ≈ 120°) | Not supported: it bends straight lines. The guide says to stay on 1× |

To use another phone, choose *Other phone* in Settings and enter its field of view.

## Filming

1. Tripod, side-on, 15–25 m from the pitch, level with its middle, both sets of stumps in the picture, landscape.
2. Camera › More › Slow-mo. Use the fastest speed offered. Stay on the 1× lens. Don't zoom.
3. In the app, tap **Measure a delivery**, pick the clip and mark the two middle stumps: base and top at the bowler's end, then the batter's end. Press and slide to fine-tune each point in the magnifier. The app remembers the marks for the next clip from the same spot.
4. Say which side of the stumps the ball left the hand, as seen from the phone. This removes 1–2 % error.

## Hosting the standalone file (for live radar)

Browsers only give camera access to pages served over `https://`.

1. Create a free GitHub account, then a public repository, e.g. `cricket-radar`.
2. Upload `cricket-radar-standalone.html` and rename it `index.html`.
3. Settings › Pages › Deploy from branch › `main` › Save. After a minute it is live at `https://<your-name>.github.io/cricket-radar/`.
4. Open that address in Chrome on the Y9 Prime. Use ⋮ › *Add to Home screen* for an app icon. Allow the camera when asked.

## Files

| File | What it is |
|---|---|
| `cricket-radar-standalone.html` | The complete app in one file (engine and sample clip included) |
| `engine.js` | Detection, tracking and physics engine (plain JavaScript, also runs in Node) |
| `app.html`, `build.py` | App source and the script that builds the single-file app |
| `test_engine.js` | Runs the engine on a video file with ffmpeg; used for the accuracy tests |
| `clips/` | Rendered Y9-style test deliveries with their true speeds (`.json`) |

## Test results (rendered video with known speed)

These clips copy the Y9 Prime's 65° lens and rolling shutter. Each also has
random tap errors of about ±1 px on the stumps.

| Clip | True | Measured | Slow-mo found |
|---|---|---|---|
| 1080p, 30 fps | 130 | 129.2 | 1× (normal) |
| 1080p, 30 fps | 145 | 145.8 | 1× |
| 4× slow-mo saved at 30 fps, 720p | 140 | 136.7 | 4× |
| 4× slow-mo, spin pace | 95 | 93.9 | 4× |
| 8× slow-mo | 150 | 151.6 | 8× |
| 4× slow-mo, ball released on the far side | 120 | 119.6 | 4× |
| Sample in the app, measured in a browser | 140 | 138.9 | 4× |
| Live radar on a looping 145 km/h clip (9 balls) | 145 | 144.5–145.0 | – |

Real footage will be harder than rendered clips, with blur, clutter and
changing light. Before trusting the last km/h, film 20–30 balls alongside a
radar gun and compare.

## Limits

- The ball's sideways line can't be seen side-on, so the release side you choose matters (about 1 % per 0.2 m).
- If Huawei's slow-mo ramps speed inside one clip (normal → slow → normal), trim the clip to the slow part first.
- Very cluttered backgrounds (nets, people walking behind the pitch) can hide the ball. Raise or lower detection sensitivity in Settings.
