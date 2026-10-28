# Recorded demos

The supplied recordings show physical robot runs and PC-side mapping and exploration displays. The previews below play at normal speed. Full robot recordings are available as silent MP4 downloads; GitHub may offer a download instead of an inline player.

## Physical robot

| Run 1 | Run 2 |
| --- | --- |
| ![Physical robot moving around indoor furniture, run 1](run-1-preview.gif) | ![Physical robot moving around indoor furniture, run 2](run-2-preview.gif) |
| [Watch on Drive](https://drive.google.com/file/d/1N3Wnz_RERe_v0gNNwyVqRQkU6eJ4O28J/view?usp=drivesdk) · [Download silent MP4 — 40 seconds, 3.21 MB](https://raw.githubusercontent.com/ADHU-ADHU/roverlink-ros2/main/docs/demos/run-1-live.mp4) | [Watch on Drive](https://drive.google.com/file/d/1L8eBylxhtYKRo3VYYARCjCyfaFWwbkDD/view?usp=drivesdk) · [Download silent MP4 — 35 seconds, 2.45 MB](https://raw.githubusercontent.com/ADHU-ADHU/roverlink-ros2/main/docs/demos/run-2-live.mp4) |

These clips show the physical platform moving around furniture. The camera footage alone does not identify the active controller or distinguish teleoperation from autonomous motion. The GIFs show source seconds 5–15 of Run 1 and 7–17 of Run 2. The MP4s retain the full supplied video duration and framing; audio has been removed.

## Frontier planning display

![Recorded PC frontier clusters, robot pose and selected goal](frontier-planning-preview.gif)

An 18-second crop of the recorded frontier plot, from source seconds 14–32. Startup commands in the source recording reference `frontier_explorer55.py`, `a*_smooth5.py` and `path_follower5.py`. This supports the recorded components; it does not establish the exact source revision used in every robot run.

## SLAM in RViz2

![Recorded occupancy-map display in RViz2](slam-rviz-preview.gif)

An 18-second excerpt of the PC-side RViz2 map display, from source seconds 86–104. The source recording also shows robot-side LiDAR, IMU and motor-controller startup commands, alongside PC-side SLAM Toolbox and RViz2 commands.

## Media provenance

- [Original recording manifest](original-media-manifest.json): source hashes, sizes, durations and stream metadata. Each of the seven unique recordings had a byte-identical nested duplicate in the supplied folder.
- [Prepared media manifest](prepared-media-manifest.json): source hashes, excerpt windows, crops, encoding settings and output hashes.

The Drive links point to the full original recordings. Both had “Anyone with the link — Viewer” access when checked. The local original recordings remain unchanged. The published MP4 copies use H.264 with browser-compatible pixel format and faststart metadata; both were decoded in full without errors. GIFs use reduced dimensions, frame rates and colour palettes for a smaller README preview. No playback speed changes or synthetic imagery were added.
