# Footprints

The three SVG envelopes the benchmark campaigns handed to Bench-MR. They are kept
as they were: changing them would change the published results, so they are
described here rather than corrected.

`bot`, `bot_zoom` and `bot_zoom_zoom` are working names with no meaning beyond
their order. This is which campaign used which.

| file | campaigns | path in the file |
|---|---|---|
| `bot.svg` | `warehouse`, `procedural` | `M -0.969 0.7 H 7.298 v -1.4 H -0.969 Z` |
| `bot_zoom.svg` | `two_rooms`, `narrow_passage` | `M -3.876,1.4 H 29.188 V -4.2 H -3.876 Z` |
| `bot_zoom_zoom.svg` | `docking` | `M -7.752,2.8 H 58.376 V -8.4 H -7.752 Z` |

## What they contain

The same rectangle at three scales. The rear overhang is `0.4845` times the
scale in each: `0.969 = 2 × 0.4845`, `3.876 = 8 × 0.4845`, `7.752 = 16 × 0.4845`.

| file | length | width | ratio |
|---|---|---|---|
| `bot.svg` | 8.267 | 1.400 | 5.905 |
| `bot_zoom.svg` | 33.064 | 5.600 | 5.904 |
| `bot_zoom_zoom.svg` | 66.128 | 11.200 | 5.904 |

Two things follow from those numbers.

**The aspect ratio is not the vehicle's.** The forklift this package declares is
3.6490 m by 0.7000 m, ratio 5.213. The SVGs are 5.904, which is the ratio
of a rectangle running from `-0.4845` to `+3.6490`, that is the vehicle plus its
rear overhang counted twice. Bench-MR sizes the shape from its half width, so the
envelope that reaches the collision checker is 4.133 m by 0.700 m on the
navigation scenes and 3.306 m by 0.560 m on the docking one. Neither matches
`DEFAULT_VEHICLE`. The planner's own collision model, in
`shfm.environments.lift_to_configuration_space`, uses the declared vehicle.

**The zoomed variants are not centred on their reference point.** `bot.svg` runs
from `y = -0.7` to `y = +0.7`, centred. `bot_zoom.svg` runs from `-4.2` to
`+1.4`, so its centre sits at `y = -1.4`, and `bot_zoom_zoom.svg` from `-8.4` to
`+2.8`, centre `y = -2.8`. The offset is one scale unit of width in both cases.

## The declared vehicle

`../forklift.svg` is what `Vehicle.to_svg` writes: the declared vehicle, in
metres, origin at the centre of the rear axle, `viewBox="-0.4845 -0.35 3.649 0.7"`.
Use that one for new work. A planner given a reference point of `(0, 0)` and a
scale equal to the grid zoom places it correctly.
