# Shooter Model Inputs

This is the checklist for making the trajectory-based setpoint generator match the 2026 FRC game `REBUILT` and this robot's drum shooter as closely as possible.

## Public Game Numbers Found

- `Fuel diameter`: `5.91 in`
  Source: https://andymark.com/products/official-rebuilt-fuel?variant=45224853569708
- `Fuel weight range`: `0.448 - 0.5 lb`
  Source: https://andymark.com/products/official-rebuilt-fuel?variant=45224853569708
- `Fuel material`: `high-density foam`
  Source: https://andymark.com/products/official-rebuilt-fuel?variant=45224853569708
- `Hub top opening`: `41.7 in hexagonal opening`
  Source: https://firstfrc.blob.core.windows.net/frc2026/Manual/HTML/2026GameManual.htm
- `Hub front edge height`: `72 in off carpet`
  Source: https://firstfrc.blob.core.windows.net/frc2026/Manual/HTML/2026GameManual.htm
- `Hub location`: `158.6 in from alliance wall`
  Source: https://firstfrc.blob.core.windows.net/frc2026/Manual/HTML/2026GameManual.htm
- `Fuel compression variance exists and is legal`
  Source: https://firstfrc.blob.core.windows.net/frc2026/Manual/TeamUpdates/REBUILT_TeamUpdate03.pdf

## Public Numbers Not Found Reliably

- `Fuel durometer`
- `Fuel official compression spec`
- `Fuel drag coefficient`
- `Fuel lift / Magnus coefficient`
- `Fuel coefficient of restitution`
- `Batch-to-batch stiffness distribution`

## Robot Numbers Already Provided

- `Shooter type`: drum shooter
- `Drum diameter`: `4 in`
- `Drum radius`: `2 in = 0.0508 m`
- `Release height`: `15 in = 0.381 m`
- `Hood angle`: basically launch angle
- `Hood zero`: actually about `10 deg` above flat
- `Maximum hood angle`: `27.5 deg`
- `Compression`: approximately `1 in = 0.0254 m`
- `Current setpoint map works for getting fuel into the hub`
- Trusted anchor shots:
  - `9 ft`, `2650 RPM`, `12.5 deg`, medium-low arc
  - `10 ft`, `2700 RPM`, `13 deg`, medium-low arc

## Important Modeling Note

- The current robot code uses distance to the `hub center`.
- A scored shot does not need to travel to the `hub center`; it only needs to enter the opening.
- Because of that, `target_x_offset_m` in the generator matters a lot for accuracy.
- If we model the target as the front edge of the opening, the horizontal target point should be closer than the hub center.
- With `release height = 15 in` and `launch angle = hood + 10 deg`, the trusted `9-10 ft` shots are not reachable with a pure no-lift parabola to the HUB opening height.
- That means the real system almost certainly needs at least one of these modeled:
  - more true launch angle than the current estimate
  - meaningful backspin lift
  - a different exact target point in the opening than assumed by the simple model

## Highest Priority Robot Numbers Still Needed

- `Distance reference point`
  Is the current map tuned against distance to hub center, front edge, or just whatever `SnapCommands.distanceToHub()` reports?
- `Best target point in the opening`
  Do you want to model "just clear the front lip", "center of opening", or "preferred make window"?
- `Hood zero offset`
  Even if hood angle is basically launch angle, is `0 deg` truly flat, or is there a few degrees of built-in shooter tilt?
- `Effective ball speed ratio`
  How close is actual exit speed to drum surface speed?
- `Backspin / lift behavior`
  Enough to matter, based on the trusted 9-10 ft anchor shots.
- `Measured anchor shots`
  At least 3 known-good points with distance, hood angle, shooter RPM, and whether the shot is flat or arc-y.
- `Maximum usable hood angle`
  Mechanical max and practical max are both useful.

## Medium Priority Robot Numbers

- `Drum width`
- `Ball contact wrap angle`
- `Drum surface material`
- `Estimated backspin amount`
- `Shot-to-shot RPM sag under sustained fire`
- `Whether practice fuel matches event fuel`

## Best Next Measurements

- Record slow-motion video for `3` known-good shots.
- Measure real launch angle from video instead of encoder units.
- Measure real time of flight.
- Note whether the ball enters near the front edge or deeper into the opening.
- Compare KOP fuel against your current practice set.
