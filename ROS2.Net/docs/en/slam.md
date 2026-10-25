# SLAM — building maps

`ROS2.Net.Navigation.Slam` is an online 2-D graph SLAM in the spirit of slam_toolbox and Karto. It is part of the `ROS2.Net.Navigation` package.

## How it works

1. **Prediction.** For each scan, the pose is predicted from the last keyframe plus the odometry delta.
2. **Scan-to-map matching.** Once the robot has moved `MinimumTravelDistance` or turned `MinimumTravelHeading`, the scan is matched against a rolling local map built from the last `ScanBufferSize` keyframes. This uses a multi-resolution correlative scan matcher (Olson 2009):
   - an exhaustive search over a coarse, max-pooled grid
   - a fine search on the full-resolution grid
   - bilinear sub-cell refinement
3. **Keyframes.** The corrected pose becomes a pose-graph node, linked to the previous node by an odometry/scan-match edge.
4. **Loop closure.** Keyframes at least `LoopMatchMinimumChainSize` old and within `LoopSearchMaximumDistance` are candidates. The scan is matched against the candidate's submap over a wide window (`LoopSearchSpaceDimension`, `LoopAngularWindow`). If the score exceeds `LoopMatchMinimumResponse`, a loop-closure edge is added.
5. **Optimization.** After each closure, `PoseGraph` runs Levenberg-Marquardt over SE(2), with the first node fixed.
6. **Map.** The occupancy grid is rebuilt lazily from all keyframes at their optimized poses, using log-odds ray casting.

```csharp
var slam = new OnlineSlam(new SlamOptions { Resolution = 0.05, MaxLaserRange = 12 });
foreach (var (odomPose, scan) in stream)
{
    slam.Process(odomPose, scan);
}

OccupancyGridMap map = slam.GetMap();
slam.SaveMap("maps/lab");          // lab.yaml + lab.pgm (map_server format)
Pose2D correction = slam.MapToOdom; // publish as map → odom
```

`SlamMode.Localization` matches against the existing graph without adding keyframes. The building blocks are public, so you can use them on their own:
- `CorrelativeScanMatcher` and `MatchGrid`
- `PoseGraph`, including `Chi2()` and `Optimize()`
- `ScanPoints.FromScan`

## ROS node

`SlamNode` (default name `slam_toolbox`):
- subscribes `/scan` (sensor-data QoS) and `/odom`
- publishes latched `/map` every `map_update_interval` seconds (parameter, default 2)
- publishes `map → odom` on `/tf` after every scan
- `SaveMap(basePath)` writes a map_server map

To navigate while mapping, feed `SlamNode.Slam.MapToOdom` into an `OdometryPoseProvider.Correction`.

## Limits compared to slam_toolbox

- The pose graph is solved with dense Cholesky. That is fine for hundreds of keyframes. For very large maps you would need a sparse solver; it is not included.
- Loop-closure candidates come from the closest old keyframe only, with no multi-candidate voting.
- There is no lifelong map pruning, no serialization of the pose graph, no interactive mode and no multi-robot merging.
- The map is rebuilt in full when it changes, not incrementally.

## Tests

The test suite drives a simulated robot once around a 12 × 12 m loop corridor with drifting odometry (about 2–3 % scale error and a yaw bias). It checks that:
- at least one loop closure fires
- the final SLAM pose error stays below 0.3 m, where odometry alone drifts more than 1 m
- more than 85 % of the occupied map cells lie on real walls

Separate tests check the scan matcher's recovery of an offset pose and the convergence of the pose-graph optimizer.
