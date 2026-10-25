using ROS2.Net.Navigation.Localization;
using ROS2.Net.Navigation.Maps;
using ROS2.Net.Navigation.Slam;
using Shouldly;

namespace ROS2.Net.Navigation.Tests;

public class LocalizationAndSlamTests
{
    [Fact]
    public void Amcl_converges_from_an_offset_initial_guess()
    {
        var map = TestWorlds.LoopCorridor();
        var rng = new GaussianRandom(7);
        var amcl = new Amcl(map, new AmclOptions { Seed = 11, MinParticles = 300, MaxParticles = 1500 });
        var truth = new Pose2D(1.5, 1.5, 0);
        amcl.Initialize(new Pose2D(1.9, 1.2, 0.15), 0.4, 0.2);

        // Drive along the bottom corridor; odometry has a small multiplicative error.
        var odom = truth;
        var truthAtUpdate = truth;
        var updates = 0;
        for (var i = 0; i < 60; i++)
        {
            truth = truth.Compose(new Pose2D(0.1, 0, 0));
            odom = odom.Compose(new Pose2D(0.1 * 1.03, 0, 0.002));
            if (amcl.Update(odom, TestWorlds.Scan(map, truth, 180, 10, 0.02, rng)))
            {
                truthAtUpdate = truth;
                updates++;
            }
        }

        // The estimate refers to the last filter update (AMCL only updates after update_min_d of travel).
        updates.ShouldBeGreaterThan(10);
        amcl.Estimate.DistanceTo(truthAtUpdate).ShouldBeLessThan(0.1);
        Math.Abs(Angles.Difference(amcl.Estimate.Theta, truthAtUpdate.Theta)).ShouldBeLessThan(0.05);
        amcl.ParticleCount.ShouldBeLessThanOrEqualTo(1500);
    }

    [Fact]
    public void Likelihood_field_measures_distance_to_walls()
    {
        var map = TestWorlds.Room(4, 4);
        var field = new LikelihoodField(map, 2.0);
        field.DistanceAt(2.0, 2.0).ShouldBe(1.9, 0.1);
        field.DistanceAt(0.5, 2.0).ShouldBe(0.45, 0.06);
    }

    [Fact]
    public void Ekf_fusing_gyro_beats_noisy_wheel_odometry()
    {
        var rng = new GaussianRandom(3);
        var ekf = new Ekf2D();
        ekf.Reset(Pose2D.Identity);
        var truth = Pose2D.Identity;
        var deadReckoning = Pose2D.Identity;
        const double dt = 0.02;
        for (var i = 0; i < 1500; i++)
        {
            var t = i * dt;
            var v = 0.5;
            var w = 0.3 * Math.Sin(t * 0.5);
            truth = new Pose2D(truth.X + (v * Math.Cos(truth.Theta) * dt), truth.Y + (v * Math.Sin(truth.Theta) * dt), truth.Theta + (w * dt));

            // Wheel yaw rate is biased and noisy (slipping); the gyro is clean.
            var wheelW = w + 0.05 + rng.Next(0.1);
            var wheelV = v + rng.Next(0.05);
            deadReckoning = new Pose2D(deadReckoning.X + (wheelV * Math.Cos(deadReckoning.Theta) * dt), deadReckoning.Y + (wheelV * Math.Sin(deadReckoning.Theta) * dt), deadReckoning.Theta + (wheelW * dt));
            ekf.UpdateTwist(t, wheelV, wheelW, 0.0025, 0.04);
            ekf.UpdateYawRate(t, w + rng.Next(0.002), 0.00001);
        }

        var ekfError = ekf.Pose.DistanceTo(truth);
        var drError = deadReckoning.DistanceTo(truth);
        ekfError.ShouldBeLessThan(drError / 3);
        Math.Abs(Angles.Difference(ekf.Pose.Theta, truth.Theta)).ShouldBeLessThan(0.05);
    }

    [Fact]
    public void Pose_graph_optimization_distributes_loop_error()
    {
        // Square of 4 nodes with drifted initial guesses and a loop closure back to the start.
        var graph = new PoseGraph();
        var truths = new[] { new Pose2D(0, 0, 0), new Pose2D(2, 0, Math.PI / 2), new Pose2D(2, 2, Math.PI), new Pose2D(0, 2, -Math.PI / 2) };
        graph.AddNode(truths[0]);
        graph.AddNode(new Pose2D(2.1, 0.1, Math.PI / 2 + 0.05));
        graph.AddNode(new Pose2D(2.3, 2.2, Math.PI + 0.1));
        graph.AddNode(new Pose2D(0.4, 2.3, -Math.PI / 2 + 0.15));
        for (var i = 0; i < 3; i++)
        {
            graph.AddEdge(new PoseGraphEdge(i, i + 1, truths[i].Between(truths[i + 1]), 100, 400, false));
        }

        graph.AddEdge(new PoseGraphEdge(3, 0, truths[3].Between(truths[0]), 100, 400, true));
        var before = graph.Chi2();
        var after = graph.Optimize();

        after.ShouldBeLessThan(before * 1e-3);
        for (var i = 0; i < 4; i++)
        {
            graph.Nodes[i].DistanceTo(truths[i]).ShouldBeLessThan(1e-3);
        }
    }

    [Fact]
    public void Scan_matcher_recovers_an_offset_pose()
    {
        var map = TestWorlds.LoopCorridor();
        var truth = new Pose2D(1.5, 6, 0.4);
        var reference = ScanPoints.FromScan(TestWorlds.Scan(map, truth, 360));
        var world = reference.Select(p => truth.TransformPoint(p.X, p.Y)).ToList();
        var grid = MatchGrid.Build(world, 0.05, 0.06);
        var result = new CorrelativeScanMatcher().Match(grid, reference, new Pose2D(1.75, 5.85, 0.3), 0.4, 0.3, 0.01);

        result.Pose.DistanceTo(truth).ShouldBeLessThan(0.05);
        Math.Abs(Angles.Difference(result.Pose.Theta, truth.Theta)).ShouldBeLessThan(0.02);
        result.Score.ShouldBeGreaterThan(0.8);
    }

    private static (Pose2D Truth, Pose2D Odom, (double X, double Y)[] Points)[] DriveLoop(OccupancyGridMap map, int seed)
    {
        var rng = new GaussianRandom(seed);
        var waypoints = new[] { (1.5, 1.5), (10.5, 1.5), (10.5, 10.5), (1.5, 10.5), (1.5, 1.5), (5.0, 1.5) };
        var samples = new List<(Pose2D, Pose2D, (double X, double Y)[])>();
        var truth = new Pose2D(1.5, 1.5, 0);
        var odom = truth;
        for (var w = 1; w < waypoints.Length; w++)
        {
            var (tx, ty) = waypoints[w];
            var heading = Math.Atan2(ty - truth.Y, tx - truth.X);

            // Turn in place, then drive.
            while (Math.Abs(Angles.Difference(heading, truth.Theta)) > 1e-6)
            {
                var dth = Math.Clamp(Angles.Difference(heading, truth.Theta), -0.15, 0.15);
                truth = truth.Compose(new Pose2D(0, 0, dth));
                odom = odom.Compose(new Pose2D(0, 0, (dth * 1.03) + rng.Next(0.003)));
                samples.Add((truth, odom, ScanPoints.FromScan(TestWorlds.Scan(map, truth, 180, 10, 0.01, rng), stride: 1)));
            }

            while (truth.DistanceTo(new Pose2D(tx, ty, 0)) > 1e-6)
            {
                var d = Math.Min(0.1, truth.DistanceTo(new Pose2D(tx, ty, 0)));
                truth = truth.Compose(new Pose2D(d, 0, 0));
                odom = odom.Compose(new Pose2D((d * 1.02) + rng.Next(0.003), 0, 0.003 + rng.Next(0.002)));
                samples.Add((truth, odom, ScanPoints.FromScan(TestWorlds.Scan(map, truth, 180, 10, 0.01, rng), stride: 1)));
            }
        }

        return samples.ToArray();
    }

    [Fact]
    public void Slam_builds_a_consistent_map_and_loop_closure_fixes_drift()
    {
        var world = TestWorlds.LoopCorridor();
        var drive = DriveLoop(world, 5);
        var slam = new OnlineSlam(new SlamOptions { BeamStride = 2 });
        foreach (var (_, odom, points) in drive)
        {
            slam.Process(odom, points.Where((_, i) => i % 2 == 0).ToArray());
        }

        var finalTruth = drive[^1].Truth;
        var odomError = Pose2D.Identity.DistanceTo(Pose2D.Identity);
        odomError = drive[0].Truth.Compose(drive[0].Odom.Inverse()).Compose(drive[^1].Odom).DistanceTo(finalTruth);
        var slamError = slam.Pose.DistanceTo(finalTruth);

        slam.LoopClosureCount.ShouldBeGreaterThan(0);
        odomError.ShouldBeGreaterThan(1.0);
        slamError.ShouldBeLessThan(0.3);

        // Most occupied cells of the built map lie on real walls.
        var map = slam.GetMap();
        var truthField = new LikelihoodField(world, 1.0);
        int occupied = 0, onWall = 0;
        for (var cy = 0; cy < map.Height; cy++)
        {
            for (var cx = 0; cx < map.Width; cx++)
            {
                if (map[cx, cy] == OccupancyGridMap.Occupied)
                {
                    occupied++;
                    var (wx, wy) = map.CellToWorld(cx, cy);
                    if (truthField.DistanceAt(wx, wy) < 0.2)
                    {
                        onWall++;
                    }
                }
            }
        }

        occupied.ShouldBeGreaterThan(500);
        ((double)onWall / occupied).ShouldBeGreaterThan(0.85);
    }
}
