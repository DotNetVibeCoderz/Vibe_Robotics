using ROS2.Net.Navigation.Costmaps;
using ROS2.Net.Navigation.Maps;
using ROS2.Net.Navigation.Planning;
using Shouldly;

namespace ROS2.Net.Navigation.Tests;

public class MapAndPlannerTests
{
    private static (OccupancyGridMap Map, Costmap2D Costmap) WallWithGap(bool gap = true)
    {
        var map = TestWorlds.Room(10, 6);
        map.FillRect(5, 0, 5.2, 6, OccupancyGridMap.Occupied);
        if (gap)
        {
            map.FillRect(5, 4.5, 5.2, 5.5, OccupancyGridMap.Free);
        }

        var costmap = Costmap2D.FromMap(map)
            .AddLayer(new StaticLayer(map))
            .AddLayer(new InflationLayer(0.4, 3.0, 0.2));
        costmap.Update();
        return (map, costmap);
    }

    [Fact]
    public void World_cell_and_message_conversions_round_trip()
    {
        var map = new OccupancyGridMap(20, 10, 0.1, new Pose2D(-1, -0.5, 0));
        map.WorldToCell(0.05, 0.05, out var cx, out var cy).ShouldBeTrue();
        (cx, cy).ShouldBe((10, 5));
        map.CellToWorld(10, 5).X.ShouldBe(0.05, 1e-9);
        map.WorldToCell(5, 5, out _, out _).ShouldBeFalse();

        map[3, 4] = 100;
        var back = OccupancyGridMap.FromMsg(map.ToMsg());
        back[3, 4].ShouldBe((sbyte)100);
        back.Origin.X.ShouldBe(-1, 1e-6);
        back.Resolution.ShouldBe(0.1, 1e-6);
    }

    [Fact]
    public void Map_server_files_round_trip()
    {
        var map = TestWorlds.Room(3, 2);
        map.FillRect(1, 1, 1.5, 1.5, OccupancyGridMap.Unknown);
        var dir = Path.Combine(Path.GetTempPath(), "ros2net-map-" + Guid.NewGuid().ToString("N"));
        try
        {
            map.SaveMapServer(Path.Combine(dir, "room"));
            File.Exists(Path.Combine(dir, "room.pgm")).ShouldBeTrue();
            var loaded = OccupancyGridMap.LoadMapServer(Path.Combine(dir, "room.yaml"));
            loaded.Width.ShouldBe(map.Width);
            loaded.Height.ShouldBe(map.Height);
            loaded.Resolution.ShouldBe(0.05, 1e-9);
            for (var i = 0; i < map.Data.Length; i++)
            {
                loaded.Data[i].ShouldBe(map.Data[i]);
            }
        }
        finally
        {
            Directory.Delete(dir, true);
        }
    }

    [Fact]
    public void Raycast_hits_the_wall_at_the_cell_edge()
    {
        var map = TestWorlds.Room(5, 5);
        map.Raycast(2.5, 2.5, 0, 10).ShouldBe(2.45, 0.051);
        map.Raycast(2.5, 2.5, Math.PI / 2, 1.0).ShouldBe(1.0);
    }

    [Fact]
    public void Inflation_follows_the_nav2_cost_function()
    {
        var map = new OccupancyGridMap(40, 40, 0.05, default, OccupancyGridMap.Free);
        map[20, 20] = OccupancyGridMap.Occupied;
        var layer = new InflationLayer(0.5, 3.0, 0.2);
        var costmap = Costmap2D.FromMap(map).AddLayer(new StaticLayer(map)).AddLayer(layer);
        costmap.Update();

        costmap.GetCost(20, 20).ShouldBe(Costs.Lethal);
        costmap.GetCost(21, 20).ShouldBe(Costs.Inscribed);
        costmap.GetCost(24, 20).ShouldBe(Costs.Inscribed); // 0.2 m == inscribed radius
        var d = 8 * 0.05;
        costmap.GetCost(28, 20).ShouldBe((byte)(252 * Math.Exp(-3.0 * (d - 0.2))));
        costmap.GetCost(39, 20).ShouldBe(Costs.Free);
        layer.ComputeCost(0.6).ShouldBe(Costs.Free);
    }

    public static TheoryData<string> GridPlanners => ["AStar", "Dijkstra", "ThetaStar"];

    private static IGlobalPlanner Create(string name) => name switch
    {
        "AStar" => new AStarPlanner(),
        "Dijkstra" => new DijkstraPlanner(),
        _ => new ThetaStarPlanner(),
    };

    [Theory]
    [MemberData(nameof(GridPlanners))]
    public void Grid_planners_find_collision_free_paths_through_the_gap(string name)
    {
        var (_, costmap) = WallWithGap();
        var result = Create(name).CreatePlan(costmap, new Pose2D(1, 1, 0), new Pose2D(9, 1, 0));

        result.Success.ShouldBeTrue(result.Error);
        PathUtils.IsCollisionFree(costmap, Footprint.Circle(0.2), result.Poses).ShouldBeTrue();
        result.Poses.ShouldContain(p => p.X > 4.9 && p.X < 5.3 && p.Y > 4.4); // went through the gap
        result.Poses[^1].X.ShouldBe(9, 1e-9);
    }

    [Theory]
    [MemberData(nameof(GridPlanners))]
    public void Grid_planners_fail_cleanly_when_blocked(string name)
    {
        var (_, costmap) = WallWithGap(gap: false);
        var result = Create(name).CreatePlan(costmap, new Pose2D(1, 1, 0), new Pose2D(9, 1, 0));
        result.Success.ShouldBeFalse();
        result.Error.ShouldBe("no path found");
    }

    [Fact]
    public void Theta_star_is_not_longer_than_a_star_and_a_star_expands_less_than_dijkstra()
    {
        var (_, costmap) = WallWithGap();
        var astar = new AStarPlanner(new GridPlannerOptions { CostWeight = 0 });
        var dijkstra = new DijkstraPlanner(new GridPlannerOptions { CostWeight = 0 });
        var theta = new ThetaStarPlanner(new GridPlannerOptions { CostWeight = 0 });
        var a = astar.CreatePlan(costmap, new Pose2D(1, 1, 0), new Pose2D(9, 1, 0));
        var d = dijkstra.CreatePlan(costmap, new Pose2D(1, 1, 0), new Pose2D(9, 1, 0));
        var t = theta.CreatePlan(costmap, new Pose2D(1, 1, 0), new Pose2D(9, 1, 0));

        t.Length.ShouldBeLessThanOrEqualTo(a.Length + 1e-6);
        a.Length.ShouldBe(d.Length, 0.05);
        astar.LastExpansions.ShouldBeLessThan(dijkstra.LastExpansions);
    }

    [Fact]
    public void Goal_inside_an_obstacle_snaps_to_nearby_free_space_or_fails()
    {
        var (_, costmap) = WallWithGap();
        var planner = new AStarPlanner();
        planner.CreatePlan(costmap, new Pose2D(1, 1, 0), new Pose2D(5.1, 2, 0)).Success.ShouldBeTrue();
        new AStarPlanner(new GridPlannerOptions { GoalTolerance = 0 }).CreatePlan(costmap, new Pose2D(1, 1, 0), new Pose2D(5.1, 2, 0)).Success.ShouldBeFalse();
        planner.CreatePlan(costmap, new Pose2D(1, 1, 0), new Pose2D(50, 2, 0)).Error.ShouldBe("goal is outside the costmap");
    }

    [Fact]
    public void Hybrid_a_star_respects_the_turning_radius_and_obstacles()
    {
        var (_, costmap) = WallWithGap();
        var planner = new HybridAStarPlanner(Footprint.Circle(0.2), new HybridAStarOptions { MinTurningRadius = 0.6 });
        var result = planner.CreatePlan(costmap, new Pose2D(1, 1, 0), new Pose2D(8, 1.5, -Math.PI / 2));

        result.Success.ShouldBeTrue(result.Error);
        PathUtils.IsCollisionFree(costmap, Footprint.Circle(0.2), result.Poses).ShouldBeTrue();
        result.Poses[^1].DistanceTo(new Pose2D(8, 1.5, 0)).ShouldBeLessThan(0.2);
        Math.Abs(Angles.Difference(result.Poses[^1].Theta, -Math.PI / 2)).ShouldBeLessThan(0.3);

        // Curvature bound: heading change per meter never exceeds 1/R (with sampling slack).
        for (var i = 1; i < result.Poses.Count; i++)
        {
            var ds = result.Poses[i - 1].DistanceTo(result.Poses[i]);
            if (ds > 0.02)
            {
                (Math.Abs(Angles.Difference(result.Poses[i].Theta, result.Poses[i - 1].Theta)) / ds).ShouldBeLessThan((1 / 0.6) * 1.5);
            }
        }
    }

    [Fact]
    public void Dubins_paths_reach_the_goal_pose()
    {
        var path = Dubins.Sample(new Pose2D(0, 0, 0), new Pose2D(2, 2, Math.PI), 0.5, 0.05)!;
        path.ShouldNotBeNull();
        path[^1].ShouldBe(new Pose2D(2, 2, Math.PI));
        for (var i = 1; i < path.Count; i++)
        {
            path[i - 1].DistanceTo(path[i]).ShouldBeLessThan(0.06);
        }
    }

    [Fact]
    public void Smoother_shortens_zigzags_and_keeps_endpoints()
    {
        var zigzag = Enumerable.Range(0, 20).Select(i => new Pose2D(i * 0.1, i % 2 == 0 ? 0 : 0.1, 0)).ToList();
        var smooth = new PathSmoother().Smooth(zigzag);
        smooth[0].X.ShouldBe(0);
        smooth[^1].X.ShouldBe(1.9, 1e-9);
        PathUtils.Length(smooth).ShouldBeLessThan(PathUtils.Length(zigzag));
    }

    [Fact]
    public void Polygon_footprint_detects_collisions()
    {
        var (_, costmap) = WallWithGap();
        var fp = Footprint.Rectangle(0.6, 0.4);
        fp.InCollision(costmap, new Pose2D(2, 2, 0)).ShouldBeFalse();
        fp.InCollision(costmap, new Pose2D(4.8, 2, 0)).ShouldBeTrue();
        fp.InscribedRadius.ShouldBe(0.2, 1e-9);
    }

    [Fact]
    public void Obstacle_layer_marks_and_clears()
    {
        var map = TestWorlds.Room(6, 6);
        var costmap = Costmap2D.FromMap(map).AddLayer(new StaticLayer(map)).AddLayer(new ObstacleLayer());
        var obstacles = costmap.GetLayer<ObstacleLayer>()!;
        var world = map.Clone();
        world.FillRect(3, 2.9, 3.1, 3.1, OccupancyGridMap.Occupied);
        obstacles.AddScan(costmap, TestWorlds.Scan(world, new Pose2D(1.025, 3.025, 0)), new Pose2D(1.025, 3.025, 0));
        costmap.Update();
        costmap.GetCostAt(3.025, 3.025).ShouldBe(Costs.Lethal);

        // The obstacle moves away: the next scan clears it.
        obstacles.AddScan(costmap, TestWorlds.Scan(map, new Pose2D(1.025, 3.025, 0)), new Pose2D(1.025, 3.025, 0));
        costmap.Update();
        costmap.GetCostAt(3.025, 3.025).ShouldBe(Costs.Free);
    }
}
