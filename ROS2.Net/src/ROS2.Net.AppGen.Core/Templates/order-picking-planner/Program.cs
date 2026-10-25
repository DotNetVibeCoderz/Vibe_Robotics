// {{ProjectName}} — warehouse order-picking tour planner (ROS2.Net template).
using ROS2.Net.Navigation.Costmaps;
using ROS2.Net.Navigation.Maps;
using ROS2.Net.Navigation.Planning;
using ROS2.Net.Simulation;
using NavPose = ROS2.Net.Navigation.Pose2D;

var world = Scenarios.Create(ScenarioKind.Warehouse);
var map = OccupancyGridMap.FromMsg(world.ToOccupancyGrid(0.05));
var costmap = Costmap2D.FromMap(map).AddLayer(new StaticLayer(map)).AddLayer(new InflationLayer(0.5, 3, 0.25));
costmap.Update();

var shelves = world.Waypoints.Where(w => w.Key.StartsWith("shelf_", StringComparison.Ordinal)).ToDictionary(w => w.Key, w => w.Value);
var dock = world.Waypoints["loading_dock"];
var rnd = new Random(11);
var order = shelves.Keys.OrderBy(_ => rnd.Next()).Take(6).ToList();
Console.WriteLine($"Order: {string.Join(", ", order)}");

var planner = new AStarPlanner();
var stops = new List<(string Name, NavPose Pose)> { ("loading_dock", new NavPose(dock.X, dock.Y, dock.Theta)) };
stops.AddRange(order.Select(o => (o, new NavPose(shelves[o].X, shelves[o].Y, shelves[o].Theta))));

// Pairwise travel distances with A* (what the robot would actually drive).
var n = stops.Count;
var dist = new double[n, n];
for (var i = 0; i < n; i++)
{
    for (var j = i + 1; j < n; j++)
    {
        var plan = planner.CreatePlan(costmap, stops[i].Pose, stops[j].Pose);
        dist[i, j] = dist[j, i] = plan.Success ? plan.Length : 1e6;
    }
}

// Nearest neighbor from the dock, then 2-opt improvement.
var tour = new List<int> { 0 };
var left = Enumerable.Range(1, n - 1).ToHashSet();
while (left.Count > 0)
{
    var last = tour[^1];
    var next = left.MinBy(k => dist[last, k]);
    tour.Add(next);
    left.Remove(next);
}

tour.Add(0);
double Length(List<int> t) => Enumerable.Range(0, t.Count - 1).Sum(k => dist[t[k], t[k + 1]]);
var improved = true;
while (improved)
{
    improved = false;
    for (var a = 1; a < tour.Count - 2; a++)
    {
        for (var b = a + 1; b < tour.Count - 1; b++)
        {
            var candidate = tour.Take(a).Concat(tour.Skip(a).Take(b - a + 1).Reverse()).Concat(tour.Skip(b + 1)).ToList();
            if (Length(candidate) + 1e-6 < Length(tour))
            {
                tour = candidate;
                improved = true;
            }
        }
    }
}

const double Speed = 0.6;   // m/s
const double PickTime = 20; // s per pick
Console.WriteLine("\nPick list:");
for (var k = 1; k < tour.Count; k++)
{
    Console.WriteLine($"  {k,2}. {stops[tour[k]].Name,-14} +{dist[tour[k - 1], tour[k]],5:0.0} m");
}

var total = Length(tour);
Console.WriteLine($"\nTour: {total:0.0} m, ≈ {(total / Speed + (order.Count * PickTime)) / 60:0.0} min including picks.");
