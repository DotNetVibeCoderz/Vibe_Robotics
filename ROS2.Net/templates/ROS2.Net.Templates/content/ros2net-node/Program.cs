// Ros2NetNode — a ROS 2 application built with ROS2.Net (made by Gravicode Studios, led by Kang Fadhil).
//
//   dotnet run                                    # middleware from appsettings.json
//   dotnet run -- --ros-args -p rate_hz:=5.0      # standard ROS arguments work too
//   ros2net topic echo /chatter                   # watch it from another terminal
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using ROS2.Net.Hosting;
using ROS2.Net.Rmw;
using ROS2.Net.Rmw.Rosbridge;
using ROS2.Net.Rmw.Rtps;
using Ros2NetNode;

// Make the network middlewares selectable by name ("Ros2:Rmw" in appsettings.json or ROS2NET_RMW).
RmwRegistry.Register(new RtpsRmwFactory());
RmwRegistry.Register(new RosbridgeRmwFactory());

var builder = Host.CreateApplicationBuilder(args);
builder.Services.AddRos2(builder.Configuration.GetSection("Ros2"), options => options.Arguments.AddRange(args));
builder.Services.AddRos2Node<TalkerNode>();
builder.Services.AddRos2Node<ListenerNode>();

await builder.Build().RunAsync();
