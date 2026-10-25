namespace ROS2.Net.Gallery.Web.Services;

/// <summary>One gallery demo: where it lives, what it is called, and the ROS name it is about.</summary>
public sealed record Demo(string Href, string Group, string GroupId, string Title, string TitleId, string RosName, string Summary, string SummaryId);

/// <summary>The gallery catalog, in navigation order.</summary>
public static class Demos
{
    public static IReadOnlyList<Demo> All { get; } =
    [
        new("talker", "Communication", "Komunikasi", "Talker & listener", "Talker & listener", "/chatter",
            "Publish and subscribe std_msgs/String with QoS, the hello world of ROS 2.",
            "Publish dan subscribe std_msgs/String dengan QoS, hello world-nya ROS 2."),
        new("services", "Communication", "Komunikasi", "Services", "Service", "/add_two_ints",
            "Call example_interfaces/AddTwoInts and await the response.",
            "Memanggil example_interfaces/AddTwoInts dan menunggu responsnya dengan await."),
        new("actions", "Communication", "Komunikasi", "Actions", "Action", "/fibonacci",
            "Send a Fibonacci goal, stream feedback, and cancel it half-way.",
            "Mengirim goal Fibonacci, menerima feedback, dan membatalkannya di tengah jalan."),
        new("parameters", "Communication", "Komunikasi", "Parameters", "Parameter", "/gallery/controller",
            "Edit a node's parameters remotely, with ranges and read-only rules enforced.",
            "Mengubah parameter node dari jarak jauh, dengan rentang nilai dan aturan read-only."),
        new("lifecycle", "Communication", "Komunikasi", "Lifecycle node", "Lifecycle node", "/gallery/lc_talker",
            "Drive a managed node through configure, activate and deactivate.",
            "Menjalankan node terkelola melalui configure, activate, dan deactivate."),
        new("teleop", "Mobile robots", "Robot bergerak", "Teleop", "Teleop", "/cmd_vel",
            "Drive the simulated robot with a joystick or W A S D.",
            "Kendalikan robot simulasi dengan joystick atau tombol W A S D."),
        new("lidar", "Mobile robots", "Robot bergerak", "LiDAR & map", "LiDAR & peta", "/scan",
            "LaserScan, occupancy grid, camera and battery, live from the simulator.",
            "LaserScan, occupancy grid, kamera, dan baterai, langsung dari simulator."),
        new("navigation", "Mobile robots", "Robot bergerak", "Navigation", "Navigasi", "/navigate_to_pose",
            "Click the map to send a Nav2 NavigateToPose goal: plan, follow, arrive.",
            "Klik peta untuk mengirim goal NavigateToPose ala Nav2: rencanakan, ikuti, tiba."),
        new("slam", "Mobile robots", "Robot bergerak", "SLAM", "SLAM", "/slam/map",
            "Build a map from LiDAR and odometry while the robot explores.",
            "Membangun peta dari LiDAR dan odometri sambil robot menjelajah."),
        new("tf", "Mobile robots", "Robot bergerak", "TF tree", "Pohon TF", "/tf",
            "The coordinate frames of the robot and how they are chained.",
            "Kerangka koordinat robot dan bagaimana semuanya saling terhubung."),
        new("pointcloud", "Perception & arms", "Persepsi & lengan", "Point clouds", "Point cloud", "/points",
            "Voxel filter, outlier removal, plane segmentation and clustering.",
            "Filter voxel, penghapusan outlier, segmentasi bidang, dan clustering."),
        new("arm", "Perception & arms", "Persepsi & lengan", "Robot arm IK", "IK lengan robot", "/joint_states",
            "Move a target and solve UR5 inverse kinematics in real time.",
            "Geser target dan selesaikan inverse kinematics UR5 secara real time."),
        new("bag", "Data & AI", "Data & AI", "rosbag2", "rosbag2", "bag://mcap",
            "Record /scan and /odom to MCAP, inspect it, and play it back.",
            "Rekam /scan dan /odom ke MCAP, lihat isinya, lalu putar ulang."),
        new("rl", "Data & AI", "Data & AI", "RL agent", "Agen RL", "gym://point-goal",
            "Train a Q-learning agent to reach goals in the simulator.",
            "Melatih agen Q-learning untuk mencapai target di simulator."),
        new("commander", "Data & AI", "Data & AI", "Language commander", "Perintah bahasa", "/nl_command",
            "Type commands in English or Indonesian; a safety limiter executes them.",
            "Ketik perintah dalam bahasa Inggris atau Indonesia; pembatas keselamatan menjalankannya."),
        new("connectivity", "System", "Sistem", "Connectivity", "Konektivitas", "rmw://",
            "The live ROS graph, topic rates, and switching middleware.",
            "Graf ROS langsung, laju topic, dan berpindah middleware."),
    ];

    public static Demo Get(string href) => All.First(d => d.Href == href);
}
