using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.NavMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Msgs.Tf2Msgs;
using ROS2.Net.Navigation.Localization;
using ROS2.Net.Navigation.Maps;
using ROS2.Net.Navigation.Slam;

namespace ROS2.Net.Navigation.Nodes;

/// <summary>
/// AMCL localization node (Nav2 <c>amcl</c> topics): subscribes <c>/map</c>, <c>/scan</c>,
/// <c>/initialpose</c> and odometry; publishes <c>/amcl_pose</c>, <c>/particle_cloud</c> and the
/// <c>map → odom</c> transform on <c>/tf</c>. <see cref="MapToOdom"/> can feed an
/// <see cref="OdometryPoseProvider.Correction"/> so the rest of the stack sees map-frame poses.
/// </summary>
public sealed class AmclNode : Node
{
    private readonly AmclOptions _options;
    private readonly OdometryPoseProvider _odom;
    private readonly Publisher<PoseWithCovarianceStamped> _posePub;
    private readonly Publisher<PoseArray> _particles;
    private readonly Publisher<TFMessage> _tf;
    private readonly object _gate = new();
    private Amcl? _filter;
    private Pose2D? _pendingInitial;

    public AmclNode(Ros2Context context, AmclOptions? amclOptions = null, string odomTopic = "/odom", Pose2D? initialPose = null, string name = "amcl", NodeOptions? options = null)
        : base(context, name, options)
    {
        _options = amclOptions ?? new AmclOptions();
        GlobalFrame = DeclareParameter("global_frame_id", "map");
        OdomFrame = DeclareParameter("odom_frame_id", "odom");
        _pendingInitial = initialPose;
        _odom = new OdometryPoseProvider(this, odomTopic);
        _posePub = CreatePublisher<PoseWithCovarianceStamped>("/amcl_pose", QosProfile.Latched);
        _particles = CreatePublisher<PoseArray>("/particle_cloud", QosProfile.SensorData);
        _tf = CreatePublisher<TFMessage>("/tf", QosProfile.Default.KeepLast(100));
        CreateSubscription<OccupancyGrid>("/map", OnMap, QosProfile.Latched);
        CreateSubscription<LaserScan>("/scan", OnScan, QosProfile.SensorData);
        CreateSubscription<PoseWithCovarianceStamped>("/initialpose", m => SetInitialPose(Pose2D.FromMsg(m.Pose.Pose)));
    }

    public string GlobalFrame { get; }

    public string OdomFrame { get; }

    /// <summary>Laser pose in the robot frame.</summary>
    public Pose2D LaserOffset { get; set; }

    public Amcl? Filter => _filter;

    public Pose2D MapToOdom { get; private set; }

    /// <summary>Raised after each filter update with the new map→odom correction.</summary>
    public event Action<Pose2D>? CorrectionUpdated;

    public void SetInitialPose(Pose2D pose)
    {
        lock (_gate)
        {
            if (_filter is null)
            {
                _pendingInitial = pose;
                return;
            }

            _filter.Initialize(pose);
        }
    }

    private void OnMap(OccupancyGrid msg)
    {
        lock (_gate)
        {
            var map = OccupancyGridMap.FromMsg(msg);
            if (_filter is null)
            {
                _filter = new Amcl(map, _options);
                if (_pendingInitial is { } p)
                {
                    _filter.Initialize(p);
                }
                else
                {
                    _filter.InitializeGlobal();
                }
            }
            else
            {
                _filter.SetMap(map);
            }
        }
    }

    private void OnScan(LaserScan scan)
    {
        Amcl? filter;
        lock (_gate)
        {
            filter = _filter;
        }

        if (filter is null || !_odom.HasOdometry)
        {
            return;
        }

        var odom = _odom.OdometryPose;
        bool updated;
        lock (_gate)
        {
            updated = filter.Update(odom, scan, LaserOffset);
        }

        if (!updated)
        {
            return;
        }

        var estimate = filter.Estimate;
        MapToOdom = estimate.Compose(odom.Inverse());
        var now = Clock.Now();
        var msg = new PoseWithCovarianceStamped();
        now.CopyTo(msg.Header.Stamp);
        msg.Header.FrameId = GlobalFrame;
        msg.Pose.Pose = estimate.ToMsg();
        var cov = filter.Covariance;
        msg.Pose.Covariance[0] = cov[0];
        msg.Pose.Covariance[1] = cov[1];
        msg.Pose.Covariance[6] = cov[3];
        msg.Pose.Covariance[7] = cov[4];
        msg.Pose.Covariance[35] = cov[8];
        _posePub.Publish(msg);

        var cloud = new PoseArray { Poses = new Pose[filter.ParticleCount] };
        now.CopyTo(cloud.Header.Stamp);
        cloud.Header.FrameId = GlobalFrame;
        for (var i = 0; i < filter.ParticleCount; i++)
        {
            cloud.Poses[i] = filter.GetParticle(i).ToMsg();
        }

        _particles.Publish(cloud);
        _tf.Publish(TfHelpers.ToTfMessage(MapToOdom, GlobalFrame, OdomFrame, now));
        CorrectionUpdated?.Invoke(MapToOdom);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _odom.Dispose();
        }

        base.Dispose(disposing);
    }
}

/// <summary>
/// EKF fusion node (robot_localization <c>ekf_node</c> style): fuses <c>/odom</c> twist and
/// <c>/imu</c> yaw rate into <c>/odometry/filtered</c> at <c>frequency</c> Hz.
/// </summary>
public sealed class EkfNode : Node
{
    private readonly Ekf2D _ekf = new();
    private readonly object _gate = new();
    private readonly Publisher<Odometry> _out;
    private bool _started;

    public EkfNode(Ros2Context context, string odomTopic = "/odom", string imuTopic = "/imu", string name = "ekf_filter_node", NodeOptions? options = null)
        : base(context, name, options)
    {
        Frequency = DeclareParameter("frequency", 30.0);
        OdomFrame = DeclareParameter("odom_frame", "odom");
        BaseFrame = DeclareParameter("base_link_frame", "base_link");
        _out = CreatePublisher<Odometry>("/odometry/filtered");
        CreateSubscription<Odometry>(odomTopic, OnOdom, QosProfile.SensorData.AsReliable());
        CreateSubscription<Imu>(imuTopic, OnImu, QosProfile.SensorData);
        CreateTimer(TimeSpan.FromSeconds(1.0 / Frequency), Publish);
    }

    public double Frequency { get; }

    public string OdomFrame { get; }

    public string BaseFrame { get; }

    public Ekf2D Filter => _ekf;

    private void OnOdom(Odometry msg)
    {
        var t = RosTime.FromMsg(msg.Header.Stamp).Seconds;
        lock (_gate)
        {
            if (!_started)
            {
                _ekf.Reset(Pose2D.FromMsg(msg.Pose.Pose));
                _ekf.PredictTo(t);
                _started = true;
            }

            var cov = msg.Twist.Covariance;
            _ekf.UpdateTwist(t, msg.Twist.Twist.Linear.X, msg.Twist.Twist.Angular.Z, cov[0] > 0 ? cov[0] : 0.01, cov[35] > 0 ? cov[35] : 0.02);
        }
    }

    private void OnImu(Imu msg)
    {
        lock (_gate)
        {
            if (!_started)
            {
                return;
            }

            var cov = msg.AngularVelocityCovariance[8];
            _ekf.UpdateYawRate(RosTime.FromMsg(msg.Header.Stamp).Seconds, msg.AngularVelocity.Z, cov > 0 ? cov : 0.001);
        }
    }

    private void Publish()
    {
        Odometry msg;
        lock (_gate)
        {
            if (!_started)
            {
                return;
            }

            msg = new Odometry { ChildFrameId = BaseFrame };
            msg.Pose.Pose = _ekf.Pose.ToMsg();
            msg.Twist.Twist.Linear.X = _ekf.LinearVelocity;
            msg.Twist.Twist.Angular.Z = _ekf.AngularVelocity;
            var p = _ekf.Covariance;
            msg.Pose.Covariance[0] = p[0];
            msg.Pose.Covariance[7] = p[6];
            msg.Pose.Covariance[35] = p[12];
            msg.Twist.Covariance[0] = p[18];
            msg.Twist.Covariance[35] = p[24];
        }

        Clock.Now().CopyTo(msg.Header.Stamp);
        msg.Header.FrameId = OdomFrame;
        _out.Publish(msg);
    }
}

/// <summary>
/// Online SLAM node (slam_toolbox <c>async_slam_toolbox_node</c> topics): consumes <c>/scan</c> and
/// odometry, publishes <c>/map</c> every <c>map_update_interval</c> seconds and the
/// <c>map → odom</c> correction on <c>/tf</c>.
/// </summary>
public sealed class SlamNode : Node
{
    private readonly OnlineSlam _slam;
    private readonly OdometryPoseProvider _odom;
    private readonly Publisher<OccupancyGrid> _map;
    private readonly Publisher<TFMessage> _tf;
    private readonly object _gate = new();

    public SlamNode(Ros2Context context, SlamOptions? slamOptions = null, string odomTopic = "/odom", string name = "slam_toolbox", NodeOptions? options = null)
        : base(context, name, options)
    {
        _slam = new OnlineSlam(slamOptions);
        MapFrame = DeclareParameter("map_frame", "map");
        OdomFrame = DeclareParameter("odom_frame", "odom");
        var interval = DeclareParameter("map_update_interval", 2.0);
        _odom = new OdometryPoseProvider(this, odomTopic);
        _map = CreatePublisher<OccupancyGrid>("/map", QosProfile.Latched);
        _tf = CreatePublisher<TFMessage>("/tf", QosProfile.Default.KeepLast(100));
        CreateSubscription<LaserScan>("/scan", OnScan, QosProfile.SensorData);
        CreateTimer(TimeSpan.FromSeconds(interval), PublishMap);
    }

    public string MapFrame { get; }

    public string OdomFrame { get; }

    public OnlineSlam Slam => _slam;

    private void OnScan(LaserScan scan)
    {
        if (!_odom.HasOdometry)
        {
            return;
        }

        Pose2D correction;
        lock (_gate)
        {
            _slam.Process(_odom.OdometryPose, scan);
            correction = _slam.MapToOdom;
        }

        _tf.Publish(TfHelpers.ToTfMessage(correction, MapFrame, OdomFrame, Clock.Now()));
    }

    /// <summary>Publishes the current map now.</summary>
    public void PublishMap()
    {
        OccupancyGridMap map;
        lock (_gate)
        {
            if (_slam.Keyframes.Count == 0)
            {
                return;
            }

            map = _slam.GetMap();
        }

        _map.Publish(map.ToMsg(MapFrame, Clock.Now()));
    }

    /// <summary>Saves the map in map_server format (<c>&lt;basePath&gt;.yaml</c>/<c>.pgm</c>).</summary>
    public void SaveMap(string basePath)
    {
        lock (_gate)
        {
            _slam.SaveMap(basePath);
        }
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _odom.Dispose();
        }

        base.Dispose(disposing);
    }
}
