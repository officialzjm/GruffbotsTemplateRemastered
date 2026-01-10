The Gruffbots Template is an autonomous template for the VEX V5 Robotics Competition.

Features:

  Position Tracking:
  
    utilizes odometry position tracking with a set of different configurations such as: no_tracker_odom, fwd_horizontal_tracker_odom, and fwd_tracker_odom.
    Includes a distance sensor localization class to predict intersections with field elements.
    Uses a particle filter to combine odometry and distance sensor inputs for a more accurate position estimate.

  Motion Profiling / Path Generation:

    Uses a Quintic Spline generator to convert input waypoints into a continuous smooth function.
    Uses time parametrization to achieve motion profiling through the spline generation.
    Requires a point list that includes the target pose along with the desired time to reach that pose.

  Path Following:

    Uses a Ramsette Feedback Controller, combined with a set of PIDs and Feedforward Controllers
      -The feedforward constants kS, kV, and kA must be tuned for each robot.

Task Scheduler:

    Uses a task scheduler to time parametricize basic autonomous tasks.
    Users must create a task list at the beginning of auton as well as the time they should execute.
    These tasks will then be inputted and executed by the scheduler.
