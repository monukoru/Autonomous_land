#!/usr/bin/env python3


import rospy
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import math
import time

# MAVROS message types — these are what actually talk to the flight controller
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header


# TUNING PARAMETERS
# Adjust these before flying. Seriously, verify these match your setup.

FLIGHT_ALTITUDE      = 3.0    # meters — hover height before scanning
SCAN_HOVER_TIME      = 3.0    # seconds to hover and let camera stabilise
RANSAC_ITERATIONS    = 1000   # more = better plane, slower detection
RANSAC_THRESHOLD     = 0.05   # meters — point-to-plane inlier tolerance (5cm)
MAX_TILT_ANGLE_DEG   = 13.0   # max allowed surface tilt (ArduPilot default is ~14°)
MIN_LANDING_AREA     = 1.5    # m² — minimum acceptable landing pad size
SQUARE_RATIO_THRESH  = 0.70   # how square the pad needs to be (0=any, 1=perfect square)
APPROACH_SPEED       = 0.5    # m/s — descent speed when moving to landing spot
SAFETY_TIMEOUT       = 30.0   # seconds — abort and RTL if landing takes too long


# RealSense camera helpers

def capture_point_cloud():
    
    #initialises up the RealSense, waits for auto-exposure to settle (takes ~20 frames),
    #grabs a depth + color frame, builds the point cloud, saves it, done.
#
    #Returns an Open3D PointCloud object or None if something went wrong.
    
    rospy.loginfo("[Camera] Initialising RealSense pipeline...")

    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    try:
        pipe.start(cfg)
    except Exception as e:
        rospy.logerr(f"[Camera] Failed to start RealSense: {e}")
        return None

    rospy.loginfo("[Camera] Letting auto-exposure settle (20 frames)...")
    for _ in range(20):
        pipe.wait_for_frames()

    # grab the actual frame we want
    frames       = pipe.wait_for_frames()
    depth_frame  = frames.get_depth_frame()
    color_frame  = frames.get_color_frame()

    if not depth_frame or not color_frame:
        rospy.logerr("[Camera] Got empty frames — is the camera blocked?")
        pipe.stop()
        return None

    # build point cloud
    pc = rs.pointcloud()
    pc.map_to(color_frame)
    points = pc.calculate(depth_frame)
    points.export_to_ply("/tmp/raw_scan.ply", color_frame)

    pipe.stop()
    rospy.loginfo("[Camera] Point cloud captured successfully.")

    # hand it over to Open3D for processing
    pcd = o3d.io.read_point_cloud("/tmp/raw_scan.ply")
    return pcd


# RANSAC landing zone detector  (this is the core of the whole project)

def detect_landing_zone(pcd):
    
    #Takes a raw point cloud and returns the best flat landing spot,
    #or None if nothing acceptable is found.
#
    #Steps:
    #  - Align cloud to its own centroid (makes all the math easier)
    #  - Downsample to reduce noise and speed things up
    #  - Iteratively strip out planes with RANSAC
    #  - For each plane, check tilt and size
    #  - Pick the best candidate
#
    #Returns a dict with keys: center, normal, area, tilt_deg
    
    if len(pcd.points) < 100:
        rospy.logwarn("[RANSAC] Point cloud too sparse — probably a bad scan.")
        return None

    rospy.loginfo(f"[RANSAC] Starting with {len(pcd.points)} points.")

    # centre the cloud — this avoids floating-point drama with big coordinates
    centroid = np.mean(np.asarray(pcd.points), axis=0)
    pcd.translate(-centroid)

    # voxel downsample — 3cm grid keeps enough detail without killing CPU
    pcd_ds = pcd.voxel_down_sample(voxel_size=0.03)
    rospy.loginfo(f"[RANSAC] After downsampling: {len(pcd_ds.points)} points.")

    z_axis         = np.array([0, 0, 1])
    remaining      = pcd_ds
    candidates     = []   # list of valid landing zones we find
    plane_count    = 0

    # keep peeling off planes until we don't have enough points left to bother
    while len(remaining.points) > 150:
        plane_count += 1

        plane_model, inliers = remaining.segment_plane(
            distance_threshold = RANSAC_THRESHOLD,
            ransac_n           = 3,
            num_iterations     = RANSAC_ITERATIONS
        )
        a, b, c, d = plane_model
        normal = np.array([a, b, c])

        # angle between the plane's normal vector and the vertical — tells us how tilted it is
        cos_theta = np.dot(normal, z_axis) / (np.linalg.norm(normal) + 1e-9)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)   # floating point safety
        tilt_deg  = np.degrees(np.arccos(cos_theta))

        rospy.loginfo(f"[RANSAC] Plane {plane_count}: tilt = {tilt_deg:.1f}°, inliers = {len(inliers)}")

        if tilt_deg <= MAX_TILT_ANGLE_DEG:
            # this plane is flat enough — now check if it's big enough and square-ish
            plane_cloud = remaining.select_by_index(inliers)
            obb         = plane_cloud.get_oriented_bounding_box()
            extents     = np.sort(obb.extent)[-2:]    # two largest dimensions
            width, height = extents[0], extents[1]
            area          = width * height
            square_ratio  = min(width, height) / (max(width, height) + 1e-9)

            rospy.loginfo(
                f"[RANSAC]   → Area: {area:.2f} m²  |  Ratio: {square_ratio:.2f}  |  "
                f"Center: ({obb.center[0]:.2f}, {obb.center[1]:.2f}, {obb.center[2]:.2f})"
            )

            if area >= MIN_LANDING_AREA and square_ratio >= SQUARE_RATIO_THRESH:
                candidates.append({
                    "center":      obb.center + centroid,   # un-centre back to world coords
                    "normal":      normal,
                    "area":        area,
                    "tilt_deg":    tilt_deg,
                    "inlier_count": len(inliers)
                })
                rospy.loginfo(f"[RANSAC] Valid landing zone found!")
            else:
                rospy.loginfo(f"[RANSAC] Too small or not square enough — skipping.")
        else:
            rospy.loginfo(f"[RANSAC]  Tilt {tilt_deg:.1f}° exceeds limit — skipping.")

        # strip out this plane and keep searching the rest of the scene
        remaining = remaining.select_by_index(inliers, invert=True)

    if not candidates:
        rospy.logwarn("[RANSAC] No suitable landing zone found in this scan.")
        return None

    # pick the flattest one — in practice you could also pick largest, or closest to centre
    best = min(candidates, key=lambda x: x["tilt_deg"])
    rospy.loginfo(
        f"[RANSAC] Best landing zone: tilt={best['tilt_deg']:.1f}°, area={best['area']:.2f} m²"
    )
    return best


# MAVROS interface — the part that actually talks to ArduPilot

class DroneController:
    
    #Wraps all the MAVROS calls so the main logic above doesn't have to
    #think about ROS topics and services directly.
    

    def __init__(self):
        rospy.loginfo("[FC] Initialising drone controller...")

        self.current_state    = State()
        self.current_pose     = PoseStamped()
        self.home_gps         = None
        self.is_armed         = False

        # subscribe to flight controller state — tells us mode, armed status, etc.
        rospy.Subscriber("/mavros/state",          State,       self._state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self._gps_cb)

        # this topic is how we send position setpoints in GUIDED/OFFBOARD mode
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local",
            PoseStamped,
            queue_size=10
        )

        # wait for MAVROS services to come up — can take a few seconds on real hardware
        rospy.loginfo("[FC] Waiting for MAVROS services...")
        rospy.wait_for_service("/mavros/cmd/arming",   timeout=20)
        rospy.wait_for_service("/mavros/set_mode",     timeout=20)
        rospy.wait_for_service("/mavros/cmd/land",     timeout=20)

        self.arm_srv    = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_srv   = rospy.ServiceProxy("/mavros/set_mode",   SetMode)
        self.land_srv   = rospy.ServiceProxy("/mavros/cmd/land",   CommandTOL)

        rospy.loginfo("[FC] Controller ready.")

    def _state_cb(self, msg):
        self.current_state = msg
        self.is_armed      = msg.armed

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _gps_cb(self, msg):
        if self.home_gps is None:
            self.home_gps = msg
            rospy.loginfo(
                f"[FC] Home GPS locked: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}"
            )

    def get_position(self):
        #Current local position as (x, y, z) in metres from home.
        p = self.current_pose.pose.position
        return np.array([p.x, p.y, p.z])

    def is_connected(self):
        return self.current_state.connected

    def set_guided_mode(self):
        
        #ArduPilot calls it GUIDED,   
        
        rospy.loginfo("[FC] Requesting GUIDED mode...")
        resp = self.mode_srv(custom_mode="GUIDED")
        if resp.mode_sent:
            rospy.loginfo("[FC] GUIDED mode accepted.")
        else:
            rospy.logerr("[FC] Mode change rejected — check your RC switch positions.")
        return resp.mode_sent

    def arm(self):
  
        rospy.loginfo("[FC] Sending arm command...")
        resp = self.arm_srv(True)
        if resp.success:
            rospy.loginfo("[FC] Armed!")
        else:
            rospy.logerr("[FC] Arming failed — check pre-arm conditions in QGroundControl.")
        return resp.success

    def takeoff(self, altitude):
       # 
       # Uses the MAVLink TAKEOFF command — ArduPilot handles the climb itself.
       # altitude is relative to home, in metres.
       # 
        rospy.loginfo(f"[FC] Taking off to {altitude:.1f} m...")
        resp = self.land_srv(min_pitch=0, yaw=0, latitude=0, longitude=0, altitude=altitude)
        # reusing CommandTOL for takeoff (it covers both) — altitude is AGL in metres
        return resp.success

    def go_to_local(self, x, y, z, yaw_deg=0.0, tolerance=0.3):
       # 
       # Fly to a local ENU position (metres from home).
       # Blocks until we're within `tolerance` metres or we timeout.
#
       # This publishes setpoints at 10 Hz — ArduPilot needs a steady stream
       # or it'll switch out of GUIDED mode after ~3 seconds.
        
        rospy.loginfo(f"[FC] Flying to local position: ({x:.2f}, {y:.2f}, {z:.2f})...")

        target        = PoseStamped()
        target.header = Header(frame_id="map")
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z

        rate      = rospy.Rate(10)
        start     = time.time()

        while not rospy.is_shutdown():
            target.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(target)

            # check if we've arrived
            current = self.get_position()
            error   = np.linalg.norm(current - np.array([x, y, z]))

            if error < tolerance:
                rospy.loginfo(f"[FC] Reached target (error = {error:.2f} m).")
                return True

            if time.time() - start > SAFETY_TIMEOUT:
                rospy.logerr(f"[FC] Timeout reaching ({x:.1f}, {y:.1f}, {z:.1f}) — aborting!")
                return False

            rate.sleep()

        return False

    def land_at_current_position(self):
       # 
       # Triggers ArduPilot's built-in LAND mode.
       # The FC handles the descent and motor cutoff — don't try to do that manually.
        
        rospy.loginfo("[FC] Initiating landing sequence (LAND mode)...")
        resp = self.mode_srv(custom_mode="LAND")
        if resp.mode_sent:
            rospy.loginfo("[FC] LAND mode accepted — descending.")
        else:
            rospy.logerr("[FC] LAND mode rejected. Trying RTL as fallback...")
            self.mode_srv(custom_mode="RTL")
        return resp.mode_sent

    def rtl(self):
        """Return to launch — the nuclear option if something goes wrong."""
        rospy.logwarn("[FC] Commanding RTL!")
        self.mode_srv(custom_mode="RTL")



# Main mission logic

def run_landing_mission():
    
    #Full autonomous landing mission:
    #  1. Arm and take off
    #  2. Hover at scan altitude
    #  3. Capture point cloud and run RANSAC
    #  4. Fly to detected landing zone
    #  5. Land

    #If anything goes wrong at any step, we RTL. No heroics.
    
    rospy.init_node("autonomous_landing", anonymous=False)
    rospy.loginfo("=" * 60)
    rospy.loginfo("  Autonomous RANSAC Landing System — starting up")
    rospy.loginfo("=" * 60)

    drone = DroneController()

    # wait until MAVROS actually connects to the flight controller
    rospy.loginfo("[Mission] Waiting for FCU connection...")
    rate = rospy.Rate(2)
    while not rospy.is_shutdown() and not drone.is_connected():
        rate.sleep()
    rospy.loginfo("[Mission] FCU connected!")

    # give the FC a moment to finish its startup sequence
    rospy.sleep(2.0)

    #Step 1: enter GUIDED mode and arm 
    if not drone.set_guided_mode():
        rospy.logerr("[Mission] Could not enter GUIDED mode. Bailing out.")
        return

    rospy.sleep(1.0)

    if not drone.arm():
        rospy.logerr("[Mission] Arming failed. Check pre-arm errors in QGC.")
        return

    rospy.sleep(1.0)

    #Step 2: take off 
    rospy.loginfo(f"[Mission] Taking off to {FLIGHT_ALTITUDE} m...")
    drone.takeoff(FLIGHT_ALTITUDE)
    rospy.sleep(6.0)   # let the drone stabilise — adjust this based on your PID tune

    # keep pumping setpoints during the hover so ArduPilot doesn't time out
    rospy.loginfo(f"[Mission] Hovering for {SCAN_HOVER_TIME}s to let things settle...")
    current_pos = drone.get_position()
    hover_start = time.time()
    pub_rate    = rospy.Rate(10)

    while time.time() - hover_start < SCAN_HOVER_TIME:
        sp              = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "map"
        sp.pose.position.x = current_pos[0]
        sp.pose.position.y = current_pos[1]
        sp.pose.position.z = FLIGHT_ALTITUDE
        drone.setpoint_pub.publish(sp)
        pub_rate.sleep()

    # ── Step 3: scan the scene
    rospy.loginfo("[Mission] Capturing point cloud...")
    pcd = capture_point_cloud()

    if pcd is None:
        rospy.logerr("[Mission] Camera capture failed — RTL.")
        drone.rtl()
        return

    rospy.loginfo("[Mission] Running RANSAC plane detection...")
    zone = detect_landing_zone(pcd)

    if zone is None:
        rospy.logwarn("[Mission] No safe landing zone detected — RTL.")
        drone.rtl()
        return

    # ── Step 4: fly to the landing zone 
    # The RANSAC center is in camera frame (metres from drone body).
    # We add it to the drone's current position to get the target in local ENU.
    # If your camera is mounted at an offset from the FC, add that offset here too.
    cam_center   = zone["center"]
    drone_pos    = drone.get_position()
    target_x     = drone_pos[0] + cam_center[0]
    target_y     = drone_pos[1] + cam_center[1]
    target_z     = FLIGHT_ALTITUDE   # approach at same altitude, land separately

    rospy.loginfo(
        f"[Mission] Moving to landing zone at local ({target_x:.2f}, {target_y:.2f})..."
    )

    success = drone.go_to_local(target_x, target_y, target_z)

    if not success:
        rospy.logerr("[Mission] Could not reach landing zone — RTL.")
        drone.rtl()
        return

    rospy.loginfo("[Mission] Positioned over landing zone. Starting descent...")
    rospy.sleep(1.0)   # quick pause before committing to land

    # Step 5: land 
    drone.land_at_current_position()

    # wait here so the node doesn't exit while the drone is still descending
    rospy.loginfo("[Mission] Landing in progress — monitoring...")
    while not rospy.is_shutdown():
        pos = drone.get_position()
        rospy.loginfo_throttle(2.0, f"[Mission] Altitude: {pos[2]:.2f} m")

        if pos[2] < 0.15:   # basically on the ground
            rospy.loginfo("[Mission] ✓ Touchdown confirmed. Mission complete.")
            break

        rospy.sleep(0.5)


#

if __name__ == "__main__":
    try:
        run_landing_mission()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Mission] Interrupted by user.")
    except Exception as e:
        rospy.logerr(f"[Mission] Unhandled exception: {e}")
        # if we crash here and the drone is flying, someone needs to grab the RC
        rospy.logerr("[Mission] TAKE MANUAL CONTROL IMMEDIATELY if drone is airborne!")
        raise
