"""
Differential drive + trailer kinematics.

Robot-trailer modeled as a tractor with a single-axle trailer on a rigid hitch.

State: x, y, theta (robot heading), trailer_phi (hitch angle relative to robot)

Integration at configurable timestep (default 100Hz).
"""
import math


# Physical dimensions (mm)
TRACK_WIDTH = 100.0      # Distance between left and right wheels
ROBOT_LENGTH = 100.0     # Front axle to hitch point
TRAILER_LENGTH = 140.0   # Hitch point to trailer axle
ROBOT_WIDTH = 115.0      # Robot body width
TRAILER_WIDTH = 100.0    # Trailer body width
JACKKNIFE_LIMIT = math.radians(60)  # Max hitch angle


class RobotTrailerState:
    """Full kinematic state of the robot-trailer system."""

    __slots__ = ('x', 'y', 'theta', 'trailer_phi',
                 'left_speed', 'right_speed')

    def __init__(self, x=0.0, y=0.0, theta=0.0, trailer_phi=0.0):
        self.x = x              # Robot center X (mm)
        self.y = y              # Robot center Y (mm)
        self.theta = theta      # Robot heading (radians, 0=right, CCW positive)
        self.trailer_phi = trailer_phi  # Hitch angle (radians, relative to robot)
        self.left_speed = 0.0   # Current left wheel speed (mm/s)
        self.right_speed = 0.0  # Current right wheel speed (mm/s)

    @property
    def hitch_x(self):
        """X position of the hitch point."""
        return self.x - ROBOT_LENGTH * math.cos(self.theta)

    @property
    def hitch_y(self):
        """Y position of the hitch point."""
        return self.y - ROBOT_LENGTH * math.sin(self.theta)

    @property
    def trailer_x(self):
        """X position of the trailer axle center."""
        trailer_heading = self.theta + self.trailer_phi
        return self.hitch_x - TRAILER_LENGTH * math.cos(trailer_heading)

    @property
    def trailer_y(self):
        """Y position of the trailer axle center."""
        trailer_heading = self.theta + self.trailer_phi
        return self.hitch_y - TRAILER_LENGTH * math.sin(trailer_heading)

    @property
    def trailer_heading(self):
        """Absolute heading of the trailer."""
        return self.theta + self.trailer_phi

    def get_robot_corners(self):
        """Get four corners of the robot body for collision/rendering."""
        hw = ROBOT_WIDTH / 2
        # Front-left, front-right, back-right, back-left
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        front_x = self.x + (ROBOT_LENGTH * 0.3) * cos_t
        front_y = self.y + (ROBOT_LENGTH * 0.3) * sin_t
        back_x = self.x - (ROBOT_LENGTH * 0.7) * cos_t
        back_y = self.y - (ROBOT_LENGTH * 0.7) * sin_t
        perp_x = -sin_t * hw
        perp_y = cos_t * hw
        return [
            (front_x + perp_x, front_y + perp_y),  # front-left
            (front_x - perp_x, front_y - perp_y),  # front-right
            (back_x - perp_x, back_y - perp_y),    # back-right
            (back_x + perp_x, back_y + perp_y),    # back-left
        ]

    def get_trailer_corners(self):
        """Get four corners of the trailer body for collision/rendering."""
        hw = TRAILER_WIDTH / 2
        th = self.trailer_heading
        cos_t = math.cos(th)
        sin_t = math.sin(th)
        hx, hy = self.hitch_x, self.hitch_y
        tx, ty = self.trailer_x, self.trailer_y
        perp_x = -sin_t * hw
        perp_y = cos_t * hw
        return [
            (hx + perp_x, hy + perp_y),   # hitch-left
            (hx - perp_x, hy - perp_y),   # hitch-right
            (tx - perp_x, ty - perp_y),   # tail-right
            (tx + perp_x, ty + perp_y),   # tail-left
        ]


def step(state: RobotTrailerState, dt: float):
    """
    Advance the kinematic state by dt seconds (Euler integration).

    Updates state in-place.
    """
    v_left = state.left_speed
    v_right = state.right_speed

    # Differential drive kinematics
    v = (v_left + v_right) / 2.0            # Forward velocity
    omega = (v_right - v_left) / TRACK_WIDTH  # Angular velocity

    # Update robot pose
    state.x += v * math.cos(state.theta) * dt
    state.y += v * math.sin(state.theta) * dt
    state.theta += omega * dt

    # Normalize theta to [-pi, pi]
    state.theta = (state.theta + math.pi) % (2 * math.pi) - math.pi

    # Trailer follows (bicycle model for single-axle trailer)
    phi = state.trailer_phi
    if abs(v) > 0.1 or abs(omega) > 0.1:  # Only update if moving
        phi_dot = (v * math.sin(phi) / TRAILER_LENGTH) - \
                  (omega * ROBOT_LENGTH * math.cos(phi) / TRAILER_LENGTH)
        state.trailer_phi += phi_dot * dt

        # Clamp to jackknife limit
        state.trailer_phi = max(-JACKKNIFE_LIMIT,
                                min(JACKKNIFE_LIMIT, state.trailer_phi))
