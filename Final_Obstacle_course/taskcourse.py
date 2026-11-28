"""Skeletal taskcourse"""
import time
from time import ticks_diff, ticks_add

class TaskCourse:
    IDLE = 0
    PRE_LANE = 1
    LANE_CHANGE = 2
    GARAGE_APPROACH = 3
    GARAGE_ENTRY = 4
    GARAGE_TRAVERSE = 5
    GARAGE_EXIT = 6
    CUP_MOVE = 7
    LOOP_FIRST_HALF = 8
    LOOP_SECOND_HALF = 9
    SLALOM_APPROACH = 10
    SLALOM = 11
    REVERSE_PARK = 12
    VICTORY = 13
    FINISHED = 14
    FAULT = 15

    def __init__(self, start_event, bump_status, sensor_data,
                 left_position_cm, right_position_cm,
                 maneuver_request, maneuver_status, course_status, stop_latch):
        self.start_event = start_event
        self.bump_status = bump_status
        self.sensor_data = sensor_data
        self.left_position_cm = left_position_cm
        self.right_position_cm = right_position_cm
        self.maneuver_request = maneuver_request
        self.maneuver_status = maneuver_status
        self.course_status = course_status
        self.stop_latch = stop_latch
        self.state = self.IDLE
        self.substate = 0
        self.request_id = 0
        # TODO: entry time, encoder origins, heading origin, accumulated yaw.

    def enter_state(self, state):
        """TODO: capture references and send one request on state entry."""
        self.start_position_right = self.right_position_cm.value
        self.start_position_left = self.left_position_cm.value #starting positions so that we can get displacement

        if self.sensor_data.value != None:
            self.start_heading = self.sensor_data.value["imu"][0]

        else:
            self.start_heading = None

        self.start_time = time.ticks_us()

        self.state = state
        self.substate = 0

    def request_maneuver(self, mode, distance, angle, speed):
        """TODO: increment ID and publish request."""

        self.maneuver_request.value["request_id"] += 1
        self.maneuver_request.value["mode"] = mode
        self.maneuver_request.value["distance"] = distance
        self.maneuver_request.value["angle"] = angle
        self.maneuver_request.value["speed"] = speed


    def maneuver_complete(self):
        """TODO: require matching request ID ompleted status."""
        raise NotImplementedError("Implement completion checks")

    def run(self):
        while True:
            if self.state == self.IDLE:
                pass  # TODO: wait for a new start event.
            elif self.state == self.PRE_LANE:
                pass  # TODO: follow the line to the lane-change section.
            elif self.state == self.LANE_CHANGE:
                pass  # TODO: perform the lane change.
            elif self.state == self.GARAGE_APPROACH:
                pass  # TODO: approach the garage.
            elif self.state == self.GARAGE_ENTRY:
                pass  # TODO: enter the garage.
            elif self.state == self.GARAGE_TRAVERSE:
                pass  # TODO: traverse the garage.
            elif self.state == self.GARAGE_EXIT:
                pass  # TODO: exit the garage.
            elif self.state == self.CUP_MOVE:
                pass  # TODO: move the cup.
            elif self.state == self.LOOP_FIRST_HALF:
                pass  # TODO: complete the first half of the loop.
            elif self.state == self.LOOP_SECOND_HALF:
                pass  # TODO: complete the second half of the loop.
            elif self.state == self.SLALOM_APPROACH:
                pass  # TODO: approach the slalom.
            elif self.state == self.SLALOM:
                pass  # TODO: complete the slalom.
            elif self.state == self.REVERSE_PARK:
                pass  # TODO: reverse into the parking space.
            elif self.state == self.VICTORY:
                pass  # TODO: perform the victory maneuver.
            elif self.state == self.FINISHED:
                pass  # TODO: hold position after completing the course.
            elif self.state == self.FAULT:
                pass  # TODO: stop and wait for fault recovery.
            # TODO: publish course state/substate without excessive printing.
            yield self.state
