"""Skeletal taskcourse"""


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
        self.state = state
        self.substate = 0

    def request_maneuver(self, mode, **parameters):
        """TODO: increment ID and publish request."""
        raise NotImplementedError("still need maneuver requests")

    def maneuver_complete(self):
        """TODO: require matching request ID ompleted status."""
        raise NotImplementedError("Implement completion checks")

    def run(self):
        while True:
            # TODO: publish course state/substate without excessive printing.
            yield self.state
