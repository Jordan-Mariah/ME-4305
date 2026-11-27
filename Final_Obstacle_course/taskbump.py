"""Skeletal bump reader; pins and electrical configuration come from main.py."""


class TaskBump:
    def __init__(self, pins, bump_status, active_level, debounce_ms=20):
        self.pins = pins
        self.bump_status = bump_status
        self.active_level = active_level
        self.debounce_ms = debounce_ms
        # TODO: independent stable/candidate levels and timers for each pin.

    def run(self):
        while True:
            # TODO: poll every pin without blocking.
            # TODO: debounce both press and release independently per pin
            # TODO: publish a complete snapshot, for example:
            # {pressed: bool, contacts: tuple, press_id: int, time_ms: int}.
            # TODO: increment press_id only for a newly accepted contact event.
            # TODO: course uses contact during wall approach, unexpected contact
            # elsewhere needs a separately defined stop 
            yield 0
