"""Skeletal blue-button input pins still need to add in main"""


class TaskButton:
    POLL = 0
    DEBOUNCE = 1

    def __init__(self, pin, start_event, active_level, debounce_ms=20):
        self.pin = pin
        self.start_event = start_event  # Suggested Share(int): press counter.
        self.active_level = active_level
        self.debounce_ms = debounce_ms
        self.state = self.POLL
        # TODO: stable level, candidate level, candidate start time.
        # TODO: define startup behavior if button is already held.

    def run(self):
        while True:
            if self.state == self.POLL:
                pass  # TODO: read pin; start debounce when level changes.
            elif self.state == self.DEBOUNCE:
                pass  # TODO: accept stable change after debounce_ms.
                # Increment press counter only on a new accept
                # A held button must not produce repeated start events.
            yield self.state
