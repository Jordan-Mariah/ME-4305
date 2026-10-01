

class Share:

    def __init__(self, value = None, value_type = None) -> None:
        self.value = value

        if value_type == None and value != None:
            self.value_type == type(value)
        else:
            self.value_type = value_type


    def isSameType(self, obj) -> bool:
        return type(obj) == self.value_type


    def set(self, set_val):
        if self.isSameType(set_val):
            self.value = set_val
        else:
            raise TypeError(f"Cannot set value of share to {set_val} as it is of type {type(set_val)} and not {self.value_type}")


    def force_set(self, set_val, set_type=False):
        self.value = set_val
        if set_type or self.value_type == None:
            self.value_type = type (set_val)