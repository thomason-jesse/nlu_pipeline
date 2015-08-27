class Action:

    def __init__(self, name=None, params=None):
        self.name = name
        self.params = params

    def __str__(self):
        return self.name+'('+','.join(self.params)+')'

    # assumes elements of params list are atomic
    def __eq__(self, other):
        if self.name != other.name:
            return False
        if len(self.params) != len(other.params):
            return False
        for i in range(0, len(self.params)):
            if self.params[i] != other.params[i]:
                return False
        return True
