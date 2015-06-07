class Experience:
    """
    Experience <s, a, s', r>
    """
    def __init__(self):
        self.s = None  # List of numbers
        self.act = None  # Number
        self.reward = None  # Number
        self.next = None  # List of numbers
        self.terminal = None  # Boolean
