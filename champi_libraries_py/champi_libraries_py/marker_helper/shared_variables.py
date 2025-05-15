# This is an easy way to implement a singleton pattern in Python. Taken from https://www.geeksforgeeks.org/singleton-pattern-in-python-a-complete-guide/.

# Indicates if visualization is enabled.
visualization_enabled = True

# Index, we use to assign colors to items in a round-robin fashion. It is reset to 0 when canva is cleared.
i_color = 0

