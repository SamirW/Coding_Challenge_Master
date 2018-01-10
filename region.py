class Region():
    def __init__(self, points=None):
        self._points = points

        if points is None:
            self._points = []

    def __repr__(self):
        return "<Region (id: %s)| points: %s, line_segments: %s>" % (id(self), self.points, len(self.line_segments()))

    @property
    def points(self):
        return self._points

    def add_point(self, point):
        self.points.append((point))

    def line_segments(self):
        line_segments = []
        for i in range(len(self.points)):
            line_segments.append([self.points[i-1], self.points[i]])
        return line_segments

class Obstacle(Region):
    def __init__(self, points=None):
        Region.__init__(self, points=None)
        self._discovered = False

    def __repr__(self):
        return "<Obstacle (id: %s)| discovered: %r, points: %s, line_segments: %s>" % (id(self), self.discovered, self.points, 
            len(self.line_segments()))

    @property
    def discovered(self):
        return self._discovered