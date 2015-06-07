import random
import math

class Gridworld:
    """
    A representation of a rectangular grid with interior walls that
    allows users easily to determine available directions of travel in
    each cell.
    """
    def __init__(self, height, width, northsouth=None, eastwest=None, rng=random):
        """
        Creates a gridworld using the given wall occupancy matrices.
        :param height:     Height of the gridworld.
        :param width:      Width of the gridworld.
        :param northsouth: 2D list of bools which say whether each interior
                           wall blocking NS movement exists, organized first
                           by [w] columns and then by [h-1] rows.
        :param eastwest:   2D list of bools which say whether each interior
                           wall blocking EW movement exists, organized first
                           by [h] rows and then by [w-1] columns.
        :param rng:        Random number generator to use. Defaults to python's
                           inbuilt random module - used if ns,ew are not given
        """
        self._height = height
        self._width = width
        if northsouth != None and eastwest != None:
            self._ns = northsouth
            self._es = eastwest
        else:
            self._ns = [[False] * (self.height - 1)] * self._width
            self._ew = [[False] * (self.width - 1)] * self._height
            for i in xrange(int(math.sqrt(self._height*self._width))):
                self.add_obstacle(rng)

    @property
    def height(self):
        return self._height

    @property
    def width(self):
        return self._width

    def wall(ns_coord, ew_coord, dir):
        """
        Checks if a wall blocks movement in a given direction from a given
        coordinate.
        :param ns_coord: The coordinate along the NS direction.
        :param ew_coord: The coordinate along the EW direction.
        :param dir:      The direction in which to check movement. 0 is
                         north, 1 is south, 2 is east, 3 is west.
        :return:         True if a wall is presentm False if not present.
        """
        is_ns = 0 == dir / 2
        is_incr = 0 == dir % 2
        if is_ns:
            walls = self._ns
            major = ew_coord
            minor = ns_coord
        else:
            walls = self._ew
            major = ns_coord
            minor = ew_coord

        if not is_incr:
            if minor == 0:
                return True;
            minor = minor - 1

        if minor >= len(walls[major]):
            return True

        return walls[major][minor]

    def __str__(self):
        """
        Creates a string sendable version of the gridworld
        """
        retval = ""
        for i in xrange(len(self._ns)):
            retval += " -"
        retval += " \n"

        for h in xrange(len(self._ew), 0, -1):
            retval += "| "
            for w in xrange(0, len(self._ew[h])):
                retval += "| " if self._ew[h][w] else "  "
            retval += "|\n"

            for w in xrange(0, len(self._ns)):
                retval += " -" if self._ns[w][h-1] else "  "
            retval += " \n"

        retval += "| "
        for w in xrange(0, len(self._ew)):
            retval += "| " if self._ew[0][w] else "  "
        retval += "|\n"

        for i in xrange(0, len(self._ns)):
            retval += " -"
        retval += " \n"

        return retval

    def _add_obstacle(self, rng=random):
        """
        Attempts to add a random wall that must not touch any other
        interior wall.

        :param rng: The random number generator to use.
        """
        direction = rng.random()
        if direction >= 0.5:
            parallel = self._ns
            perpendicular = self._ew
        else:
            parallel = self._ew
            perpendicular = self._ns
        seedi = rng.randint(1, len(parallel)) - 1
        seedj = rng.randint(1, len(parallel[seedi])) - 1

        first = seedi + 1;
        while _is_clear(first - 1, seedj, parallel, perpendicular):
            first = first - 1

        last = seedi
        while _is_clear(last + 1, seedj, parallel, perpendicular):
            last = last + 1

        _choose_segment(first, last, seedj, parallel, rng);


    def _choose_segment(self, first, last, j, parallel, rng=random):
        """
        Given a segment of wall that could be built, builds a subset of
        it of random length. Always builds from one of the two ends.

        :param first:    First index of the segment with no walls.
        :param last:     Last index of the segment with no walls.
        :param j:        Index of the perpendicular axis in the wall grid.
        :param parallel: The wall grid to modify
        :param rng:      Random number generator to use
        """
        if last <= first:
            return
        max_length = last - first;
        if max_length >= len(parallel):
            maxLength = len(parallel) - 1

        if max_length > 1:
            length = rng.randint(1, max_length)
        else:
            length = 1

        dir = 1 - 2 * rng.randint(0, 1)
        if dir > 0:
            start = first
        else:
            start = last - 1

        for i in xrange(length):
            parallel[start + i*dir][j] = True

    def _is_clear(self, i, j, parallel, perpendicular):
        """
        Determines if the "smaller" endpoint of the given line segment
        is clear: none of the four possible walls using that endpoint
        exist.

        :param i:             An index into parallel.
        :param j:             An index into parallel[i].
        :param parallel:      Occupancy matrix for walls parallel to the wall
                              under consideration.
        :param perpendicular: Occupancy matrix for walls perpendicular to
                              the wall under consideration.
        :return:              True if a wall edge doesnt has any walls.
        """
        if i > len(parallel):
            return False
        if i < len(parallel) and parallel[i][j]:
            return False
        if i > 0 and parallel[i - 1][j]:
            return False
        if i > 0 and i <= len(perpendicular[j]) and perpendicular[j][i - 1]:
            return False
        if i > 0 and i <= len(perpendicular[j + 1]) and perpendicular[j + 1][i - 1]:
            return False
        return True
