from dataclasses import dataclass


@dataclass(frozen=True)
class Interval:
    begin: int
    end: int
    data: object = None


class IntervalTree:
    def __init__(self):
        self._intervals = []

    def add(self, interval):
        self._intervals.append(interval)

    def __getitem__(self, point):
        return [interval for interval in self._intervals if interval.begin <= point < interval.end]
