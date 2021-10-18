from dataclasses import dataclass, field


@dataclass(frozen=False, order=True)
class VerticesCoordinates:
    X0: list[float] = field(default_factory=list)
    X1: list[float] = field(default_factory=list)
    X2: list[float] = field(default_factory=list)
    X3: list[float] = field(default_factory=list)
    coordinates_x: list[list[float]] = field(default_factory=list)
    coordinates_y: list[list[float]] = field(default_factory=list)
    coordinates_z: list[list[float]] = field(default_factory=list)
