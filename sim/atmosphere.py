"""
Atmosphere density models.
All inputs/outputs in SI units (metres, kg/m³).
"""
import math

_G0 = 9.80665  # standard gravity (m/s²)


def density(model: str, h: float) -> float:
    """
    Return air density (kg/m³) at altitude h (m).

    Models
    ------
    'isa'  : ICAO standard atmosphere (piecewise)
    'exp'  : simple exponential with scale height 8500 m
    'none' : vacuum (ρ = 0)
    """
    if model == 'none':
        return 0.0

    if model == 'exp':
        return 1.225 * math.exp(-h / 8_500.0)

    # ISA standard atmosphere
    if h > 86_000:
        return 0.0
    if h > 20_000:
        T = 216.65
        p = 5_474.89 * math.exp(-_G0 * (h - 20_000) / (287.0 * T))
        return p / (287.0 * T)
    if h > 11_000:
        T = 216.65
        p = 22_632.1 * math.exp(-_G0 * (h - 11_000) / (287.0 * T))
        return p / (287.0 * T)

    T = 288.15 - 0.0065 * h
    p = 101_325.0 * (T / 288.15) ** 5.2561
    return p / (287.0 * T)
