#ifndef VECTORVIEW_CONSTANTS_H
#define VECTORVIEW_CONSTANTS_H

// Scale between force intensity and rendered vector length (N^-1).
#define FORCE_SCALE 8E-2

// Minimum force magnitude (N) below which contacts are ignored.
#define NOISE_THRESHOLD 1E-3

// Arrowhead length in world units.
#define ARROW_LENGTH .05

// Headless geometry arrows clamp shaft length while preserving force direction.
#define MAX_GEOMETRY_ARROW_LENGTH 0.35

// Contact sensor and plot update rate (Hz).
#define RATE 25

// Rolling plot window width (s).
#define TIME_MAX 120

#endif
