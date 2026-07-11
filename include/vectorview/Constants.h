#ifndef VECTORVIEW_CONSTANTS_H
#define VECTORVIEW_CONSTANTS_H

// Scale between force intensity and rendered vector length (N^-1).
#define FORCE_SCALE 1.2E-1

// Cap arrow shaft length so contact vectors stay readable in the view.
#define MAX_ARROW_LENGTH 0.22

// Minimum force magnitude (N) below which contacts are ignored.
#define NOISE_THRESHOLD 1E-3

// Arrowhead length in world units.
#define ARROW_LENGTH 0.10

// Minimum visible shaft length when contact force is non-zero.
#define MIN_ARROW_LENGTH 0.08

// Contact sensor and plot update rate (Hz).
#define RATE 25

// Rolling plot window width (s).
#define TIME_MAX 120

#endif
