#pragma once

using namespace daisy;
using namespace khaos;

/* --- Parameter ranges ------------------------------------------------------------------------- */

/* Useful ranges of the digital model parameters (chaotic zones) */
constexpr float ROSSLER_A_MIN = 0.02f, ROSSLER_A_MAX = 0.45f;
constexpr float ROSSLER_B_MIN = 0.05f, ROSSLER_B_MAX = 1.00f;
constexpr float ROSSLER_C_MIN = 1.00f, ROSSLER_C_MAX = 15.00f;
constexpr float HALVORSEN_A_MIN = 1.00f, HALVORSEN_A_MAX = 2.60f;
constexpr float SPROTT_A_MIN = 0.0f, SPROTT_A_MAX = 4.00f;
constexpr float SPROTT_B_MIN = 0.0f, SPROTT_B_MAX = 4.00f;
/* Encoder step per detent, for each parameter group */
constexpr float STEP_A = 0.01f;
constexpr float STEP_B = 0.01f;
constexpr float STEP_C = 0.10f;
/* Depth of the CV modulation, as a fraction of the parameter range.
A full-scale CV shifts the parameter by CV_DEPTH * (max - min). */
constexpr float CV_DEPTH = 0.5f;
/* Digipot: 8-bit rheostat, R_WB = R/N with N in 0x000..0x100 (257 positions) */
constexpr uint16_t DIGIPOT_FULL_SCALE = 0x100;
constexpr size_t DIGIPOT_NUM_WIPERS = 4;
/* Wiper -> parameter mapping: adapt to the actual PCB wiring */
constexpr digipot::Wiper WIPER_ANALOG1_A = digipot::Wiper::Wiper0;
constexpr digipot::Wiper WIPER_ANALOG1_B = digipot::Wiper::Wiper1;
constexpr digipot::Wiper WIPER_ANALOG2_A = digipot::Wiper::Wiper2;
constexpr digipot::Wiper WIPER_ANALOG2_B = digipot::Wiper::Wiper3;
/* DAC remapping scales: conservative estimates of the attractor amplitude,
to be tuned with an oscilloscope. The remap centers the value around
half scale and clamps to the 12-bit DAC range. */
constexpr float ROSSLER_SCALE = 300.0f; // attractor roughly within +-20
constexpr float HALVORSEN_SCALE = 150.0f; // attractor roughly within +-25
constexpr float SPROTT_SCALE = 400.0f;