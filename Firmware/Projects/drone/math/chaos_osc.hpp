#pragma once

#include "models.hpp"
#include <cmath>

template <typename M> class ChaosOsc {
private:
  M model;
  typename M::Time dt{};
  typename M::StateType state;

  /// @brief External sampling frequency
  float sampling_frequency; // Hz
  /// @brief Multiplies the intrinsic frequency of the chaotic model by this value
  float freq_multiplier;
  /// @brief Maximum time step allowed for the discretized model
  float max_dt;
  /// @brief Ratio between desired time step and max_dt
  float dt_overshoot;

  void recalculate_params() {
    float new_dt = freq_multiplier / sampling_frequency;

    if (new_dt <= max_dt) {
      dt = new_dt;
      dt_overshoot = 0.0;
    } else {
      dt = max_dt;
      dt_overshoot = (new_dt / max_dt);
    }
  }

public:
  ChaosOsc(M model, typename M::StateType initial_state, float sampling_frequency,
           float freq_multiplier, float max_dt = math::DEFAULT_DT)
      : model{model}
      , state(initial_state)
      , sampling_frequency{sampling_frequency}
      , freq_multiplier{freq_multiplier}
      , max_dt{max_dt}
  {
    recalculate_params();
  }

  void set_sampling_frequency(float new_sampling_frequency) {
    sampling_frequency = new_sampling_frequency;
    recalculate_params();
  }

  void set_frequency_multiplier(float new_frequency_multiplier) {
    freq_multiplier = new_frequency_multiplier;
    recalculate_params();
  }

  void set_max_dt(float new_max_dt) {
    max_dt = new_max_dt;
    recalculate_params();
  }

  void set_model(M new_model) { model = new_model; }

  [[nodiscard]] float get_dt_overshoot() const { return dt_overshoot; }

  const M &get_model() const { return model; }

  M &get_model() { return model; }

  [[nodiscard]] float get_dt() const { return dt; }

  void reset(float new_sampling_frequency, float new_frequency_multiplier,
             float new_max_dt) {
    sampling_frequency = new_sampling_frequency;
    freq_multiplier = new_frequency_multiplier;
    max_dt = new_max_dt;
    recalculate_params();
  }

  typename M::StateType step() {
    // exact match is fine even if it is floating point,
    // since we set it to 0.0 in recalculate_params
    if (dt_overshoot == 0.0) {
      state = model.step(state, dt);
    } else {
      auto overshoot = static_cast<size_t>(truncf(dt_overshoot));

      // step 'overshoot' times using max_dt
      for (size_t k = overshoot; k > 0; k--) {
        state = model.step(state, max_dt);
      }

      // do a single step with the remaining dt
      float remainder_dt = dt_overshoot - static_cast<float>(overshoot) * max_dt;
      state = model.step(state, remainder_dt);
    }

    return state;
  }
};
