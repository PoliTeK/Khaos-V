#pragma once

#include "models.hpp"
#include <daisy_seed.h>

template <typename M> class ChaosOsc {
private:
  /// @brief External sampling frequency
  float sampling_frequency; // Hz
  /// @brief Multiplies the intrinsic frequency of the chaotic model by this
  /// value
  float freq_multiplier;
  /// @brief Maximum time step allowed for the discretized model
  float max_dt;
  /// @brief Ratio between desired time step and max_dt
  float dt_overshoot;

  void recalculate_params() {
    float new_dt = freq_multiplier / sampling_frequency;

    if (new_dt <= max_dt) {
      model.dt = new_dt;
      dt_overshoot = 0.0;
    } else {
      model.dt = max_dt;
      dt_overshoot = (new_dt / max_dt);
    }
  }

public:
  math::DiscretizedModel<M> model;
  typename M::StateType state;

  ChaosOsc(M model, typename M::StateType initial_state, float sampling_frequency,
           float freq_multiplier, float max_dt = math::DEFAULT_DT)
      : sampling_frequency(sampling_frequency),
        freq_multiplier(freq_multiplier), max_dt(max_dt), model{model, 0.0f},
        state(initial_state) {
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

  void set_model(M new_model) { model.model = new_model; }

  [[nodiscard]] float get_dt_overshoot() const { return dt_overshoot; }

  const M &get_model() const { return model.model; }

  M &get_model() { return model.model; }

  [[nodiscard]] float get_dt() const { return model.dt; }

  void reset(float new_sampling_frequency, float new_frequency_multiplier,
             float new_max_dt) {
    sampling_frequency = new_sampling_frequency;
    freq_multiplier = new_frequency_multiplier;
    max_dt = new_max_dt;
    recalculate_params();
  }

  typename M::StateType step() {
    if (dt_overshoot == 0.0) {
      state = model.step(state);
    } else {
      auto overshoot = static_cast<size_t>(truncf(dt_overshoot));

      // step 'overshoot' times using max_dt
      for (size_t k = overshoot; k > 0; k--) {
        state = model.step(state);
      }

      // do a single step with the remaining dt
      float remainder_dt = dt_overshoot - static_cast<float>(overshoot) * max_dt;
      model.dt = remainder_dt;
      state = model.step(state);
      model.dt = max_dt;
    }

    return state;
  }
};
