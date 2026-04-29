#pragma once

#include <functional>

#include "vecmath.hpp"

namespace math {
/** Default integration step used by continuous models */
constexpr float DEFAULT_DT = 0.01f;

/**
 * @brief Runge-Kutta 4 implementation for N-valued float vectors.
 */
template <size_t N>
vec<N, float> rk4(vec<N, float> x, std::function<vec<N, float>(vec<N, float>)> dx, float dt) {
    // Runge-Kutta coefficients
    vec<N, float> k1, k2, k3, k4;
    k1 = dx(x);
    k2 = dx(x + dt * k1 / 2.0f);
    k3 = dx(x + dt * k2 / 2.0f);
    k4 = dx(x + dt * k3);

    return x + dt * (k1 + 2.0f * k2 + 2.0f * k3 + k4) / 6.0f;
}

/**
 * @brief Discrete chaotic model interface
 * Discrete models extend this class, providing an implementation for `step()`.
 *
 * @tparam T type of the internal state (e.g. `vec<3, float>`, `vec<2, double>`, etc.);
 * T must provide a `constexpr size()` for the default implementation of `step()` and a
 * `T::Base` type (the underlying component type).
 */
template <class T> class DiscreteModel {
  public:
    using StateType = T;

    /**
     * @brief Advances the system by one iteration.
     * @return the next state after one iteration
     */
    virtual T step(T state) const = 0;

    T step(T state, size_t iterations) {
        for (size_t i = 0; i < iterations; i++) {
            state = step(state);
        }
        return state;
    }
};

/**
 * @brief Continuous chaotic model interface
 *
 * Classes which implement this interface need to provide an implementation for `gradient()`;
 * `step()` computes the next step using the gradient. This interface provides a default
 * implementation for `step()`.
 *
 * @tparam T type of the internal state (e.g. `vec<3, float>`, `vec<2, double>`, etc.);
 * T must provide a `constexpr size()` for the default implementation of `step()` and a
 * `T::Base` type (the underlying component type).
 */
template <class T> class ContinuousModel {
  public:
    using StateType = T;
    using Time = typename T::Base;

    /**
     * @brief Calculates the gradient of the state vector at a point.
     *
     * To compute the trajectory of the system, this gradient has to be
     * integrated (e.g. using RK4 or explicit Euler).
     */
    virtual T gradient(T state) const = 0;

    /**
     * @brief Integrates `gradient()` using Runge-Kutta 4 by default
     */
    virtual T step(T state, Time dt) const {
        auto grad = [this](T state) { return this->gradient(state); };

        return rk4<T::size()>(state, grad, dt);
    }
};

template <class M> class DiscretizedModel : public DiscreteModel<typename M::StateType> {
  public:
    using StateType = typename M::StateType;
    using Time = typename M::StateType::Base;

    M model;
    Time dt;

    DiscretizedModel(const M &model, Time dt) : model(model), dt(dt) {}

    // Calls the underlying continuous model with a certain dt
    StateType step(StateType state) const override { return model.step(state, dt); }
};

// -- Discrete oscillators --

/**
 * Henon oscillator (discrete, 2D)
 * Useful initial conditions: (x0, y0) = (0.1, 0.3)
 */
class Henon : public DiscreteModel<vec2f> {
  public:
    float a = 1.14;
    float b = 0.3;

    vec2f step(vec2f state) const override;
};

class Ikeda : public DiscreteModel<vec2f> {
  public:
    float u = 0.9;
    float k = 0.4;
    float p = 6.0;

    vec2f step(vec2f) const override;
};

// -- Continuous oscillators --

/**
 * Chua oscillator (continuous, 3D)
 * Useful initial conditions (x0, y0, z0) = (0.1, 0, 0)
 */
class Chua : public ContinuousModel<vec3f> {
  public:
    float alpha = 18.39f, beta = 39.0f, m0 = -1.143, m1 = -0.714;

    float chua_diode(float x) const;
    vec3f gradient(vec3f state) const override;
};

class Sprott : public ContinuousModel<vec3f> {
  public:
    float a = 2.07, b = 1.79;

    vec3f gradient(vec3f) const override;
};

class Rossler : public ContinuousModel<vec3f> {
  public:
    float a = 0.2, b = 0.2, c = 5.7;

    vec3f gradient(vec3f) const override;
};

class Halvorsen : public ContinuousModel<vec3f> {
  public:
    float a = 1.89;

    vec3f gradient(vec3f) const override;
};

class Lorentz : public ContinuousModel<vec3f> {
  public:
    float rho = 10;
    float sigma = 10;
    float beta = 8.f / 3.f;

    vec3f gradient(vec3f) const override;
};
} // namespace math
