#include "rope.h"

#include <iostream>
#include <vector>

#include "CGL/vector2D.h"
#include "mass.h"
#include "spring.h"

namespace CGL {

constexpr float damping_factor = 0.00005f;
Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass,
           float k, vector<int> pinned_nodes) {
  // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and
  // containing `num_nodes` nodes.

  //        Comment-in this part when you implement the constructor
  Vector2D step = (end - start) / (num_nodes - 1);
  for (int i = 0; i < num_nodes; i++) {
    masses.push_back(new Mass(start + step * i, node_mass, false));
    if (i != 0) {
      springs.push_back(new Spring(masses[i - 1], masses[i], k));
    }
  }
  for (auto &i : pinned_nodes) {
    masses[i]->pinned = true;
  }
}

void Rope::simulateEuler(float delta_t, Vector2D gravity) {
  for (auto &s : springs) {
    // TODO (Part 2): Use Hooke's law to calculate the force on a node
    Vector2D vec = (s->m2->position - s->m1->position);
    Vector2D force1 = s->k * vec / vec.norm() * (vec.norm() - s->rest_length);
    s->m1->forces += force1;
    s->m2->forces -= force1;
  }

  constexpr bool explicit_method = false;
  for (auto &m : masses) {
    if (!m->pinned) {
      // TODO (Part 2): Add the force due to gravity, then compute the new
      // velocity and position
      m->forces += gravity * m->mass;
      // x(t+1) = x(t) + v(t)*dt
      // semi-implicit
      if (!explicit_method) m->velocity += m->forces / m->mass * delta_t;

      // TODO (Part 2): Add global damping
      m->position += m->velocity * delta_t;
      // x(t+1) = x(t) + v(t+1)*dt
      // explicit
      if (explicit_method) m->velocity += m->forces / m->mass * delta_t;
    }

    // Reset all forces on each mass
    m->forces = Vector2D(0, 0);
  }
}

void Rope::simulateVerlet(float delta_t, Vector2D gravity) {
  for (auto &s : springs) {
    // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet
    // （solving constraints)
    Vector2D vec = s->m2->position - s->m1->position;
    Vector2D force1 = s->k * vec.unit() * (vec.norm() - s->rest_length);
    s->m1->forces += force1;
    s->m2->forces -= force1;
  }

  for (auto &m : masses) {
    if (!m->pinned) {
      Vector2D temp_position = m->position;
      // TODO (Part 3.1): Set the new position of the rope mass
      m->forces += gravity * m->mass;
      m->position += (1 - damping_factor) * (m->position - m->last_position) +
                     m->forces / m->mass * delta_t * delta_t;

      m->last_position = temp_position;
      // TODO (Part 4): Add global Verlet damping
    }
    m->forces = Vector2D(0, 0);
  }
}
}  // namespace CGL
