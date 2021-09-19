//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

#include <limits>

constexpr float epsilon = std::numeric_limits<float>::epsilon();

void Scene::buildBVH() {
  printf(" - Generating BVH...\n\n");
  this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const {
  return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const {
  float emit_area_sum = 0;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    if (objects[k]->hasEmit()) {
      emit_area_sum += objects[k]->getArea();
    }
  }
  float p = get_random_float() * emit_area_sum;
  emit_area_sum = 0;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    if (objects[k]->hasEmit()) {
      emit_area_sum += objects[k]->getArea();
      if (p <= emit_area_sum) {
        objects[k]->Sample(pos, pdf);
        break;
      }
    }
  }
}

bool Scene::trace(const Ray &ray, const std::vector<Object *> &objects,
                  float &tNear, uint32_t &index, Object **hitObject) {
  *hitObject = nullptr;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    float tNearK = kInfinity;
    uint32_t indexK;
    Vector2f uvK;
    if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
      *hitObject = objects[k];
      tNear = tNearK;
      index = indexK;
    }
  }

  return (*hitObject != nullptr);
}

std::default_random_engine gen;
std::uniform_real_distribution<double> dist(0.0, 1.0);
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const {
  // TODO Implement Path Tracing Algorithm here
  Intersection p = intersect(ray);
  if (!p.happened) {
    return {};
  }

  if (p.m->hasEmission()) {
    return p.m->getEmission();
  }

  Intersection l_inter;
  float pdf_l = 0.0;
  sampleLight(l_inter, pdf_l);

  Vector3f l_dir(0, 0, 0), l_indir(0, 0, 0);
  Vector3f ws = l_inter.coords - p.coords;
  Intersection inter = intersect(Ray(p.coords, ws.normalized()));
  if (inter.distance - ws.norm() > -EPSILON) {
    // the ray hasn't been shielded by object on the way
    Vector3f emit = l_inter.emit,
             f_r = p.m->eval(ray.direction, ws.normalized(), p.normal);
    float cos_theta = dotProduct(ws.normalized(), p.normal),
          cos_theta_x = dotProduct(-ws.normalized(), l_inter.normal);
    l_dir = emit * f_r * cos_theta * cos_theta_x / pow(ws.norm(), 2) / pdf_l;
  }

  if (dist(gen) > RussianRoulette) {
    return l_dir;
  }
  auto wi = p.m->sample(ray.direction, p.normal);
  Ray r(p.coords, wi.normalized());
  auto rec = castRay(r, depth);
  Intersection inn = intersect(r);
  if (inn.happened && !inn.m->hasEmission())
    l_indir = rec * p.m->eval(ray.direction, wi.normalized(), p.normal) *
              dotProduct(wi.normalized(), p.normal) /
              p.m->pdf(ray.direction, wi.normalized(), p.normal) /
              RussianRoulette;
  return l_dir + l_indir;
}