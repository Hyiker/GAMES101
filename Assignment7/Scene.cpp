//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

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
    return Vector3f();
  }

  Intersection inter;
  float pdf_l;
  sampleLight(inter, pdf_l);

  Vector3f l_dir(0, 0, 0);
  Vector3f wo = ray.direction, ws = inter.coords - p.coords;
  if (!intersect(Ray(p.coords, (inter.coords - p.coords).normalized()))
           .happened) {
    l_dir = p.emit * p.m->eval(wo, ws, p.normal) * dotProduct(ws, p.normal) *
            dotProduct(ws, inter.normal) /
            pow((inter.coords - p.coords).norm(), 2) / pdf_l;
  }

  if (dist(gen) > RussianRoulette) {
    return Vector3f();
  }
  auto wi = p.m->sample(wo, p.normal);
  Ray r(p.coords, (wi - p.coords).normalized());
  Intersection indir_inter = intersect(r);
  Vector3f l_indir(0, 0, 0);
  if (indir_inter.happened && !indir_inter.m->hasEmission()) {
    l_indir = castRay(r, depth) * p.m->eval(wo, wi, p.normal) *
              dotProduct(wi, p.normal) / p.m->pdf(wo, wi, p.normal) /
              RussianRoulette;
  }
  return l_dir + l_indir;
}