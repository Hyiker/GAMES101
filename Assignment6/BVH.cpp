#include "BVH.hpp"

#include <algorithm>
#include <cassert>

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)),
      splitMethod(splitMethod),
      primitives(std::move(p)) {
  time_t start, stop;
  time(&start);
  if (primitives.empty()) return;

  root = recursiveBuild(primitives);

  time(&stop);
  double diff = difftime(stop, start);
  int hrs = (int)diff / 3600;
  int mins = ((int)diff / 60) - (hrs * 60);
  int secs = (int)diff - (hrs * 3600) - (mins * 60);

  printf(
      "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
      hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects) {
  BVHBuildNode* node = new BVHBuildNode();

  // Compute bounds of all primitives in BVH node
  Bounds3 bounds;
  for (int i = 0; i < objects.size(); ++i)
    bounds = Union(bounds, objects[i]->getBounds());
  if (objects.size() == 1) {
    // Create leaf _BVHBuildNode_
    node->bounds = objects[0]->getBounds();
    node->object = objects[0];
    node->left = nullptr;
    node->right = nullptr;
    return node;
  } else if (objects.size() == 2) {
    node->left = recursiveBuild(std::vector{objects[0]});
    node->right = recursiveBuild(std::vector{objects[1]});

    node->bounds = Union(node->left->bounds, node->right->bounds);
    return node;
  } else {
    Bounds3 centroidBounds;
    for (int i = 0; i < objects.size(); ++i)
      centroidBounds =
          Union(centroidBounds, objects[i]->getBounds().Centroid());
    int dim = centroidBounds.maxExtent();
    switch (dim) {
      case 0:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
          return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;
        });
        break;
      case 1:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
          return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;
        });
        break;
      case 2:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
          return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;
        });
        break;
    }

    auto beginning = objects.begin();
    std::vector<Object*>::iterator middling;
    if (splitMethod == SplitMethod::NAIVE) {
      middling = objects.begin() + (objects.size() / 2);
    } else {
      int s = 0;
      float cost_min = 1.0 / 0.0;
      for (int i = 1; i < objects.size(); i++) {
        // split objects[i] into the left collection.
        auto obj_bd = objects[i]->getBounds();
        float cost;
        switch (dim) {
          case 0:
            cost = i * (obj_bd.pMin.x - bounds.pMin.x) +
                   (objects.size() - i) * (bounds.pMax.x - obj_bd.pMin.x);
            break;
          case 1:
            cost = i * (obj_bd.pMin.y - bounds.pMin.y) +
                   (objects.size() - i) * (bounds.pMax.y - obj_bd.pMin.y);
            break;
          case 2:
            cost = i * (obj_bd.pMin.z - bounds.pMin.z) +
                   (objects.size() - i) * (bounds.pMax.z - obj_bd.pMin.z);
            break;
        }
        if (cost < cost_min) {
          s = i;
          cost_min = cost;
        }
      }
      middling = objects.begin() + s;
    }
    auto ending = objects.end();

    auto leftshapes = std::vector<Object*>(beginning, middling);
    auto rightshapes = std::vector<Object*>(middling, ending);

    assert(objects.size() == (leftshapes.size() + rightshapes.size()));

    node->left = recursiveBuild(leftshapes);
    node->right = recursiveBuild(rightshapes);

    node->bounds = Union(node->left->bounds, node->right->bounds);
  }

  return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const {
  Intersection isect;
  if (!root) return isect;
  isect = BVHAccel::getIntersection(root, ray);
  return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node,
                                       const Ray& ray) const {
  auto dir = ray.direction;
  // TODO Traverse the BVH to find intersection
  if (!node || !node->bounds.IntersectP(
                   ray, ray.direction_inv,
                   {int(dir.x > 0), int(dir.y > 0), int(dir.z > 0)})) {
    return Intersection();
  }
  if (!node->left && !node->right) {
    // leaf node
    return node->object->getIntersection(ray);
  }
  auto hit1 = getIntersection(node->left, ray);
  auto hit2 = getIntersection(node->right, ray);
  return hit1.distance < hit2.distance ? hit1 : hit2;
}