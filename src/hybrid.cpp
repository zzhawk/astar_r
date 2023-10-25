// Copyright 2023 watson.wang
// https://github.com/zzhawk/

#include "hybrid.hpp"
#include "reeds_shepp.hpp"

namespace pl
{
   using rs = freespace_planning_algorithms::ReedsSheppStateSpace;

    motions hybrid::setMotionModel(void)
	{
        // Vehicle moving for each angle
        motions mots;
        mots.resize(_cfg.THETA_SIZE);

        const double dtheta = 2.0 * M_PI / _cfg.THETA_SIZE;

        // Minimum moving distance with one state update
        const double arc = 1.9 * _map.resolution;
        const auto& R_min = _cfg.MIN_TURN_RADIUS;
        const auto& R_max = _cfg.MAX_TURN_RADIUS;
        //const double step_min = R_min * dtheta;
        const double dR = (R_max - R_min) / std::max(_cfg.TURN_RADIUS_SIZE - 1, 1);

        // motion actions
        std::vector<motion> forward_node_candidates;
        const motion forward_straight{ arc, 0.0, 0.0, arc, 0.0, false, false };
        forward_node_candidates.push_back(forward_straight);
        for (int i = 0; i < _cfg.TURN_RADIUS_SIZE; ++i) {
            double R = R_min + i * dR;
            double theta = arc / R;
            const motion forward_left{
                R * sin(theta), R * (1 - cos(theta)), theta, arc, R, true, false };
            const motion forward_right = forward_left.flipped();
            forward_node_candidates.push_back(forward_left);
            forward_node_candidates.push_back(forward_right);
        }

        for (int i = 0; i < _cfg.THETA_SIZE; i++) {
            const double theta = dtheta * i;

            for (const auto& nu : forward_node_candidates) {
               mots[i].push_back(nu.rotated(theta));
            }

            if (_cfg.USE_BACK) {
                for (const auto& nu : forward_node_candidates) {
                   mots[i].push_back(nu.reversed().rotated(theta));
                }
            }
        }

        return mots;
	}


    double hybrid::calMoveCost(const node* n1, const node* n2, motion& mot)
    {

        double cost = 0.0;

        if (mot.is_back) cost += _cfg.SB_COST;
        cost += _cfg.STEER_COST * abs(mot.shift_theta);
        cost += _cfg.STEER_CHANGE_COST * abs(n1->p.t - n2->p.t);
        cost += mot.distance;
        return cost;
    }

   double hybrid::normalizeRadian(const double rad, const double min_rad, const double max_rad)
   {
      int i = 0;
      const auto value = std::fmod(rad, 2 * M_PI);
      if (min_rad < value && value <= max_rad) {
         return value;
      }
      else {
         return value - std::copysign(2 * M_PI, value);
      }
   }

   int hybrid::discretizeAngle(const double theta)
   {
      const double one_angle_range = 2.0 * M_PI / _cfg.THETA_SIZE;
      auto idx = static_cast<int>(std::rint(normalizeRadian(theta, 0.0) / one_angle_range)) % _cfg.THETA_SIZE;
      if (idx < 0) { idx += _cfg.THETA_SIZE; }
      return idx;
   }

   index hybrid::pose2index(gmp::pose p) 
   {
      const int x = static_cast<int>(p.x / _map.resolution);
      const int y = static_cast<int>(p.y / _map.resolution);
      const int t = discretizeAngle(p.t);
      return { x , y, t };
   }

   gmp::pose hybrid::index2pose(index idx)
   {
      const double x = static_cast<double>(idx.x) * _map.resolution;
      const double y = static_cast<double>(idx.y) * _map.resolution;

      const double one_angle_range = 2.0 * M_PI / _cfg.THETA_SIZE;
      const double t = static_cast<double>(idx.t) * one_angle_range;
      return { x , y, t };
   }

   void hybrid::fillData(node* nd, index& idx, node* cur, motion& mot)
   {
      nd->status = NodeStatus::Open;
      nd->p.x = cur->p.x + mot.shift_x;
      nd->p.y = cur->p.y + mot.shift_y;
      nd->p.t = cur->p.t + mot.shift_theta;
      nd->gc = cur->gc + calMoveCost(cur, nd, mot);
      nd->hc = calHeuristic(nd);
      nd->is_back = mot.is_back;
      nd->parent = cur;
      _openList.push(nd);
   }

   double hybrid::calHeuristic(const node* n1)
   {
      const double radius = (_cfg.MIN_TURN_RADIUS + _cfg.MAX_TURN_RADIUS) * 0.5;
      return _w_hc * calcReedsSheppDistance(n1->p, _goal->p, radius);
   }

   bool hybrid::isGoal(node* cur, index& idx)
   {
      const double radius = (_cfg.MIN_TURN_RADIUS + _cfg.MAX_TURN_RADIUS) * 0.5;
      auto path = calcReedsSheppPath(cur->p, _goal->p, radius);

      /* need improve */
      for (auto &pth : path) {
         int x = getIdx(pth.x);
         int y = getIdx(pth.y);
         int t = getIdx(pth.t);

         if ((x >= _map.grids[0].size()) || (x < 0) || (y >= _map.grids.size()) || (y < 0)) return false;
         if (_map.grids[y][x].obs) return false;
      }

      _path = path;
      std::reverse(_path.begin(),_path.end());
      _goal->parent = cur;

       return true;

   }

   index hybrid::getNewIdx(node* cur, motion& mot)
   {
      int x = getIdx(cur->p.x) + static_cast<int>(mot.shift_x / _map.resolution);
      int y = getIdx(cur->p.y) + static_cast<int>(mot.shift_y / _map.resolution);
      int t = getIdx(cur->p.t) + discretizeAngle(mot.shift_theta);

      return { x, y, t };
   }

   bool hybrid::hitObstacle(node* cur, index& idx, motion& mot)
   {
      /* need improve */
      index cidx;
      cidx.x = getIdx(cur->p.x);
      cidx.y = getIdx(cur->p.y);

      int dx = abs(cidx.x - idx.x);
      int dy = abs(cidx.y - idx.y);

      for (int i = 0; i <= dy; ++i) {
         for (int j = 0; j <= dx; ++j) {
            auto y = std::min(idx.y, cidx.y) + i;
            auto x = std::min(idx.x, cidx.x) + j;
            if ((x >= _map.grids[0].size()) || (x < 0) || (y >= _map.grids.size()) || (y < 0)) continue;
            if (_map.grids[y][x].obs) return true;
         }
      }
      return false;
   }

   void hybrid::genFinalPath()
   {
      auto ptr = _goal;
      ptr = ptr->parent;
      ptr = ptr->parent;
      while (ptr != nullptr) {
         _path.push_back(ptr->p);
         ptr = ptr->parent;
      }
   }

   double hybrid::calcReedsSheppDistance(const gmp::pose& p1, const gmp::pose& p2, double radius)
   {
      const auto rs_space = rs(radius);
      const rs::StateXYT pose0{p1.x, p1.y, p1.t };
      const rs::StateXYT pose1{p1.x, p2.y, p2.t };
      return rs_space.distance(pose0, pose1);
   }

   std::vector<gmp::pose> hybrid::calcReedsSheppPath(const gmp::pose& p1, const gmp::pose& p2, double radius)
   {
      std::vector<gmp::pose> pout;
      const auto rs_space = rs(radius);
      const rs::StateXYT pose0{ p1.x, p1.y, p1.t };
      const rs::StateXYT pose1{ p2.x, p2.y, p2.t };
      auto path = rs_space.reedsShepp(pose0, pose1);

      double seg = 0;
      auto p = pose0;
      int iter = static_cast<int>(path.totalLength_* radius / _path_res)+1;
      for (int i = 0; i <= iter; ++i) {
         p = rs_space.interpolate(pose0, path, static_cast<double>(i)*_path_res / radius);
         pout.push_back({ p.x, p.y, p.yaw});
      }

      return pout;
   }
}