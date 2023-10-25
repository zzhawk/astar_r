// Copyright 2023 watson.wang
// https://github.com/zzhawk/

#include "astar.hpp"
#include "cmath"

namespace pl
{
   void astar::planning()
   {
      setStartNode();
      setGoalNode();

      auto search = [&]() -> bool {
         while (!_openList.empty()) {
            node* cur = _openList.top();
            _openList.pop();
            cur->status = NodeStatus::Closed;

            const auto index_theta = discretizeAngle(cur->p.t);
            for (auto &mot : _mots[index_theta]) {
               index idx = getNewIdx(cur, mot);

               if (isGoal(cur, idx)) {
                   return true;
               }

               if ((idx.x >= _map.grids[0].size()) || (idx.x < 0) || (idx.y >= _map.grids.size()) || (idx.y < 0)) continue;

               if (hitObstacle(cur, idx, mot))  continue;

               if (_graph.find(getKey(idx)) != _graph.end()) continue;

               fillData(getNodeRef(idx), idx, cur, mot);
            }
         }
         return false;
      };

      if (search()) {
         genFinalPath();
      }

   }

   void astar::genFinalPath()
   {
      auto ptr = _goal;
      while (ptr != nullptr) {
         _path.push_back(ptr->p);
         ptr = ptr->parent;
      }
   }

   void astar::setStartNode(void)
   {
      gmp::pose p = {_map.s.x, _map.s.y, _map.s.t};
      _start = getNodeRef(pose2index(p));
      _start->p = p;
      _start->hc = _w_hc * std::hypot(_map.s.x - _map.g.x, _map.s.y - _map.g.y);
      _start->gc = getGrid(_map.s.x, _map.s.y).cost;
      _start->status = NodeStatus::Open;

      _openList.push(_start);
   }

   void astar::setGoalNode(void)
   {
      gmp::pose p = {_map.g.x, _map.g.y, _map.g.t};
      /* theta */
      _goal = getNodeRef(pose2index(p));
      _goal->p = p;
   }

   double astar::calHeuristic(const node *n1)
   {
      return _w_hc * std::hypot(n1->p.x - _goal->p.x, n1->p.y - _goal->p.y);
   }


   motions astar::setMotionModel(void)
   {
      motions mots;
      std::vector<motion> buff;

      buff.push_back({ 1.0, 0.0 });
      buff.push_back({ 0.0, 1.0 });
      buff.push_back({ -1.0, 0.0 });
      buff.push_back({ 0.0, -1.0 });
      buff.push_back({ 1.0, 1.0 });
      buff.push_back({ 1.0, -1.0 });
      buff.push_back({ -1.0, 1.0 });
      buff.push_back({ -1.0, -1.0 });

      mots.push_back(buff);

      return mots;
   }


   std::vector<gmp::pose>& astar::getPath(void)
   {
      return _path;
   }

   std::vector<gmp::pose> astar::debug_getSearched(void)
   {   
       std::vector<gmp::pose> ans;

       for (auto& it : _graph) {
          ans.push_back(it.second.p);
       }

       return ans;
   }

   node* astar::getNodeRef(const index& idx)
   {
      return &(_graph.emplace(getKey(idx), node()).first->second);
   }

   inline int astar::getKey(const index& idx)
   {
      return (idx.t + (idx.y * x_scale_ + idx.x) * y_scale_);
   }

   int astar::discretizeAngle(const double theta) 
   { 
      return 0; 
   }

   index astar::getNewIdx(node* cur, motion& mot)
   {
      index idx;
      idx.x = getIdx(cur->p.x) + static_cast<int>(mot.shift_x);
      idx.y = getIdx(cur->p.y) + static_cast<int>(mot.shift_y);
      idx.t = getIdx(cur->p.t) + static_cast<int>(mot.shift_theta);

      return idx;
   }

   index astar::pose2index(gmp::pose p) 
   {
      const int x = static_cast<int>(p.x / _map.resolution);
      const int y = static_cast<int>(p.y / _map.resolution);
      const int t = 0;
      return { x , y, t };
   }

   gmp::pose astar::index2pose(index idx) 
   {
      const double x = static_cast<double>(idx.x) * _map.resolution;
      const double y = static_cast<double>(idx.y) * _map.resolution;
      const double t = 0.0;
      return { x , y, t };
   }

   inline int astar::getIdx(double d) 
   {
      return static_cast<int>(d / _map.resolution);
   }

   gmp::grid astar::getGrid(double x, double y) 
   {
      return _map.grids[getIdx(x)][getIdx(y)];
   }

   void astar::fillData(node* nd, index &idx, node* cur, motion& mot)
   {
      nd->p.x = _map.grids[idx.y][idx.x].x;
      nd->p.y = _map.grids[idx.y][idx.x].y;
      nd->hc = calHeuristic(nd);
      nd->gc = _map.grids[idx.y][idx.x].cost;
      nd->parent = cur;
      nd->status = NodeStatus::Open;
      _openList.push(nd);
   }

   bool astar::isGoal(node* cur, index& idx)
   {
       if ((idx.x == getIdx(_goal->p.x)) && (idx.y == getIdx(_goal->p.y))) {
           _goal->parent = cur;
           return true;
       }
       return false;
   }

   bool astar::hitObstacle(node* cur, index& idx, motion& mot)
   {
      return _map.grids[idx.y][idx.x].obs;
   }
}