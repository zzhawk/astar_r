
#include "astar.hpp"
#include "cmath"

namespace pl
{
   void astar::planning()
   {
      _start->visited = true;
      _start->hc = calHeuristic(_start, _goal);
      _openList.push(_start);

      auto search = [&]() -> bool {
         while (!_openList.empty()) {
            grid* cur = _openList.top();
            std::vector<grid*> buff;

            for (auto mot : _mots) {
               int x = cur->x + mot.x;
               int y = cur->y + mot.y;

               if ((x >= _grids[0].size()) || (x < 0) || (y >= _grids.size()) || (y < 0)) continue;

               if (_grids[x][y].obs)  continue;

               if (_grids[x][y].visited) continue;


               grid *nd = &_grids[x][y];
               nd->hc = calHeuristic(_goal, nd);
               nd->parent = cur;
               nd->visited = true;

               if ((x == _goal->x) && (y == _goal->y)) {
                  return true;
               }
               buff.push_back(nd);
            }

            _openList.pop();

            for (int i = 0; i < buff.size(); ++i) {
               _openList.push(buff[i]);
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
      pathExt p;

      auto ptr = _goal;
      while (ptr != nullptr) {
         p.x = (double)ptr->x * _res;
         p.y = (double)ptr->y * _res;
         _path.push_back(p);

         ptr = ptr->parent;
      }
   }

   double astar::calHeuristic(const grid *n1, const grid *n2)
   {
      return _w_hc * std::hypot(n1->x - n2->x, n1->y - n2->y);
   }

   void astar::mapTransformer(gmp::occMap& map)
   {
       _res = map.resolution;
      for (int i = 0; i < map.grids[0].size(); ++i) {
         std::vector<grid> grid_r;
         for (int j = 0; j < map.grids.size(); ++j) {
            int x = (int)(map.grids[j][i].x / _res);
            int y = (int)(map.grids[j][i].y / _res);
            double cost = map.grids[j][i].cost;
            bool obs = map.grids[j][i].obs;
            grid g(x, y, cost, obs);
            grid_r.push_back(g);
         }
         _grids.push_back(grid_r);
      }

      int sx = (int)(map.sx / _res);
      int sy = (int)(map.sy / _res);
      int gx = (int)(map.gx / _res);
      int gy = (int)(map.gy / _res);

      _start = &_grids[sx][sy];
      _goal = &_grids[gx][gy];
   }

   void astar::setMotionModel(void)
   {
      double til = std::sqrt(2.0);

      motion m1(1, 0, 1.0);
      motion m2(0, 1, 1.0);
      motion m3(-1, 0, 1.0);
      motion m4(0, -1, 1.0);

      motion m5(1, 1, til);
      motion m6(1, -1, til);
      motion m7(-1, 1, til);
      motion m8(-1, -1, til);

      _mots.push_back(m1);
      _mots.push_back(m2);
      _mots.push_back(m3);
      _mots.push_back(m4);
      _mots.push_back(m5);
      _mots.push_back(m6);
      _mots.push_back(m7);
      _mots.push_back(m8);
   }


   std::vector<pathExt>& astar::getPath(void)
   {
      return _path;
   }

   std::vector<pathExt> astar::debug_getSearched(void)
   {   
       std::vector<pathExt> ans;

       for (int i = 0; i < _grids.size(); ++i) {
           for (int j = 0; j < _grids[0].size(); ++j) {
               if (_grids[i][j].visited) {
                   pathExt vi;
                   vi.x = _grids[i][j].x * _res;
                   vi.y = _grids[i][j].y * _res;
                   ans.push_back(vi);
               }
           }
       }

       return ans;
   }
}