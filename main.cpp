#include <fstream>
#include "astar.hpp"


int main()
{

   std::ofstream mpf("../map.csv");
   std::ofstream plf("../plan.csv");
   std::ofstream srf("../search.csv");

   gmp::occMap themap;

   themap.sx = 10.0;
   themap.sy = 10.0;
   themap.gx = 50.0;
   themap.gy = 50.0;

   themap.length = 60.0;
   themap.width = 60.0;
   themap.resolution = 1.0;

   for (int i = 0; i < (int)(themap.length / themap.resolution); ++i) {
      std::vector<gmp::grid> buf;
      for (int j = 0; j < (int)(themap.width / themap.resolution); ++j) {
         gmp::grid g((double)j, (double)i, 0.0, false);
         buf.push_back(g);
      }
      themap.grids.push_back(buf);
   }

   for (int i = 0; i < 40; ++i) {
      themap.grids[i][19].obs = true;
      themap.grids[59-i][39].obs = true;
   }

   for (int i = 0; i < themap.grids.size(); ++i) {
       for (int j = 0; j < themap.grids[0].size(); ++j) {
           mpf << themap.grids[j][i].x << "," << themap.grids[j][i].y << "," 
               << themap.grids[j][i].cost << "," << themap.grids[j][i].obs 
               << std::endl;
       }
   }

   pl::astar astar_plan(themap);
   astar_plan.planning();

   auto path = astar_plan.getPath();
   for (int i = 0; i < path.size(); ++i) {
       plf << path[i].x << "," << path[i].y << std::endl;
   }

   auto searched = astar_plan.debug_getSearched();
   for (int i = 0; i < searched.size(); ++i) {
       srf << searched[i].x << "," << searched[i].y << std::endl;
   }

   return 0;
}