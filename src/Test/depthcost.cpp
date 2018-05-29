//#include "ENC_Polygonize.h"
#include "ogrsf_frmts.h" // GDAL
#include "geodesy.h"
#include <iostream>
#include <string>
#include <cmath> // For sqrt and pow
#include <vector>

using namespace std;

int calcAveDepth(vector<vector<int> > &grid, int wptX,int wptY, int i);
void NeighborsMask(int Connecting_Dist, vector<int> &dx, vector<int> &dy);
int posORneg(int num);

int main(int argc, char* argv[])
{
    vector<vector<int> > grid(6,vector<int>(6,1));
    /*
    cout << "53" << endl;
    calcAveDepth(grid, 0,0,53);*/
    cout << "\n\n27"<< endl;
    calcAveDepth(grid, 2,3,27);
//    cout << "\n\n27"<< endl;
//    calcAveDepth(grid, 2,3,27);
//    cout << "\n\n0"<< endl;
//    calcAveDepth(grid, 2,3,0);
//    cout << "\n\n31"<< endl;
//    calcAveDepth(grid, 2,3,31);
    return 0;
}

int calcAveDepth(vector<vector<int> > &grid, int wptX,int wptY, int i)
{
    vector<int> dx, dy;
    NeighborsMask(3, dx, dy);

    double M, alpha, beta;
    int depth_cutoff = 0;

    int x1,y1,x2,y2;
    int accumDepth=0;
    int absX = abs(dx[i]);
    int absY = abs(dy[i]);
    int farestPoint_in_XY = max(absX, absY);
    int signX = posORneg(dx[i]);
    int signY = posORneg(dy[i]);

//    for (int index=0; index<dx.size(); index++)
//        cout << index << " ->\tdx: " << dx[index] << ", dy: " << dy[index] << endl;

    cout << endl <<"dx: " << dx[i] << ", dy: " << dy[i] << " (" << dx[i]+wptX << ", " << dy[i]+wptY<<")"<<endl;
    // Check to see if it is not a Moore Neighboor. If it is then return the depth of the desired cell.
    if (farestPoint_in_XY==1)
        accumDepth = grid[wptY+dy[i]][wptY+dx[i]];
    // Otherwise calculate the average depth of the cells that a line would pass between the waypoints.
    else
    {
        for (int index=1; index<=farestPoint_in_XY; index++)
        {
            if (absY<absX)
            {
                M = (1.0*dy[i])/(1.0*dx[i])*(1.0*index);
                x1 = index*signX;
                x2 = x1;
                // Make sure that you use the correct sign for 'Y's if dx and dy have the same sign
                if (signX==signY)
                    y1 = static_cast<int>(M)*signY;
                else
                    y1 = static_cast<int>(M);
                y2 = y1+signY;
            }
            else
            {
                M = (1.0*dx[i])/(1.0*dy[i])*(1.0*index);
                // Make sure that you use the correct sign for the 'X's if dx and dy have the same sign
                if (signX==signY)
                    x1 = static_cast<int>(M)*signX;
                else
                    x1 = static_cast<int>(M);
                x2 = x1+signX;
                y1 = index*signY;
                y2 = y1;
                cout << index << ", " << signX << ", " << x1<< endl;
            }
            beta = abs(fmod(M, 1.0));
            alpha = 1-beta;

            //printf("%d, %0.2f -> %0.2f * G[%i][%i] + %0.2f * G[%i][%i]\n", index,M,alpha, wptY+y1, wptX+x1,beta,wptY+y2, wptX+x2);

            // Make sure than neither of the two cells that you are passing through have a
            //if ((grid[wptY+y1][wptX+x1] < depth_cutoff)||grid[wptY+y2][wptX+x2] < depth_cutoff)
                //return;

            // If you are not at the last cell, then average the two cells and store that value. Otherwise store the value for the final cell.
            if (index!=farestPoint_in_XY)
                accumDepth += alpha*grid[wptY+y1][wptX+x1] + beta*grid[wptY+y2][wptX+x2];
            else
                accumDepth += alpha*grid[wptY+y1][wptX+x1];
        }
    }
    return accumDepth/farestPoint_in_XY;
}

void NeighborsMask(int Connecting_Dist, vector<int> &dx, vector<int> &dy)
{
  int twice = 2*Connecting_Dist;
  int Mid = Connecting_Dist;
  int r_size = 2*Connecting_Dist+1;

  // Mask of the desired neighbors
  vector<vector<int>> new_neighbors (r_size, vector<int>(r_size,1));

  // Remove the positions that are the same directions
  for (int i=0; i<Connecting_Dist-1; i++)
    {
      new_neighbors[i][i]=0;
      new_neighbors[twice-i][i]=0;
      new_neighbors[i][twice-i]=0;
      new_neighbors[twice-i][twice-i]=0;
      new_neighbors[Mid][i]=0;
      new_neighbors[Mid][twice-i]=0;
      new_neighbors[i][Mid]=0;
      new_neighbors[twice-i][Mid]=0;
    }
  new_neighbors[Mid][Mid]=0;

  // Find the locations of the mask for the new neighbors
  for (int i=0; i<r_size; i++)
    {
      for (int j=0; j<r_size; j++)
        {
          if (new_neighbors[i][j] == 1)
            {
              dy.push_back(i-Connecting_Dist);
              dx.push_back(j-Connecting_Dist);
            }
        }
    }
}

int posORneg(int num)
{
    if (num<0)
        return -1;
    else
        return 1;
}
